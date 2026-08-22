//
// A minimal MMC5983MA for the Flapjack SIL: the SPI framing, an 8-bit register
// file, and the registers whose *values* mmc5983.c actually checks. Enough for
// Mmc5983_HwInit to complete and for Mmc5983_Read to stream field samples.
//
// Not modelled: the temperature channel, the set/reset coil, the interrupt
// pins, the selftest thresholds, the ODR pacing. Config registers read back
// whatever was written, so the driver's writes are accepted without comment.
//
// Load with `include @Scripts/renode/MMC5983.cs` before the platform
// description that instantiates it, and after SamplePushListener.cs.
//
// Payload, 12 bytes, little endian: field x,y,z float32 in GAUSS, MAG die
// frame. Gauss rather than a unit vector because the part has a real full
// scale (+/-8 G over 18 bits) and mmc5983.c divides by it - feeding a unit
// vector would quietly assert that Earth's field saturates the part. Nothing
// downstream cares about the magnitude: MadgwickFilter_Update_9DOF_
// renormalises the vector before using it.
//
// Chip select is NOT wired to this model. It hangs off an SPIMultiplexer, which
// is constructed with suppressExplicitFinishTransmission and therefore swallows
// the FinishTransmission that STM32H7_SPI raises at the end of every TSIZE
// burst - i.e. in the middle of a register access - and forwards exactly one,
// when CS is released. So FinishTransmission here is a true transaction
// delimiter, which is the job BMI323.cs has to do from the CS GPIO itself.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class MMC5983 : ISPIPeripheral, IDisposable
    {
        // port 0 leaves the socket off: the register file is then whatever the
        // firmware last wrote, which is all a boot smoke test needs.
        public MMC5983(int port = 0)
        {
            registers = new byte[RegisterCount];
            Reset();
            source = new SamplePushListener(this, port, PayloadLength);
        }

        public void Dispose()
        {
            source.Dispose();
        }

        public void Reset()
        {
            ResetRegisters();
            expectAddress = true;
            Transactions = 0;
            Bytes = 0;
        }

        public byte Transmit(byte data)
        {
            Bytes++;

            if(expectAddress)
            {
                address = (uint)(data & AddressMask);
                reading = (data & ReadFlag) != 0;
                expectAddress = false;
                byteIndex = 0;
                // One snapshot per transaction, so the seventh byte's shared low
                // bits can never belong to a different measurement than the six
                // above them.
                activeSample = source.Latest;
                activeCounts = (activeSample == null) ? null : Encode(activeSample.Payload);
                Transactions++;
                return 0;
            }

            // No dummy byte: an MMC5983 read returns register contents starting
            // with the very next byte, which is why mmc5983.c uses
            // SpiDev_ReadRegister directly instead of bmp390.c's offset buffer.
            if(reading)
            {
                return Read(address + (uint)byteIndex++);
            }

            Write(address + (uint)byteIndex++, data);
            return 0;
        }

        // The multiplexer forwards this once per chip-select release, and also
        // fires it on every already-idle line whenever any other line moves, so
        // it has to be idempotent.
        public void FinishTransmission()
        {
            expectAddress = true;
        }

        // Chip-select assertions and bytes clocked, for costing the SPI path.
        public long Transactions { get; private set; }
        public long Bytes { get; private set; }

        private byte Read(uint register)
        {
            if(register >= RegisterCount)
            {
                this.Log(LogLevel.Warning, "Read from out-of-range register 0x{0:X2}", register);
                return 0;
            }

            var sample = activeSample;
            if(sample == null)
            {
                // A socket was asked for but nothing has pushed yet. Report no
                // data rather than serving mid-scale, which would read as a zero
                // field and give the heading a direction it has not measured.
                if(source.Active && register == StatusRegister)
                {
                    return 0;
                }
                return registers[register];
            }

            if(register == StatusRegister)
            {
                return (byte)(sample.Sequence != consumedSequence ? MeasMDone : 0);
            }

            if(register < DataLength)
            {
                // The driver reads STATUS and then the whole 7-byte block, so
                // consuming on the first data byte of the pass is enough. A real
                // part clears Meas_M_Done when the output registers are read.
                consumedSequence = sample.Sequence;
                return activeCounts[register];
            }

            return registers[register];
        }

        private void Write(uint register, byte value)
        {
            if(register >= RegisterCount)
            {
                this.Log(LogLevel.Warning, "Write of 0x{0:X2} to out-of-range register 0x{1:X2}", value, register);
                return;
            }

            if(register == IntCtrl1Register && (value & SwReset) != 0)
            {
                ResetRegisters();
                return;
            }

            registers[register] = value;
        }

        private void ResetRegisters()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[ProductIdRegister] = ProductId;
            // Meas_M_Done. Mmc5983_Read reports eSTATUS_BUSY while this is clear,
            // so a register file with nothing pushing must answer "ready" or
            // Mag_Task publishes nothing at all. Once the bridge pushes, this
            // becomes "a new sample arrived since the last data read".
            registers[StatusRegister] = MeasMDone;
        }

        // Gauss in, 18-bit counts out, laid out the way the part puts them on
        // the wire: high byte, low byte, then the shared low-2-bit byte.
        private byte[] Encode(byte[] payload)
        {
            var counts = new byte[DataLength];
            var low = 0;
            for(var i = 0; i < 3; i++)
            {
                var raw = ToCounts(BitConverter.ToSingle(payload, i * 4));
                counts[i * 2] = (byte)(raw >> 10);
                counts[i * 2 + 1] = (byte)((raw >> 2) & 0xFF);
                // X occupies bits 7:6, Y 5:4, Z 3:2 of the shared byte.
                low |= (raw & 0x3) << (6 - i * 2);
            }
            counts[6] = (byte)low;
            return counts;
        }

        // Unsigned 18-bit with zero field at mid-scale, so the midpoint is both
        // the offset and the full-scale divisor - the same pair mmc5983.c undoes.
        private static int ToCounts(float gauss)
        {
            var counts = (int)Math.Round(gauss * MidScale / FullScaleGauss) + MidScale;
            return Math.Max(0, Math.Min(MaxCount, counts));
        }

        private long consumedSequence;
        private PushedSample activeSample;
        private byte[] activeCounts;

        private bool expectAddress;
        private bool reading;
        private uint address;
        private int byteIndex;

        private readonly byte[] registers;
        private readonly SamplePushListener source;

        private const int RegisterCount = 0x30;
        private const byte AddressMask = 0x7F;
        private const byte ReadFlag = 0x80;

        private const int PayloadLength = 3 * 4;
        private const uint DataLength = 7;      // X/Y/Z high+low, then their shared low-2-bit byte

        private const int MidScale = 1 << 17;
        private const int MaxCount = (1 << 18) - 1;
        private const float FullScaleGauss = 8f;

        private const uint StatusRegister = 0x08;
        private const uint IntCtrl1Register = 0x0A;
        private const uint ProductIdRegister = 0x2F;

        private const byte MeasMDone = 1 << 0;
        private const byte SwReset = 1 << 7;
        private const byte ProductId = 0x30;
    }
}
