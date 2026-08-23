//
// A minimal BMP390 for the Flapjack SIL: the SPI framing, an 8-bit register
// file, a calibration NVM block, and the registers whose *values* bmp390.c
// actually checks. Enough for Bmp390_HwInit to complete and for Bmp390_Read to
// stream compensated pressure and temperature.
//
// Not modelled: FIFO, interrupts, the IIR filter, ODR pacing, conf_err. Config
// registers read back whatever was written, so the driver's writes are accepted
// without comment.
//
// Load with `include @Scripts/renode/BMP390.cs` before the platform description
// that instantiates it, and after SamplePushListener.cs.
//
// Payload, 8 bytes, little endian: pressure float32 in Pa, temperature float32
// in degrees C.
//
// THE INTERESTING PART is that this model does not hand the driver a pressure.
// It publishes a realistic 21-byte calibration NVM block and then INVERTS
// datasheet 8.5/8.6 to find the raw ADC counts that bmp390.c will compensate
// back into the pressure the bridge asked for. That is the whole point of
// emulating the part rather than injecting at the driver: the coefficient
// decode in BaroReadCalibration - fourteen little-endian words of assorted
// widths and signs, each scaled by its own power of two - is exercised for
// real, and a wrong width or sign shows up as a pressure that is out by
// kilopascals instead of passing silently.
//
// The coefficient set below is representative of a real part rather than
// degenerate: every one of the fourteen is non-zero and in its datasheet
// magnitude range, chosen so the mapping stays monotone and 50-115 kPa lands
// inside the 24-bit ADC range with room to spare. Verified round trip over
// -20..60 C and 50..115 kPa, against the driver's float32 arithmetic: worst
// error 0.016 Pa, which is float32 rounding rather than quantisation.
//
// Sensor NOISE is deliberately not here - JSBSim's SensorBaro system already
// models it (2 Pa noise + 20 Pa bias). This model contributes quantisation
// only, so the two error sources stay separable.
//
// Chip select is NOT wired to this model; it hangs off an SPIMultiplexer that
// delivers one FinishTransmission per CS release. See MMC5983.cs for why.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class BMP390 : ISPIPeripheral, IDisposable
    {
        // port 0 leaves the socket off: the register file is then whatever the
        // firmware last wrote, which is all a boot smoke test needs.
        public BMP390(IMachine machine, int port = 0)
        {
            this.machine = machine;
            IRQ = new GPIO();
            registers = new byte[RegisterCount];
            Reset();
            source = new SamplePushListener(this, port, PayloadLength, OnSamplePushed);
        }

        // INT pin. Connect it to the pin the board wires it to - PF10 on
        // flapjack_v1 (BARO_INT_GPIO_* in target/flapjack_v1/flapjack_v1.h).
        public GPIO IRQ { get; private set; }

        public void Dispose()
        {
            source.Dispose();
        }

        public void Reset()
        {
            ResetRegisters();
            IRQ.Unset();
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
                pendingDummy = reading;
                expectAddress = false;
                byteIndex = 0;
                // One snapshot per transaction. The part only shadows its data
                // registers for the length of a single read (datasheet 3.10.1),
                // which is the behaviour bmp390.c's single-burst read relies on.
                activeSample = source.Latest;
                activeCounts = (activeSample == null) ? null : Encode(activeSample.Payload);
                Transactions++;
                return 0;
            }

            if(reading)
            {
                // A BMP390 SPI read emits one dummy byte ahead of the register
                // contents; bmp390.c allows for it with SPI_DUMMY_BYTES = 1.
                if(pendingDummy)
                {
                    pendingDummy = false;
                    return 0;
                }
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
                // data rather than serving zeros: a zero raw count compensates
                // to a pressure nowhere near an atmosphere, and Nav would take
                // its altitude datum from it.
                if(source.Active && register == StatusRegister)
                {
                    return 0;
                }
                return registers[register];
            }

            if(register == StatusRegister)
            {
                return (byte)(sample.Sequence != consumedSequence ? DrdyPress : 0);
            }

            if(register >= Data0Register && register < Data0Register + DataLength)
            {
                // The driver reads STATUS and then the whole 6-byte block, so
                // consuming on the first data byte of the pass is enough. A real
                // part clears drdy_press when the data registers are read.
                // Same moment the interrupt is answered; this runs on the
                // emulation thread, so the pin can be touched directly.
                consumedSequence = sample.Sequence;
                IRQ.Unset();
                return activeCounts[register - Data0Register];
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

            if(register == CmdRegister && value == SoftResetCommand)
            {
                ResetRegisters();
                return;
            }

            registers[register] = value;
        }

        private void ResetRegisters()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[ChipIdRegister] = ChipId;
            // drdy_press. Bmp390_Read reports eSTATUS_BUSY while this is clear,
            // so a register file with nothing pushing must answer "ready" or
            // Baro_Task publishes nothing at all. Once the bridge pushes, this
            // becomes "a new sample arrived since the last data read".
            registers[StatusRegister] = DrdyPress;

            // The calibration block survives a soft reset on the real part, and
            // bmp390.c reads it after one.
            for(var i = 0; i < CalibrationNvm.Length; i++)
            {
                registers[NvmParT1Register + (uint)i] = CalibrationNvm[i];
            }
        }

        // Pa and degrees C in, raw ADC counts out - datasheet 8.5/8.6 run
        // backwards. Six bytes: pressure then temperature, 24 bit, LSB first.
        private byte[] Encode(byte[] payload)
        {
            var pressurePa = BitConverter.ToSingle(payload, 0);
            var temperatureC = BitConverter.ToSingle(payload, 4);

            var rawTemp = InvertTemperature(temperatureC);
            // Invert pressure against the tLin the driver will actually derive
            // from the QUANTISED raw temperature, not against the temperature
            // the bridge asked for. Otherwise the two sides disagree by the
            // temperature LSB, and the pressure carries that error.
            var rawPress = InvertPressure(pressurePa, CompensateTemperature(rawTemp));

            var counts = new byte[DataLength];
            counts[0] = (byte)rawPress;
            counts[1] = (byte)(rawPress >> 8);
            counts[2] = (byte)(rawPress >> 16);
            counts[3] = (byte)rawTemp;
            counts[4] = (byte)(rawTemp >> 8);
            counts[5] = (byte)(rawTemp >> 16);
            return counts;
        }

        private static double CompensateTemperature(int rawTemp)
        {
            var partial = rawTemp - T1;
            return partial * T2 + partial * partial * T3;
        }

        private static double CompensatePressure(double rawPress, double tLin)
        {
            var out1 = P5 + P6 * tLin + P7 * tLin * tLin + P8 * tLin * tLin * tLin;
            var out2 = rawPress * (P1 + P2 * tLin + P3 * tLin * tLin + P4 * tLin * tLin * tLin);
            var out3 = rawPress * rawPress * (P9 + P10 * tLin) + rawPress * rawPress * rawPress * P11;
            return out1 + out2 + out3;
        }

        // tLin is quadratic in (rawTemp - T1), so this inverts in closed form.
        // Written as 2c/(b + sqrt(b^2 - 4ac)) rather than the schoolbook root:
        // T3 is ~1e-14 against a T2 of ~1e-5, so the usual form subtracts two
        // nearly equal numbers and throws away most of the mantissa.
        private static int InvertTemperature(double temperatureC)
        {
            var disc = T2 * T2 + 4.0 * T3 * temperatureC;
            var partial = (disc <= 0.0)
                ? temperatureC / T2
                : 2.0 * temperatureC / (T2 + Math.Sqrt(disc));
            return Clamp24(partial + T1);
        }

        // Cubic in rawPress, and strictly increasing over the range this rig
        // uses, so Newton from the linear term converges in a couple of steps.
        // Four is well past convergence and still nothing next to the 20 ms
        // between samples.
        private static int InvertPressure(double pressurePa, double tLin)
        {
            var out1 = P5 + P6 * tLin + P7 * tLin * tLin + P8 * tLin * tLin * tLin;
            var k1 = P1 + P2 * tLin + P3 * tLin * tLin + P4 * tLin * tLin * tLin;
            var k2 = P9 + P10 * tLin;

            var raw = (pressurePa - out1) / k1;
            for(var i = 0; i < NewtonIterations; i++)
            {
                var slope = k1 + 2.0 * raw * k2 + 3.0 * raw * raw * P11;
                raw -= (CompensatePressure(raw, tLin) - pressurePa) / slope;
            }
            return Clamp24(raw);
        }

        private static int Clamp24(double value)
        {
            if(double.IsNaN(value))
            {
                return 0;
            }
            return (int)Math.Max(0.0, Math.Min(MaxCount, Math.Round(value)));
        }

        // Receive thread. A GPIO may only be driven from the time domain, so the
        // edge is marshalled rather than raised here; ExternalWorld is the
        // timestamp Renode uses for events originating outside the emulation.
        private void OnSamplePushed()
        {
            machine.HandleTimeDomainEvent<bool>(RaiseDataReady, true,
                new TimeStamp(default(TimeInterval), EmulationManager.ExternalWorld));
        }

        private void RaiseDataReady(bool unused)
        {
            // Only when the firmware actually enabled drdy on the INT pin - the
            // write Bmp390_HwInit makes to INT_CTRL. A driver that skips it sees
            // no interrupts here, which is the point.
            if((registers[IntCtrlRegister] & IntDrdyEnable) == 0)
            {
                return;
            }

            // Set, not pulse. A second sample landing before the driver reads
            // finds the line already high and produces no new edge, which is
            // what a real part does for a sample that was missed.
            IRQ.Set();
        }

        private long consumedSequence;
        private PushedSample activeSample;
        private byte[] activeCounts;

        private bool expectAddress;
        private bool reading;
        private bool pendingDummy;
        private uint address;
        private int byteIndex;

        private readonly IMachine machine;
        private readonly byte[] registers;
        private readonly SamplePushListener source;

        private const int RegisterCount = 0x80;
        private const byte AddressMask = 0x7F;
        private const byte ReadFlag = 0x80;

        private const int PayloadLength = 2 * 4;
        private const uint DataLength = 6;      // pressure then temperature, 24 bit, LSB first
        private const int MaxCount = (1 << 24) - 1;
        private const int NewtonIterations = 4;

        private const uint ChipIdRegister = 0x00;
        private const uint StatusRegister = 0x03;
        private const uint Data0Register = 0x04;
        private const uint NvmParT1Register = 0x31;
        private const uint IntCtrlRegister = 0x19;
        private const uint CmdRegister = 0x7E;

        // INT_CTRL bit 6 = drdy_en.
        private const byte IntDrdyEnable = 1 << 6;

        private const byte DrdyPress = 1 << 5;
        private const byte ChipId = 0x60;
        private const byte SoftResetCommand = 0xB6;

        // NVM_PAR_T1 (0x31) .. NVM_PAR_P11 (0x45), little endian, widths and
        // signs per datasheet Table 24. Keep this in step with the decoded
        // constants below - they are the same fourteen numbers, and the whole
        // check is that bmp390.c derives the second set from the first.
        private static readonly byte[] CalibrationNvm =
        {
            0x84, 0x6C,             // T1  27780  u16
            0xC6, 0x48,             // T2  18630  u16
            0xF9,                   // T3     -7  i8
            0x27, 0x71,             // P1  28967  i16
            0x19, 0x42,             // P2  16921  i16
            0x16,                   // P3     22  i8
            0xF8,                   // P4     -8  i8
            0xB0, 0x04,             // P5   1200  u16
            0xB8, 0x0B,             // P6   3000  u16
            0xF2,                   // P7    -14  i8
            0x06,                   // P8      6  i8
            0x5A, 0x31,             // P9  12634  i16
            0x1A,                   // P10    26  i8
            0xB6,                   // P11   -74  i8
        };

        // Datasheet 8.4 applied to the block above. bmp390.c writes these
        // scalings as hex-float literals (0x1pN is 2^N); C# has no such literal,
        // and 2^-65 as a decimal is not something a reader can check by eye, so
        // the exponent stays visible as an exponent.
        private static double Pow2(int n)
        {
            return Math.Pow(2.0, n);
        }

        private static readonly double T1 = 27780.0 * Pow2(8);
        private static readonly double T2 = 18630.0 * Pow2(-30);
        private static readonly double T3 = -7.0 * Pow2(-48);
        private static readonly double P1 = (28967.0 - Pow2(14)) * Pow2(-20);
        private static readonly double P2 = (16921.0 - Pow2(14)) * Pow2(-29);
        private static readonly double P3 = 22.0 * Pow2(-32);
        private static readonly double P4 = -8.0 * Pow2(-37);
        private static readonly double P5 = 1200.0 * Pow2(3);
        private static readonly double P6 = 3000.0 * Pow2(-6);
        private static readonly double P7 = -14.0 * Pow2(-8);
        private static readonly double P8 = 6.0 * Pow2(-15);
        private static readonly double P9 = 12634.0 * Pow2(-48);
        private static readonly double P10 = 26.0 * Pow2(-48);
        private static readonly double P11 = -74.0 * Pow2(-65);
    }
}
