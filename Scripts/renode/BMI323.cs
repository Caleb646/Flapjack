//
// A minimal BMI323 for the Flapjack SIL: the SPI framing, a 16-bit register
// file, and the two registers whose *values* bmi323.c actually checks. Enough
// for Imu_Init_ to complete and for Imu_Update to stream samples.
//
// Not modelled: the feature engine, ODR pacing, self-test/self-calibration.
// Reads return whatever was last written, so the driver's config read-back
// compare passes trivially.
//
// The data-ready interrupt IS modelled: INT1 is raised when a pushed sample
// arrives and dropped when the firmware reads the data registers, but only if
// the firmware actually enabled the INT1 output and mapped acc drdy onto it.
// A driver that forgets either write sees no interrupts here, which is the
// point - it must not pass because the model is permissive.
//
// Load with `include @Scripts/renode/BMI323.cs` before the platform
// description that instantiates it, and after SamplePushListener.cs.
//
// With `port:` set the model takes samples pushed by bridge.py over a socket -
// see SamplePushListener.cs for the framing and the threading.
//
// Payload, 24 bytes, little endian:
//
//     0-11  accel x,y,z  float32  m/s^2, specific force as the part measures
//                                 it (a - g), body FRD
//     12-23 gyro  x,y,z  float32  deg/s, body FRD
//
// The same die-frame physical values the retired SensorData frame used to
// carry, so the bridge has nothing new to compute - the difference is that
// these go through the real register path in bmi323.c instead of past it.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class BMI323 : ISPIPeripheral, IGPIOReceiver, IDisposable
    {
        // port 0 leaves the socket off: the register file is then whatever the
        // firmware last wrote, which is all a boot smoke test needs.
        public BMI323(IMachine machine, int port = 0)
        {
            this.machine = machine;
            IRQ = new GPIO();
            registers = new ushort[RegisterCount];
            Reset();
            source = new SamplePushListener(this, port, PayloadLength, OnSamplePushed);
        }

        // INT1. Connect it to the pin the board wires it to - PC5 on flapjack_v1
        // (IMU_INT_GPIO_* in target/flapjack_v1/flapjack_v1.h).
        public GPIO IRQ { get; private set; }

        public void Dispose()
        {
            source.Dispose();
        }

        public void Reset()
        {
            ResetRegisters();
            IRQ.Unset();
            selected = false;
            expectAddress = true;
            Transactions = 0;
            Bytes = 0;
        }

        // Chip select, active low. drivers/bus/spi.c drives it from a GPIO
        // (SPI_NSS_SOFT) and puts several TSIZE transfers inside one assertion -
        // address, then data - so the controller calls FinishTransmission in the
        // middle of a register access. Only the CS line delimits a transaction.
        public void OnGPIO(int number, bool value)
        {
            if(number != 0)
            {
                this.Log(LogLevel.Warning, "Unexpected GPIO {0}; only 0 (chip select) is connected", number);
                return;
            }

            selected = !value;
            if(selected)
            {
                // One snapshot per transaction, so a burst read can never mix
                // accel from one sample with gyro from the next. Scaling happens
                // here rather than on the receive thread because it reads the
                // range registers, which only the emulation thread may write.
                activeSample = source.Latest;
                activeWords = (activeSample == null) ? null : Scale(activeSample.Payload);
                expectAddress = true;
                Transactions++;
            }
        }

        public byte Transmit(byte data)
        {
            if(!selected)
            {
                this.Log(LogLevel.Warning, "Byte 0x{0:X2} clocked in with chip select deasserted", data);
                return 0;
            }

            Bytes++;

            if(expectAddress)
            {
                address = (uint)(data & AddressMask);
                reading = (data & ReadFlag) != 0;
                pendingDummy = reading;
                expectAddress = false;
                byteIndex = 0;
                return 0;
            }

            if(reading)
            {
                // A BMI323 SPI read emits one dummy byte ahead of the register
                // contents; bmi323.c allows for it with nBusDummyBytes = 1.
                if(pendingDummy)
                {
                    pendingDummy = false;
                    return 0;
                }

                var value = Read(address + (uint)(byteIndex / 2));
                var result = (byte)((byteIndex % 2 == 0) ? value : (value >> 8));
                byteIndex++;
                return result;
            }

            // Registers are 16 bit, low byte first on the wire.
            if(byteIndex % 2 == 0)
            {
                pendingLowByte = data;
            }
            else
            {
                Write(address + (uint)(byteIndex / 2), (ushort)((data << 8) | pendingLowByte));
            }
            byteIndex++;
            return 0;
        }

        // Deliberately empty - see OnGPIO.
        public void FinishTransmission()
        {
        }

        public int RawAccelX
        {
            get { return (short)registers[AccDataXRegister]; }
            set { registers[AccDataXRegister] = (ushort)value; }
        }

        public int RawAccelY
        {
            get { return (short)registers[AccDataXRegister + 1]; }
            set { registers[AccDataXRegister + 1] = (ushort)value; }
        }

        public int RawAccelZ
        {
            get { return (short)registers[AccDataXRegister + 2]; }
            set { registers[AccDataXRegister + 2] = (ushort)value; }
        }

        public int RawGyroX
        {
            get { return (short)registers[GyroDataXRegister]; }
            set { registers[GyroDataXRegister] = (ushort)value; }
        }

        public int RawGyroY
        {
            get { return (short)registers[GyroDataXRegister + 1]; }
            set { registers[GyroDataXRegister + 1] = (ushort)value; }
        }

        public int RawGyroZ
        {
            get { return (short)registers[GyroDataXRegister + 2]; }
            set { registers[GyroDataXRegister + 2] = (ushort)value; }
        }

        // Chip-select assertions and bytes clocked, for costing the SPI path.
        public long Transactions { get; private set; }
        public long Bytes { get; private set; }

        private ushort Read(uint register)
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
                // data rather than serving zeros: a -D sim build now depends on
                // this path, and a zero gravity vector would reach the attitude
                // filter as NaN instead of as "the bridge is not connected".
                if(source.Active && register == StatusRegister)
                {
                    return 0;
                }

                // With no socket at all the register file stands on its own, so
                // a bare boot smoke test behaves exactly as it did before.
                return registers[register];
            }

            if(register == StatusRegister)
            {
                return (ushort)(sample.Sequence != consumedSequence ? DataReadyBits : 0);
            }

            if(register >= AccDataXRegister && register < AccDataXRegister + SampleWords)
            {
                // bmi323.c reads STATUS once and then whichever blocks it named,
                // so consuming on the first data register of the pass is enough.
                // Same moment the interrupt is answered: this runs on the
                // emulation thread, so the pin can be touched directly.
                consumedSequence = sample.Sequence;
                IRQ.Unset();
                return (ushort)activeWords[register - AccDataXRegister];
            }

            return registers[register];
        }

        private void Write(uint register, ushort value)
        {
            if(register >= RegisterCount)
            {
                this.Log(LogLevel.Warning, "Write of 0x{0:X4} to out-of-range register 0x{1:X2}", value, register);
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
            // error_status = 0x5 (no error), sc_st_complete = 1. bmi323.c polls
            // bit 0 of this register to decide the feature engine came up after
            // a soft reset, and reads the same register to judge the
            // self-calibration result. A static value cannot satisfy both that
            // and CALIB_IS_ONGOING, so boot logs one benign
            // "IMU has not started the self calibration" - the driver treats it
            // as non-fatal.
            registers[FeatureIo1Register] = 0x0015;
            // drdyAccel | drdyGyro | drdyTemp (bits 7:5). IMUUpdatefromPolling
            // spins on these up to 1000 times per Imu_Update and errors out if
            // they never set, so a register file that reads 0 here costs ~1000
            // SPI transactions per update. Always-ready is the honest answer
            // while nothing feeds samples; once the bridge pushes, this becomes
            // "a new sample arrived since the last data read".
            registers[StatusRegister] = DataReadyBits;
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
            if(!DataReadyRoutedToInt1)
            {
                return;
            }

            // Set, not pulse. A second sample landing before the firmware reads
            // finds the line already high and produces no new edge, which is
            // exactly what a real latched drdy does when a sample is missed.
            IRQ.Set();
        }

        // True only when the firmware both enabled the INT1 output driver and
        // mapped acc drdy onto INT1 - the two writes IMUEnableInterrupts and
        // IMUSetupInterrupts make in bmi323.c.
        private bool DataReadyRoutedToInt1
        {
            get
            {
                var outputEnabled = (registers[IoIntCtrlRegister] & Int1OutputEnable) != 0;
                var mapping = (registers[IntMap1Register + 1] & AccDrdyMask) >> AccDrdyShift;
                return outputEnabled && mapping == Int1;
            }
        }

        // SI in, LSB counts out, using the ranges the firmware actually
        // programmed - so a range change in bmi323.c really does change what
        // the part reports, instead of being a write nobody reads back.
        private short[] Scale(byte[] payload)
        {
            var accelFullScale = AccelRangesG[RangeIndex(AccConfRegister, AccelRangesG.Length)] * StandardGravity;
            var gyroFullScale = GyroRangesDps[RangeIndex(GyrConfRegister, GyroRangesDps.Length)];

            var sample = new short[SampleWords];
            for(var i = 0; i < 3; i++)
            {
                sample[i] = ToCounts(BitConverter.ToSingle(payload, i * 4), accelFullScale);
                sample[i + 3] = ToCounts(BitConverter.ToSingle(payload, (i + 3) * 4), gyroFullScale);
            }
            return sample;
        }

        private int RangeIndex(uint register, int count)
        {
            var index = (registers[register] & RangeMask) >> RangeShift;
            return Math.Min(index, count - 1);
        }

        private static short ToCounts(float value, float fullScale)
        {
            var counts = (int)Math.Round(value * 32768f / fullScale);
            return (short)Math.Max(short.MinValue, Math.Min(short.MaxValue, counts));
        }

        private long consumedSequence;
        private PushedSample activeSample;
        private short[] activeWords;

        private bool selected;
        private bool expectAddress;
        private bool reading;
        private bool pendingDummy;
        private uint address;
        private int byteIndex;
        private byte pendingLowByte;

        private readonly IMachine machine;
        private readonly ushort[] registers;
        private readonly SamplePushListener source;

        private const int RegisterCount = 0x80;
        private const byte AddressMask = 0x7F;
        private const byte ReadFlag = 0x80;

        private static readonly float[] AccelRangesG = { 2f, 4f, 8f, 16f };
        private static readonly float[] GyroRangesDps = { 125f, 250f, 500f, 1000f, 2000f };

        // STATUS bits 7:5 - drdyAccel | drdyGyro | drdyTemp.
        private const ushort DataReadyBits = 0x00E0;

        private const int SampleWords = 6;
        private const int PayloadLength = SampleWords * 4;
        private const float StandardGravity = 9.81f;

        // ACC_CONF and GYR_CONF both carry the range in bits 6:4.
        private const int RangeMask = 0x0070;
        private const int RangeShift = 4;

        private const uint ChipIdRegister = 0x00;
        private const uint StatusRegister = 0x02;
        private const uint AccDataXRegister = 0x03;
        private const uint GyroDataXRegister = 0x06;
        private const uint FeatureIo1Register = 0x11;
        private const uint AccConfRegister = 0x20;
        private const uint GyrConfRegister = 0x21;
        private const uint IoIntCtrlRegister = 0x38;
        private const uint IntMap1Register = 0x3A;
        private const uint CmdRegister = 0x7E;

        // IO_INT_CTRL bit 2 = int1_output_en. INT_MAP1+1 (0x3B) carries
        // acc_drdy_int in bits 11:10, where 1 selects INT1.
        private const ushort Int1OutputEnable = 1 << 2;
        private const ushort AccDrdyMask = 0x0C00;
        private const int AccDrdyShift = 10;
        private const int Int1 = 1;

        private const ushort ChipId = 0x0043;
        private const ushort SoftResetCommand = 0xDEAF;
    }
}
