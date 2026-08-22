# Flapjack — The SIL: Renode CM7 + JSBSim

**Status: built, running and used daily.** This is the reference for how the software-in-the-loop
rig works, what it measures, and where it lies to you. It replaces `EmulatorResearch.md`,
`RcSilResearch.md` and `SensorSilResearch.md`, which were the build-time research for the three
pieces (emulator, RC path, sensor path) — predictions that were confirmed became statements,
predictions that turned out wrong were dropped, and the plans in all three were executed.

Defects the SIL found live in [KnownIssues.md](KnownIssues.md). Control-law analysis lives in
[ControlResearch.md](ControlResearch.md). Command syntax lives in the **flapjack-tooling** skill.

**Scope:** single-core CM7 only. CM4 is not emulated.
**Verified against:** Renode v1.16.1 (`Tools/renode/`), JSBSim 1.3.1, the upstream
`renode-infrastructure` peripheral models.

```bash
python Scripts/board.py build -b flapjack-v1 -D sim --single-core
python Scripts/board.py renode -b flapjack-v1                          # terminal 1
python Scripts/board.py sim --port socket://localhost:4000 \
    --rc-port socket://localhost:4001 --gps-port socket://localhost:4002 \
    --imu-port socket://localhost:4010 --rate 400
```

---

## 1. Architecture

### 1.1 Three wires, deliberately

```
 PC: JSBSim + bridge.py  <-ServoCmd/MotorCmd---  USART1  sim link (+ logs, + shell)
                         <-Telemetry   50 Hz---
                         --CRSF 0x16   50 Hz-->  USART3  RX UART
                         --NMEA GGA/RMC 10 Hz->  USART2  GPS UART
                         --accel/gyro 400 Hz-->  tcp/4010 -> emulated BMI323  -> SPI1
                         --field      400 Hz-->  tcp/4011 -> emulated MMC5983 -\
                                                                                >- SPI5 (mux)
                         --Pa, degC    50 Hz-->  tcp/4012 -> emulated BMP390  -/
```

**Six wires, and the sim link is now FC→PC only** (apart from the shell). No sensor value reaches
the firmware pre-decoded any more: all three parts are emulated and read through their real
drivers over emulated SPI. `SensorData` (id 1) and `BaroData` (id 7) are **retired**, along with
the sim `mag`/`baro` driver backends and `SimLink_WaitMag`/`WaitBaro`.

**Mag and baro share SPI5 on separate chip selects** (PF4, PF6), which `STM32H7_SPI` cannot
express — it is a `NullRegistrationPointPeripheralContainer` and holds exactly one child. They
hang off an `SPI.SPIMultiplexer` instead, which is also what gives those two models correct
transaction framing for free; see §2.1.

RC and GPS ride **their own wires in their real wire formats**, so a SIL run exercises the UART
ISR, the deframer, the checksum and the parser. This is not incidental — it is the highest-yield
decision in the rig. There used to be an `RcInput` sim-link frame that wrote `g_Rx.channels`
directly, and it is precisely why five CRSF spec violations survived undetected (KnownIssues
§2.21, §2.23). GPS was never given a sim-link frame for the same reason, and the first thing the
real path caught was a parser that returned success while discarding every field.

**Frame id 2 is retired and must not be reused** — an older bridge would still be sending it.

The same argument took the IMU off the sim link entirely. A register read is not a byte protocol,
but it is still a path with a chip-select, a dummy byte, an address auto-increment and a
data-ready bit to get wrong — and stepping over all of it hid §2.29 through §2.33, five firmware
defects that a driver-level injection point cannot reach by construction. Emulating the part costs
about 1,200 SPI transactions a second and roughly 18 us of host time each.

**Baro is no longer the exception**, and it turned out to be the most valuable of the three to
emulate. `bmp390.c` reads a 21-byte calibration NVM block — fourteen little-endian words of
assorted widths and signs, each scaled by its own power of two — and runs an eleven-coefficient
cubic over it. A driver-level injection point steps over all of that. The model therefore
publishes a realistic coefficient set and **inverts** datasheet 8.5/8.6 to find the raw ADC counts
the driver will compensate back into the pressure the bridge asked for, so a mis-decoded
coefficient shows up as a pressure out by kilopascals instead of passing silently. Measured
round-trip error over −20…60 °C and 50–115 kPa is **0.016 Pa**, which is the driver's own float32
rounding rather than quantisation.

Rate still matters and is still preserved: the bridge pushes baro at ~50 Hz, and the model only
raises `drdy_press` for a sample the driver has not already read, so `Baro_Task` (which polls at
100 Hz) is paced by the part exactly as it is on hardware.

### 1.2 Sim link framing and id budget

`[0xAA][0x55][msg_id][len][payload][crc8]`, crc8 poly 0x07 init 0x00 over `(msg_id,len,payload)`.
Defined identically in `Firmware/drivers/sim_link/sim_link.c` and `Scripts/link/framing.py`.

| ID | Message | Dir |
|---|---|---|
| 1 | **retired** (was `SensorData`) — never reuse | — |
| 2 | **retired** (was `RcInput`) — never reuse | — |
| 3 | `ServoCmd` (tilt, rad) | FC→PC |
| 4 | `MotorCmd` (throttle 0–1) | FC→PC |
| 5 | `Telemetry` | FC→PC |
| 6 | `SERIAL_MSG_SHELL_CMD` | PC→FC |
| 7 | **retired** (was `BaroData`) — never reuse | — |

Retiring an id does **not** free it: `SerialLink_RegisterHandler` still rejects `msgId >=
SERIAL_LINK_MAX_MSG_ID` (8), and an older bridge on the other end of the wire would still be
sending 1, 2 and 7. A new frame has to raise the ceiling and grow the handler table.

`SERIAL_LINK_MAX_MSG_ID` was bumped from 8 to 16 when `BaroData` landed; the handler table is one
pointer per id, so headroom is cheap.

**Why baro got a frame and GPS did not** is a bandwidth argument as well as a coverage one.
`SensorData` serialises to 42 B (47 B framed) = 40.8 % of the 460800-baud link at 400 Hz. Folding
baro *and* GPS into it would have taken that to 81.6 % and put the payload within a byte or two of
`SERIAL_LINK_MAX_PAYLOAD` (96). Separate frames at natural rates cost **+2.5 %**.

### 1.3 What paces what

**Pacing is now the parts' job, not the link's.** Every sensor task polls on a `vTaskDelay` and
its driver reports `eSTATUS_BUSY` when the part has no new sample — which is exactly what the
tasks do on hardware. Each model therefore drives its data-ready bit as *"a sample has arrived
that the driver has not read yet"* (`STATUS` drdy bits on the BMI323, `MEAS_M_DONE` on the
MMC5983, `drdy_press` on the BMP390) and clears it when the data registers are read. Get that
wrong — report always-ready — and `Baro_Task` publishes at its 100 Hz poll rate against a 50 Hz
push, doubling every nav correction and breaking the `baro_count` pacing check.

The semaphore-per-consumer arrangement this section used to describe is gone with the sim
backends. The hazard it existed for is worth remembering, because the same shape can recur:
**every sensor task must genuinely block or yield**. Two did not, and both starved everything
below them (KnownIssues §2.4, §2.6, §2.20).

`Telemetry.imu_count`, the pacing metric for the whole chain, used to be `SimLink_GetSensorCount()`
— a count of *frames the PC sent*, which measured the bridge rather than the firmware. `Imu_Task`
now publishes a `sensors_imu_status` topic carrying a running total of samples it actually read
and published, and `sim_telemetry.c` subscribes to it. A running total rather than a tick, so a
subscriber reading slower than the publisher still recovers the true count and the metric does not
depend on anyone's queue depth.

---

## 2. The Renode platform overlay

Renode's shipped `stm32h747.repl` is good — **the memory map matches `cm7_flash.ld` exactly**
(flash 0x08000000, DTCM 128K, AXI SRAM 512K, SRAM1-3 contiguous 288K, SRAM4 64K, ITCM 64K), and
USART1-6, TIM1-8/12-17, NVIC, EXTI, HSEM, GPIO A-K and DMA are all real models. No memory work was
needed, and none of the three UARTs the SIL uses needed a model written.

What it needed is [Scripts/renode/flapjack_h7_cm7.repl](Scripts/renode/flapjack_h7_cm7.repl), and
this is why each entry is there:

| Entry | Why |
|---|---|
| `nvic systickFrequency: 64000000` | Firmware runs SYSCLK from **HSI at 64 MHz**, not the PLL (`platform.c`, `RCC_SYSCLKSOURCE_HSI`, all dividers 1). Upstream hardcodes 400 MHz → FreeRTOS tick 6.25× fast |
| `timer16 frequency: 64000000` | HAL timebase. Upstream 250 MHz → `HAL_GetTick()` 3.9× fast |
| `usart1/2/3 frequency: 64000000` | PCLK2. Affects modeled baud |
| `dwt: Miscellaneous.DWT @ 0xE0001000` | `GetMicroseconds()` reads `DWT->CYCCNT`. The model exists upstream, it is simply not wired into the H7 platform. Without it the clock degrades to 1 ms granularity |
| `spi1/3/5: SPI.STM32H7_SPI` | Upstream leaves these as `Tag`s; `Spi_InitSystem()` runs unconditionally in `main()` |
| `bmi323: Sensors.BMI323 @ spi1` | The emulated IMU, compiled at load time from `Scripts/renode/BMI323.cs`. `port: 4010` is where the bridge pushes samples |
| `gpioPortC: 4 -> syscfg#2@4 \| bmi323@0` | Chip select. The driver drives PC4 by hand (`SPI_NSS_SOFT`), and the fan-out keeps the EXTI path intact |
| `spi5Mux: SPI.SPIMultiplexer @ spi5` | SPI5 carries two parts on separate chip selects and `STM32H7_SPI` holds only one child. `init: SetActiveLow 0/1` because both selects are active low |
| `mmc5983: Sensors.MMC5983 @ spi5Mux 0x0`, `bmp390: Sensors.BMP390 @ spi5Mux 0x1` | The emulated mag and baro, ports 4011 and 4012. The registration address **is** the mux GPIO index |
| `gpioPortF: 4 -> syscfg#5@4 \| spi5Mux@0`, `6 -> syscfg#5@6 \| spi5Mux@1` | The two chip selects. Overriding pin 6 deliberately drops upstream's `6 -> timer16@00` AF mapping: PF6 is a plain GPIO CS on this board, and timer16 is the HAL timebase |

### 2.1 Why the mux also fixes transaction framing

`BMI323.cs` has to take its chip select as a GPIO and delimit transactions itself, because
`drivers/bus/spi.c` puts the register address and its data in two separate `TSIZE` transfers
inside one CS assertion, and `STM32H7_SPI.EndTransfer` raises `FinishTransmission` at the end of
*each* — i.e. in the middle of a register access.

`SPIMultiplexer` is constructed with `suppressExplicitFinishTransmission = true`, so it **swallows**
those and forwards exactly one `FinishTransmission` to the child when the chip select is released.
So `MMC5983.cs` and `BMP390.cs` are plain `ISPIPeripheral`s that reset their framing state in
`FinishTransmission`, and take no chip-select GPIO of their own.

Two behaviours of that model to know about:

- `UpdateChipSelectState` fires `FinishTransmissionByAddress` on **every** currently-inactive line
  each time any line moves, so a child sees spurious `FinishTransmission` calls. Both models are
  idempotent there, which is why that is harmless.
- Both chip selects read **low out of reset** (GPIO outputs idle low; the firmware only drives them
  high in `SpiDev_Init`), so before driver init the mux considers *both* parts selected and refuses
  to transmit with a "multiple devices are currently selected" warning. That is faithful to the
  board, and it clears as soon as the drivers initialise.

### 2.2 The BMP390's inverse compensation

The published NVM block and the `double` coefficients in `BMP390.cs` are the same fourteen numbers,
and the check the SIL gets for free is that `BaroReadCalibration` derives the second set from the
first. Temperature inverts in closed form; pressure is a cubic solved by four Newton steps from
the linear term. The temperature root is written as `2c/(b + sqrt(b² − 4ac))` rather than the
schoolbook form because `T3` is ~1e-14 against a `T2` of ~1e-5 — the usual expression subtracts two
nearly equal numbers and throws away most of the mantissa.

Pressure is inverted against the tLin the driver will derive from the **quantised** raw
temperature, not against the temperature the bridge asked for; otherwise the two sides disagree by
the temperature LSB and the pressure carries that error.

Sensor **noise** is deliberately not modelled here — `Scripts/sim/jsbsim/systems/` already gives
the baro 2 Pa of noise and 20 Pa of bias (§5.2). The model contributes quantisation only, so the
two error sources stay separable.

**The frequency pins are the load-bearing part.** Renode's `STM32H7_RCC` does not implement
`D1CFGR`/`D2CFGR`/`D3CFGR`, so `HAL_RCC_GetPCLK2Freq()` reads 0, divides by 1, and computes
64 MHz — while the emulated peripherals tick at whatever the `.repl` says. **The `.repl` is the
single source of truth for peripheral rates.** Skip this and virtual time and firmware-perceived
time diverge by 4–6×, and your control loop is not running at the rate it claims.

Verified by reading the counters out of memory after exactly 1.0 virtual second: `uwTick` = **1000**
(expected 1000) and `xTickCount` = 983 (the scheduler starts ~17 ms after reset). With the upstream
values these read ~6250 and ~3900.

Expected benign log noise: `rcc: Unhandled read from offset 0x18/0x1C/0x54/0xD8/0xE8` (the CFGR
registers above), `usart: Unhandled write to offset 0x2C` (`PRESC`), and
`ReadDoubleWord from non existing peripheral at 0x5C001000` (DBGMCU_IDCODE).

### 2.1 Firmware guards the emulator required

All `#if`-gated on `SINGLE_CORE`; the dual-core hardware build is unchanged.

- `platform.c` — the CM4 wake block spins on `RCC_FLAG_D2CKRDY`, which Renode's RCC **tags**
  without implementing, so the bit never sets and boot dies in `CriticalErrorHandler()`.
- `main.c` — `while (!s_IsCM4Ready) { }` hangs forever with no CM4.
- `cfg.h` — `CFG_PRIMARY_LOGGER` was `CM4_CPUID`, so CM7 wrote to a ring buffer and signalled a
  core that does not exist. A single-core build was **completely silent** until this was fixed
  (KnownIssues §2.7, §2.19).

`SanityCheckTimers()` in `Core_Init()` is the first useful "is the emulation sane" gate — it fails
boot if `HAL_Delay(10)` and `DelayMicroseconds(10000)` disagree with the clock. It passes, which is
what confirms the timebase.

### 2.2 Setup traps, for whoever does this next

| Trap | Resolution |
|---|---|
| `serial.Serial` cannot take `socket://` URLs | `serial_for_url` in `link/serial_io.py` — handles `COM7` and sockets alike, so hardware HIL is unaffected |
| Renode `--console` reads EOF and quits when stdin is not a TTY | `board.py renode` uses `-P <port>` |
| `Tools/` is gitignored, so the `.repl`/`.resc` could not live beside Renode | Moved to `Scripts/renode/` |
| `pip install -e` dropped `*.egg-info` in the repo root | `setup.cfg` with `egg_base = Build` |
| `cpu VectorTableOffset 0x08000000` must precede `LoadELF` | Set in `flapjack_sil.resc` |
| `<system file="SensorBaro"/>` fails to resolve | JSBSim resolves `<system file>` against the **FDM root**, not the installed package. The files are vendored into `Scripts/sim/jsbsim/systems/` |
| Single- and dual-core images shared a build directory | `--single-core` goes to `Build/<board>/<config>-single-core/`; `renode` looks only there, `flash` only in the dual-core tree |

The portable Renode at `Tools/renode/` embeds its own .NET 8 runtime and ships the compiled
peripheral models. The `Research/renode` checkout is source-only and incomplete
(`src/Infrastructure` is an uninitialised submodule) — keep it as reference for `.repl` syntax and
model names; it is not the thing that runs.

---

## 3. Measured performance

Sensor frames streamed at a fixed wall-clock rate with exact accounting against
`Telemetry.imu_count`. `SimTelemetry_Task` runs at a fixed 50 Hz in *virtual* time, so observed
telemetry frames/s ÷ 50 gives the virtual:wall ratio for free.

| Sensor rate | Drop | Consumed/s | virt:wall |
|---|---|---|---|
| 200 Hz | 0.0 % | 200.0 | 1.00× |
| **400 Hz** | **0.2 %** | **399.2** | **1.00×** |
| 800 Hz | 7.4 % | 741.1 | 1.00× |
| 1200 Hz | 42.1 % | 694.6 | 0.99× |

**400 Hz sustains cleanly** — the bottom of the 400–800 Hz IMU ODR range
[Firmware/CLAUDE.md](Firmware/CLAUDE.md) specifies. Throughput peaks near **740/s** and then
*falls* under further load (694/s at 1200 Hz): congestion collapse, the classic sign of having
passed saturation.

**Renode is not the bottleneck.** It tracks wall clock **1.00×** under load, which is the opposite
of the pre-build expectation — the design worry was that an interpreted Cortex-M7 model would lag
and overflow the 512-byte stream buffer. It does not. An earlier 2.6× figure was an *idle*
measurement: Renode fast-forwards through `WFI`, so it out-runs wall clock when the firmware is
doing nothing and settles to exactly 1.00× once there is work.

Two consequences:

- **`Telemetry.imu_count` vs frames sent is the health metric.** It should track 1:1. It is also
  how the one real throughput limit was found — a `vTaskDelay(pdMS_TO_TICKS(5))` in `Imu_Task`
  pinned consumption at exactly 200/s and the drop rate tracked `1 − 200/rate` precisely. Removed;
  the task blocks on its semaphore as the architecture always intended.
- **Lock-step bridging is not needed for correctness.** Wall-clock pacing is already right. It
  would still buy *reproducibility*, which is what a golden-trajectory gate needs — see §7.

**Restart Renode between runs** rather than re-attaching the bridge. A gap in the sensor stream
hands `Nav_Update` one enormous `dt`, which throws the attitude estimate and costs seconds of
recovery. This matters more with an altitude estimator integrating accel, not less.

---

## 4. Fidelity — what the SIL does not reproduce

Read this section before calling anything "tested".

**Renode's UART models are not baud-paced.** A socket write delivers all 26 bytes as back-to-back
RXNE interrupts in essentially zero virtual time, regardless of the configured baud.

- The CRSF deframer is length-driven, so it works fine — but **write one whole frame per socket
  write** and never interleave anything else on that port.
- The `usFrameTimeout` resync branch in `crsf.c` is **dead code in the SIL**. Real inter-byte
  timing, partial frames and line noise are not covered. It is unreachable in the host unit tests
  too, because `GetMicroseconds()` returns 0 under `UNIT_TEST` — covering it needs a time seam
  that does not exist (KnownIssues §4.14).
- `s_CrsfFrame` is a single buffer written by the ISR and read by `Crsf_ProcessFrame` with no
  double-buffering. On hardware a frame arriving mid-poll can tear; under Renode the whole frame
  lands in one instantaneous burst, so the SIL **underexposes** this relative to hardware.

**There is no IMU noise model.** `Scripts/sim/jsbsim/systems/` models baro and GPS error, but the
gyro and accel the bridge synthesises are exact. This is why the D-term filter cutoff had to be
chosen on a host replica with injected noise rather than in the SIL (ControlResearch §2.4). Adding
a gyro noise model is the obvious next move — it would bring that whole question inside the rig.

**The FDM is unvalidated.** Estimated inertias, an **estimated 60 mm rotor height** that pitch
authority scales linearly with, `ROTOR_TMAX_LBS` from a guess rather than a thrust stand, and
representative-not-measured servo rate and lag figures. Any gate comparing firmware to FDM
trajectory is measuring *this firmware against an unvalidated plant*: a good **regression** signal,
not an absolute accuracy claim. Say so wherever the numbers get written down.

**The baro test is the exception**, and it is worth being precise about why. §5.2's oracle is the
ISA — a published standard, not the tiltrotor model — so a pressure→altitude gate is an
**absolute correctness** claim that stays valid no matter how wrong the airframe's inertias are.
Estimator-*fusion* accuracy is back in regression-only territory.

**Attitude is not repeatable run to run.** `eMISSION_MODE_MANUAL` is a rate loop, so nothing pulls
the vehicle back to level and the attitude a run settles at is whatever the takeoff transient left
(KnownIssues §1.13). Gate on NaN count, `imu#` rate and CRC failures; treat attitude as a
distribution needing several runs.

---

## 5. JSBSim reference

Measured on JSBSim 1.3.1 driving the repo's `tiltrotor` model at 37.6189 °N / 122.3750 °W.

### 5.1 Properties

| Need | Property | Units |
|---|---|---|
| Static pressure | `atmosphere/P-psf` | lbf/ft² (× 47.88026 → Pa) |
| Air temperature | `atmosphere/T-R` | Rankine (−491.67, ×5/9 → °C) |
| **Geodetic** latitude | `position/lat-geod-deg` / `-rad` | deg / rad |
| Longitude | `position/long-gc-deg` / `-rad` | deg / rad |
| MSL / AGL altitude | `position/h-sl-meters`, `h-agl-ft` | m / ft |
| NED velocity | `velocities/v-{north,east,down}-fps` | fps (× 0.3048 → m/s) |
| Climb rate | `velocities/h-dot-fps` | fps |

**One trap worth writing down.** `position/lat-gc-deg` is **geocentric**; a GPS reports
**geodetic**. Setting `ic/lat-gc-deg = 37.6189` yields `position/lat-geod-deg = 37.8051` — a
0.19° error, about **20 km**. Use `ic/lat-geod-deg` to set and `position/lat-geod-deg` to read.
JSBSim's own `SensorGps.xml` gets this right, which is a good reason to use it rather than
hand-rolling the property list.

### 5.2 Baro is exact enough to be a test oracle

JSBSim's reported pressure and the inverse barometric formula agree to **within 5 cm from 0 to
1000 m**:

| True h | `sensor/baro/presStatic_Pa` | inverse formula | error |
|---:|---:|---:|---:|
| 0 m | 101325.55 | −0.046 m | −4.6 cm |
| 50 m | 100726.33 | 49.962 m | −3.8 cm |
| 100 m | 100129.97 | 99.970 m | −3.0 cm |
| 200 m | 98945.93 | 199.979 m | −2.1 cm |
| 500 m | 95461.78 | 499.992 m | −0.8 cm |
| 1000 m | 89876.73 | 999.950 m | −5.0 cm |

**This is the most useful measurement in the document.** It gives the pressure→altitude conversion
an analytically exact ground truth, independent of the FDM's validity, and a 5 cm noise floor lets
the tolerance be tight without flakiness. These pressures are the fixtures in
[Tests/UnitTest/test_altitude.c](Tests/UnitTest/test_altitude.c).

### 5.3 The accelerometer sign

JSBSim's accelerometer and the FC's convention agree, and it is worth recording why, because they
briefly did not. Level and still on the gear:

```
sensor/imu/accelZ_mps2 = -9.800
```

The FC's contract in `sim.proto` is specific force as a real part measures it: level and still is
**(0, 0, −9.81)** in body FRD, because a part reads +1 g along whichever axis points up and FRD Z
points down. `synthesize_sensors()` now matches that sign, having previously produced +9.789.

X and Y were both ~0 in the condition that was measured, so **they are not discriminated by this
test**. If the bridge is ever switched to read JSBSim's accelerometer instead of computing its own,
verify the X/Y mapping with a static tilt sweep first — KnownIssues §2.2 records attitude having
been inverted on all three axes, and this is exactly the switch that could reintroduce it.

### 5.4 Do **not** adopt JSBSim's magnetometer

Same run, normalised body-frame field:

```
bridge mag(n)  +0.643  +0.198  +0.740      (synthetic 60 deg inclination, 0 declination)
jsbsim mag(n)  +0.762  -0.231  -0.605      |B| = 32.2 uT
```

Two problems. The **Z sign is opposite** — at 37 °N a real field has a strong *downward* (+Z in
FRD) component, and JSBSim reports it negative. And **|B| = 32.2 µT** against ~48 µT real at that
location, so it is not a faithful WMM field either.

`SensorBaro.xml` and `SensorGps.xml` are drop-in and are vendored. `SensorImu.xml` is deliberately
**not** used. Leave `synthesize_sensors()`'s dipole in place for mag until someone validates
JSBSim's model against WMM — the existing dipole is at least *self-consistent* with the attitude
the filter is being asked to reconstruct, which is what the attitude comparison depends on.

### 5.5 Sensor error models

`SensorBaro.xml` / `SensorGps.xml` publish **both** a `sensor/<x>/<field>_true_*` and a post-error
`sensor/<x>/<field>` property, so the bridge can feed the corrupted measurement while the gate
asserts against exact truth — which no recorded dataset can offer. They ship from JSBSim with every
error term at zero; the terms now enabled, and the reasoning behind each value, are in KnownIssues
§3.13. Turn error terms on **there**, in the vendored XML, not in `bridge.py`.

---

## 6. The flight model

`tiltrotor.xml` was rebuilt during bring-up and the history is worth keeping, because most of the
faults are traps anyone editing it can walk back into.

**It had never parsed.** Line 14 sat inside an XML comment containing `--port` / `--dry-run`, and
**`--` is illegal inside an XML comment**. So the closed loop had never run at all. This trap bites
easily — CLI flags and `----` rules inside XML comments.

**`FGPropeller` was replaced with `<external_reactions>`.** The propeller diverged for reasons that
scaling could not fix: the advance ratio ran to 1e8 within 0.05 s regardless of thrust direction,
with RPM exceeding `maxrpm`. A SIL does not need a blade-element propeller — it needs thrust as a
function of throttle applied along the tilt axis. Four body-axis forces scaled by
`cos(tilt)`/`sin(tilt)`: no advance ratio, no rpm ODE, and JSBSim clamps table lookups at their
endpoints. Verified standalone:

| throttle | tilt | Fx (lbf) | Fz (lbf) | T/W |
|---|---|---|---|---|
| 0.25 | 0° | 0 | −0.935 | 0.50 |
| **0.50** | 0° | 0 | **−1.874** | **1.00** |
| 1.00 | 0° | 0 | −3.733 | 1.99 |
| 0.50 | 90° | +1.863 | 0 | 0.99 |
| 0.50 | 45° | +1.321 | −1.321 | 1.00 |

The trade: genuine propeller behaviour in forward flight is gone — thrust falloff with airspeed,
windmilling. Forward flight is approximated by a thrust-vs-airspeed table instead. Fine for hover
and low-speed control work; revisit before trusting cruise or transition results.
`engine/drone_motor.xml` and `engine/drone_prop.xml` are consequently **orphaned** — kept, not
deleted, in case the propeller route is ever revisited.

**`fcs/throttle-cmd-norm[N]` and `fcs/tilt-cmd-rad[N]` must be declared in the FCS**, since JSBSim
normally creates the throttle properties per engine and this model now has no `<propulsion>`
section.

**`propulsion/engine[N]/pitch-angle-rad` is absolute and overrides the thruster's `<orient>`**,
which is why `<orient>pitch=90</orient>` had no effect once `bridge.py` started writing the
property. Measured: `pitch-angle-rad` 0 → thrust forward, +π/2 → thrust up, −π/2 → thrust down.
The FC's neutral servo angle means "rotors up", so the offset lives in the bridge as
`TILT_HOVER_OFFSET`.

**`<location>` inside `<contact>` is the structural frame** (+X aft, +Z up) — the opposite sense in
X *and* Z to the body frame used everywhere else. A contact named `TAIL` at `x = -0.25` was
actually at the nose, which is half of why any ground contact diverged (KnownIssues §3.4). Gear
spring/damping are given in JSBSim's native `LBS/FT`, not `N/M`.

**Rotor height above the CG is an estimate.** Structural `z = +0.06` (60 mm). Pitch response
scales linearly with it and the repo has no airframe CAD, only PCB designs. Flagged inline in
`tiltrotor.xml` alongside `ROTOR_TMAX_LBS`. Measured cost of getting it wrong: at `z = 0` the pitch
axis had **zero** authority — 0.3 rad of collective tilt produced exactly 0.00000 rad/s.

**The tilt servos have rate and lag dynamics** (`rate_limit` 10.5 rad/s = 0.10 s per 60°, `lag`
50). Both numbers are representative, not measured; replace them with bench figures before trusting
a tune taken against this model. Note the model got *stricter* when these landed — `roll_pitch` now
departs where it used to survive, because instantaneous servos flattered the tune.

`ROTOR_TMAX = 1.874 lbf` (850 g per rotor) is the single number most worth replacing with
thrust-stand data.

---

## 7. Open

- **`SIM_HIL` has never been split.** It conflates two independent questions: *where does sensor
  data come from* (`SIM`) and *is this real silicon* (`HIL`). All existing sites are `SIM`
  semantics, so the rename is mechanical. The predicate that actually matters is `SIM && !HIL`,
  worth naming once (`#define EMULATED (SIM && !HIL)`) so it cannot be got wrong per call site —
  `#if !HIL` is **not** the guard for emulator-only behaviour, because ordinary flight is
  `SIM=0, HIL=0` on real silicon. Define both to 0/1 unconditionally and test with `#if`, never
  `#ifdef`: the project builds with `-Wundef`, so a typo'd flag surfaces at compile time.
- **Lock-step bridging is not implemented.** Send `SensorData` for step N, wait for that step's
  `MotorCmd`/`ServoCmd`, then advance. Bridge-only, no firmware change. It buys reproducibility,
  not correctness (§3). It cannot be made bit-exact while `Rx_Task`/`Rc_Task`/`SimTelemetry_Task`
  run on `vTaskDelay` virtual time, so budget ±1 RC frame of jitter rather than chasing equality.
- **Golden trajectories are not implemented**, deliberately, and are blocked on the above.
  KnownIssues §4.13.
- **No gyro or accel noise model** (§4). The highest-value addition to the rig.
- **Native host SIL was evaluated and set aside.** Renode runs the real ARM binary against the
  ARM_CM7 port already vendored, so the FreeRTOS MSVC-MingW port is not needed. Reconsider only if
  emulation speed proves limiting — it does not today. Cost if revisited: a
  `Firmware/platform/native/`, a Winsock `sim_link` backend, a non-ARM CMake path, and stubbing
  `hal.h` out of `core_shared.c`, `sync.c`, `uart.c`, `spi.c` and the logger — that last item is
  the sleeper cost. `Vendor/FreeRTOS` is ST's fork and ships **no MSVC-MingW port**.
