# Graph Report - .  (2026-05-08)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 447 nodes · 736 edges · 62 communities (60 shown, 2 thin omitted)
- Extraction: 82% EXTRACTED · 18% INFERRED · 0% AMBIGUOUS · INFERRED: 134 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `672c6f5e`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- [[_COMMUNITY_Flight Control & PID|Flight Control & PID]]
- [[_COMMUNITY_IMU & Sensor Fusion|IMU & Sensor Fusion]]
- [[_COMMUNITY_Motor & Servo Control|Motor & Servo Control]]
- [[_COMMUNITY_GPS & NMEA Parsing|GPS & NMEA Parsing]]
- [[_COMMUNITY_SPI Bus & Devices|SPI Bus & Devices]]
- [[_COMMUNITY_Core Scheduler & RC|Core Scheduler & RC]]
- [[_COMMUNITY_Logging & Ring Buffer|Logging & Ring Buffer]]
- [[_COMMUNITY_Serial & CRSF RX|Serial & CRSF RX]]
- [[_COMMUNITY_Dual-Core Sync|Dual-Core Sync]]
- [[_COMMUNITY_System Calls (Newlib)|System Calls (Newlib)]]
- [[_COMMUNITY_Hash Map (UMap)|Hash Map (UMap)]]
- [[_COMMUNITY_DMA Controllers|DMA Controllers]]
- [[_COMMUNITY_Magnetometer|Magnetometer]]
- [[_COMMUNITY_Graphiti & Claude Setup|Graphiti & Claude Setup]]
- [[_COMMUNITY_String Formatting|String Formatting]]
- [[_COMMUNITY_Install & Download|Install & Download]]
- [[_COMMUNITY_Queue Data Structure|Queue Data Structure]]
- [[_COMMUNITY_STM32 Fault Handlers|STM32 Fault Handlers]]
- [[_COMMUNITY_System Init (STM32)|System Init (STM32)]]
- [[_COMMUNITY_Firmware Build Script|Firmware Build Script]]
- [[_COMMUNITY_Heap & Memory|Heap & Memory]]
- [[_COMMUNITY_Hardware Test Runner|Hardware Test Runner]]

## God Nodes (most connected - your core abstractions)
1. `main()` - 14 edges
2. `minmea_scan()` - 13 edges
3. `DelayMicroseconds()` - 13 edges
4. `IMUReadReg()` - 13 edges
5. `IMU_Init_()` - 11 edges
6. `GetMicroseconds()` - 11 edges
7. `DMAGetStreamById()` - 10 edges
8. `IMUCalibrate()` - 10 edges
9. `IMU_LogError()` - 9 edges
10. `IMUWriteReg()` - 9 edges

## Surprising Connections (you probably didn't know these)
- `main()` --calls--> `Spi_InitSystem()`  [INFERRED]
  Common/Src/main.c → Common/Src/drivers/bus/spi.c
- `main()` --calls--> `Uart_InitSystem()`  [INFERRED]
  Common/Src/main.c → Common/Src/drivers/serial/uart.c
- `DShotBB_Init()` --calls--> `HwTestDShot_Run()`  [INFERRED]
  Common/Src/mc/dshot.c → HwTests/hw_test_dshot.c
- `DShotBB_Write()` --calls--> `SendAndCapture()`  [INFERRED]
  Common/Src/mc/dshot.c → HwTests/hw_test_dshot.c
- `DShotBB_Write()` --calls--> `HwTestDShot_Run()`  [INFERRED]
  Common/Src/mc/dshot.c → HwTests/hw_test_dshot.c

## Communities (62 total, 2 thin omitted)

### Community 0 - "Flight Control & PID"
Cohesion: 0.08
Nodes (31): clipf32(), mapf32(), Imu_Get(), Fc_Get(), Fc_Init(), Fc_IsArmed(), Fc_LogData(), Mag_Get() (+23 more)

### Community 1 - "IMU & Sensor Fusion"
Cohesion: 0.16
Nodes (33): Delay(), IMU2CPUInterruptHandler(), IMU_Init_(), Imu_LogData_(), IMU_LogDeviceConf(), IMU_LogDeviceErr(), IMU_LogError(), IMU_LogFeatStatus() (+25 more)

### Community 2 - "Motor & Servo Control"
Cohesion: 0.11
Nodes (20): HwTestDShot_Run(), SendAndCapture(), SetupInputCapture(), HwTest_Printf(), HwTest_UartInit(), main(), SystemClock_Config(), HwTest_ReportFail() (+12 more)

### Community 3 - "GPS & NMEA Parsing"
Cohesion: 0.12
Nodes (17): Gps_Init_(), Gps_Update_(), hex2int(), minmea_check(), minmea_isfield(), minmea_parse_gbs(), minmea_parse_gga(), minmea_parse_gll() (+9 more)

### Community 4 - "SPI Bus & Devices"
Cohesion: 0.14
Nodes (22): Spi_GetBusById(), Spi_InitSystem(), Spi_IrqHandler(), SpiDev_Init(), SpiDev_ReadRegister(), SpiDev_Transactions(), SpiDev_Write(), SpiDev_WriteRegister() (+14 more)

### Community 5 - "Core Scheduler & RC"
Cohesion: 0.13
Nodes (16): __assert_func(), CoreShared_Init(), CriticalErrorHandler(), DelayMicroseconds(), DWTInit(), GetMicroseconds(), GetMilliseconds(), SanityCheckTimers() (+8 more)

### Community 6 - "Logging & Ring Buffer"
Cohesion: 0.14
Nodes (16): LoggerGetMyRingBuf(), LoggerGetOtherRingBuf(), LoggerSyncUARTTaskHandler(), LoggerWriteChar_Blocking(), LoggerWriteChar_NonBlocking(), LoggerWriteToSinks(), RingBuffAdvance(), RingBuffGetFree() (+8 more)

### Community 7 - "Serial & CRSF RX"
Cohesion: 0.11
Nodes (18): LoggerAddSink(), crc8(), Crsf_Bind(), Crsf_DataReceivedHandler_(), Crsf_Init(), Crsf_MapChannel(), Crsf_ProcessFrame(), Rx_Init() (+10 more)

### Community 8 - "Dual-Core Sync"
Cohesion: 0.14
Nodes (20): Core_Init(), CM4_SEV_IRQHandler(), CM7_SEV_IRQHandler(), LockRelease(), LockTake(), SyncGetOtherCoresMailBoxID(), SyncGetTaskHandler(), SyncInit() (+12 more)

### Community 9 - "System Calls (Newlib)"
Cohesion: 0.19
Nodes (18): _close(), _execve(), _exit(), _fork(), _fstat(), _getpid(), initialise_monitor_handles(), _isatty() (+10 more)

### Community 10 - "Hash Map (UMap)"
Cohesion: 0.2
Nodes (10): UMap_Contains(), UMap_Find(), UMap_FindIndex(), UMap_FindPtr(), UMap_GetKey(), UMap_GetValue(), UMap_Init(), UMap_InitWithFunctions() (+2 more)

### Community 12 - "DMA Controllers"
Cohesion: 0.29
Nodes (13): DMA1_Stream0_IRQHandler(), DMA1_Stream1_IRQHandler(), DMA1_Stream2_IRQHandler(), DMA1_Stream3_IRQHandler(), DMA1_Stream4_IRQHandler(), DMA1_Stream5_IRQHandler(), DMA1_Stream6_IRQHandler(), DMAClockInit() (+5 more)

### Community 13 - "Magnetometer"
Cohesion: 0.29
Nodes (12): Mag_Init_(), Mag_IsEnabled(), Mag_Update_(), MagControlRegWrite(), MagRaw2NormedGauss(), MagReadStatusReg(), MagSoftReset(), MagUpdateFromINT() (+4 more)

### Community 14 - "Graphiti & Claude Setup"
Cohesion: 0.24
Nodes (12): check_claude_code(), check_docker(), check_graphiti(), check_user_exists(), main(), Checks for Docker and installs it if missing., Checks for Claude Code CLI and installs if missing., Checks for Graphiti repository and sets up the server. (+4 more)

### Community 15 - "String Formatting"
Cohesion: 0.33
Nodes (11): a2d(), a2i(), i2a(), li2a(), putchw(), putcp(), tfp_format(), tfp_printf() (+3 more)

### Community 16 - "Install & Download"
Cohesion: 0.35
Nodes (9): _check_paths(), download_and_extract(), install_linux_tools(), install_msys(), main(), run_msys_command(), verify_tools(), verify_tools_linux() (+1 more)

### Community 17 - "Queue Data Structure"
Cohesion: 0.27
Nodes (5): Queue_Peek(), Queue_Pop(), Queue_Push(), QueueIsEmpty(), QueueIsFull()

### Community 18 - "STM32 Fault Handlers"
Cohesion: 0.43
Nodes (6): BusFault_Handler(), DebugMon_Handler(), HardFault_Handler(), MemManage_Handler(), NMI_Handler(), UsageFault_Handler()

### Community 19 - "System Init (STM32)"
Cohesion: 0.6
Nodes (3): ExitRun0Mode(), SystemCoreClockUpdate(), SystemInit()

### Community 20 - "Firmware Build Script"
Cohesion: 0.7
Nodes (4): main(), parse_arguments(), run_cmake_build(), verify_toolchain()

## Knowledge Gaps
- **5 isolated node(s):** `Checks for Docker and installs it if missing.`, `Checks for Claude Code CLI and installs if missing.`, `Checks for Graphiti repository and sets up the server.`, `Checks if a binary exists in the system PATH.`, `Executes a command and exits on failure.`
  These have ≤1 connection - possible missing edges or undocumented components.
- **2 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `main()` connect `Core Scheduler & RC` to `Flight Control & PID`, `IMU & Sensor Fusion`, `SPI Bus & Devices`, `Serial & CRSF RX`, `Dual-Core Sync`, `Magnetometer`?**
  _High betweenness centrality (0.147) - this node is a cross-community bridge._
- **Why does `SerialDebug_Init_()` connect `Serial & CRSF RX` to `Core Scheduler & RC`?**
  _High betweenness centrality (0.089) - this node is a cross-community bridge._
- **Why does `Core_Init()` connect `Dual-Core Sync` to `Core Scheduler & RC`?**
  _High betweenness centrality (0.083) - this node is a cross-community bridge._
- **Are the 12 inferred relationships involving `main()` (e.g. with `Fc_Init()` and `Pid_Init_()`) actually correct?**
  _`main()` has 12 INFERRED edges - model-reasoned connections that need verification._
- **Are the 9 inferred relationships involving `DelayMicroseconds()` (e.g. with `Scheduler_Main()` and `FlashWaitWriteInProgress()`) actually correct?**
  _`DelayMicroseconds()` has 9 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `IMUReadReg()` (e.g. with `SpiDev_ReadRegister()` and `DelayMicroseconds()`) actually correct?**
  _`IMUReadReg()` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `IMU_Init_()` (e.g. with `main()` and `SpiDev_Init()`) actually correct?**
  _`IMU_Init_()` has 2 INFERRED edges - model-reasoned connections that need verification._