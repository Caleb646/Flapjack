# Flapjack V3 Pin Map and Connector Pinouts

**Status: draft, 2026-11-22.** MCU: STM32H747BIT6 (LQFP208).

Every pin and alternate function below was checked against ST's CubeMX database for this exact
package. Power, DSI, and bootloader rules were checked against DS12930 Rev 3, RM0399 Rev 5, and
AN2606 Rev 71.

IMU, baro, debug UART, GPS UART, servo, and motor pins the firmware already uses are unchanged.
The magnetometer moves from shared SPI5 to its own SPI4 bus.
External flash becomes 1 Gbit SPI NAND on SPI2. The receiver moves to `PD8`/`PD9` on the same USART3 because the
ROM bootloader listens on its old `PB11` input. Seven [open items](#open-items) remain.

## Stack

```text
TOP     Sensor board, 2-layer: IMU, baro, USB-C, debug probe, GPS/RC, LEDs, buttons
          |  MB1 + MB2: two keyed, shrouded 2x30 1.27 mm headers, 120 pins
MIDDLE  MCU board, 4-layer, about 46 x 50 mm, all parts on its top side
          |  air gap
BOTTOM  Power board, 4-layer: XT60, shunt, INA228, regulators, servo and ESC connectors

Sensor board to power board: PB1, one keyed, shrouded 2x15 1.27 mm connector beside the MCU board
```

Connector budget: 78 signals (45 active, 33 spare), 4 `+5V`, 4 `+3V3`, 34 `GND`.

**Orientation.** Pin numbers are defined looking down at the top of the MCU board, with MCU pin 1
at the top left. MB1 is on the left and MB2 on the right, with column 1 (pins 1 and 2) at the top.
Odd pins are the outer row. The mating parts on the underside of the sensor board must connect
pin 1 to pin 1, so check the flipped footprints with a printed 1:1 layout before ordering.

**Assembly protection.** All three headers are keyed and shrouded, and the MCU board's two mounting
screws are placed asymmetrically so it fits only one way. As a backstop, MB1 and MB2 have opposite
power ends, so an MCU board forced in rotated 180° still connects 5 V to 5 V and 3.3 V to 3.3 V.

## SPI buses

| Bus | Device | SCK / MISO / MOSI | Chip select |
|---|---|---|---|
| SPI1 | BMI323 IMU | `PA5` / `PA6` / `PA7` | `PC4` |
| SPI2 | W25N01GV flash | `PI1` / `PI2` / `PI3` | `PI0` |
| SPI4 | MMC5983 magnetometer | `PE12` / `PE13` / `PE14` | `PF4` |
| SPI5 | BMP390 baro | `PF7` / `PF8` / `PF9` | `PF6` |
| SPI3 (spare) | Unassigned | `PC10` / `PC11` / `PC12` | `PE11`, `PE10`, `PE9` |

Spare SPI3 shares its three bus pins with the spare microSD lines, so a future sensor board can use one or the other.

## Connector parts

Board-to-board connectors are Samtec Tiger Eye: shrouded, polarized, and rated from -55 to +125 °C.

| Ref | Header (lower board) | Socket (sensor board underside) | Mated height | Header size |
|---|---|---|---:|---|
| MB1, MB2 | `TFM-130-02-L-D-A` on MCU board | `SFM-130-02-L-D-A` | 6.35 mm | 41.28 × 5.72 mm |
| PB1 | `TFM-115-32-L-D-A` on power board | `SFM-115-02-L-D-A` | 11.81 mm | 22.23 × 5.72 mm |

- **Stack height:** see [Stack clearance](#stack-clearance) for the height budget and standoffs.
- **Current:** distributors list 3.2 A per contact and Samtec lists 3.7 A with two pins powered. The estimated load is about 0.4 A per `+5V_PWR` pin on PB1 and 0.15 A per `+3V3` pin on MB1 and MB2.
- **Assembly:** the `-A` suffix adds board alignment pins. For machine placement, confirm the matching `-P-TR` pick-and-place, tape-and-reel variants when ordering.

| Ref | PCB header | Cable housing | Contacts |
|---|---|---|---|
| J_GPS, J_TELEM | JST `SM06B-GHS-TB` | `GHR-06V-S` | `SSHL-002T-P0.2` |
| J_RC | JST `SM04B-GHS-TB` | `GHR-04V-S` | `SSHL-002T-P0.2` |
| ESC1, ESC2 | JST `SM02B-GHS-TB` | `GHR-02V-S` | `SSHL-002T-P0.2` |

JST-GH is latching and rated 1 A per circuit with 26 AWG wire.

## Stack clearance

The worst-case space under the MCU board is 3.55 mm: 11.81 mm PB1 height, minus 6.50 mm for MB1
and MB2 with solder paste, minus a 1.76 mm MCU board (1.6 mm + 10%). The space outside the MCU
board is 11.81 mm.

| Power-board part | Height | Location | Clearance | Result |
|---|---:|---|---:|---|
| TPS5430 | 1.66 mm | Under MCU board | 1.89 mm | OK |
| `10TPE220ML` POSCAP | 2.87 mm | Under MCU board | 0.68 mm | OK |
| `XAL7070-153MEC` inductor | 7.0 mm max | Outside MCU board | 4.81 mm | **Keep out from under MCU board** |
| `B560C-13-F`, `5.0SMDJ26A` | 2.62 mm max | Under MCU board | 0.93 mm | OK |
| `XT60PW-M` | 8.78 mm | Outside MCU board | 3.03 mm | OK |
| `EEH-ZA1H101P`, `EEE-FK1H221P` | 10.5 mm max | Outside MCU board | 1.31 mm | OK |
| Right-angle servo header | 3.02 mm | Board edge | 8.79 mm | OK |
| Vertical servo header with plug | Over 11.81 mm | Any | None | **Does not fit** |

- **Standoffs:** use 12 mm M3 between the sensor and power boards, and 6.5 mm M2 between the MCU and sensor boards. Do not use shorter standoffs, because they would force the connectors past their mated height.
- **MCU board screws:** keep power-board parts away from the two M2 screw heads under the MCU board.
- **Height sources:** the TPS5430, POSCAP, and XT60 come from the project's 3D models. The inductor, catch diode, and TVS use manufacturer maximums. The bulk capacitors use Panasonic's 10.2 ± 0.3 mm FK drawing and DigiKey's 10.5 mm hybrid listing. Mated heights vary with solder paste, so measure them on the first assembled boards.

## MB1: MCU to sensor board, left header

| Pin | Net | Net | Pin |
|---:|---|---|---:|
| 1 | `+5V` | `+5V` | 2 |
| 3 | `GND` | `GND` | 4 |
| 5 | `GND` | `GPS_SCL` | 6 |
| 7 | `GPS_SDA` | `SWO` | 8 |
| 9 | `SWDIO` | `GND` | 10 |
| 11 | `SWCLK` | `NRST` | 12 |
| 13 | `GND` | `BOOT0` | 14 |
| 15 | `BARO_SCK` | `BARO_MISO` | 16 |
| 17 | `BARO_MOSI` | `GND` | 18 |
| 19 | `BARO_CS` | `MAG_CS` | 20 |
| 21 | `GND` | `BARO_INT` | 22 |
| 23 | `MAG_INT` | `LED_CM7` | 24 |
| 25 | `LED_CM4` | `GND` | 26 |
| 27 | `LED_FAULT` | `SP_ADC1` | 28 |
| 29 | `GND` | `SP_ADC2` | 30 |
| 31 | `SP_ADC3` | `SP_ADC4` | 32 |
| 33 | `BATT_ALERT` | `GND` | 34 |
| 35 | `BATT_SDA` | `BATT_SCL` | 36 |
| 37 | `GND` | `SERVO1` | 38 |
| 39 | `SERVO3` | `PWM_SP1` | 40 |
| 41 | `PWM_SP2` | `GND` | 42 |
| 43 | `SP_T3C1` | `SP_T3C2` | 44 |
| 45 | `GND` | `SP_T3C3` | 46 |
| 47 | `SP_T3C4` | `SP_U7_TX` | 48 |
| 49 | `SP_U7_RX` | `GND` | 50 |
| 51 | `IMU_SCK` | `IMU_MISO` | 52 |
| 53 | `GND` | `IMU_MOSI` | 54 |
| 55 | `IMU_CS` | `IMU_INT` | 56 |
| 57 | `GND` | `GND` | 58 |
| 59 | `+3V3` | `+3V3` | 60 |

## MB2: MCU to sensor board, right header

| Pin | Net | Net | Pin |
|---:|---|---|---:|
| 1 | `+3V3` | `+3V3` | 2 |
| 3 | `GND` | `GND` | 4 |
| 5 | `GND` | `GPS_TX` | 6 |
| 7 | `GPS_RX` | `SP_CAN1_RX` | 8 |
| 9 | `SP_CAN1_TX` | `GND` | 10 |
| 11 | `SP_GPIO1` | `SP_GPIO2` | 12 |
| 13 | `GND` | `SP_U6_TX` | 14 |
| 15 | `SP_U6_RX` | `SP_U6_RTS` | 16 |
| 17 | `SP_U6_CTS` | `GND` | 18 |
| 19 | `SP_SD_D0` | `SP_SD_D1` | 20 |
| 21 | `GND` | `SP_SD_D2` | 22 |
| 23 | `SP_SD_D3` | `SP_SD_CK` | 24 |
| 25 | `SP_SD_CMD` | `GND` | 26 |
| 27 | `RC_TX` | `RC_RX` | 28 |
| 29 | `GND` | `USB_VBUS` | 30 |
| 31 | `USB_DM` | `USB_DP` | 32 |
| 33 | `SP_GPIO3` | `GND` | 34 |
| 35 | `SP_GPIO4` | `DBG_TX` | 36 |
| 37 | `GND` | `DBG_RX` | 38 |
| 39 | `SP_T4C3` | `SP_T4C4` | 40 |
| 41 | `SERVO2` | `GND` | 42 |
| 43 | `SERVO4` | `MOTOR1` | 44 |
| 45 | `GND` | `MOTOR2` | 46 |
| 47 | `SP_CAN2_RX` | `SP_CAN2_TX` | 48 |
| 49 | `SP_SPI3_CS1` | `GND` | 50 |
| 51 | `SP_SPI3_CS2` | `SP_SPI3_CS3` | 52 |
| 53 | `GND` | `MAG_SCK` | 54 |
| 55 | `MAG_MISO` | `MAG_MOSI` | 56 |
| 57 | `GND` | `GND` | 58 |
| 59 | `+5V` | `+5V` | 60 |

## PB1: Sensor board to power board

2x15, 1.27 mm. `+5V_PWR` is the electronics rail from the power board. It feeds the USB power mux
on the sensor board, whose output is the `+5V` net on MB1 and MB2. J_GPS, J_RC, and J_TELEM use `+5V_EXT` from a
current-limited switch.
`+3V3` comes from the MCU board and powers only the INA228. Ground separates the two rails, and
neither rail reaches a signal pin if this header is rotated or offset by one column.

| Pin | Net | Net | Pin |
|---:|---|---|---:|
| 1 | `+5V_PWR` | `GND` | 2 |
| 3 | `+5V_PWR` | `SERVO1` | 4 |
| 5 | `+5V_PWR` | `SERVO2` | 6 |
| 7 | `GND` | `MOTOR1` | 8 |
| 9 | `BATT_SDA` | `GND` | 10 |
| 11 | `BATT_SCL` | `GND` | 12 |
| 13 | `BATT_ALERT` | `GND` | 14 |
| 15 | `SERVO3` | `PWM_SP1` | 16 |
| 17 | `SERVO4` | `PWM_SP2` | 18 |
| 19 | `MOTOR2` | `GND` | 20 |
| 21 | `GND` | `GND` | 22 |
| 23 | `GND` | `GND` | 24 |
| 25 | `GND` | `GND` | 26 |
| 27 | `GND` | `GND` | 28 |
| 29 | `+3V3` | `GND` | 30 |

## MCU pin map: active signals

| Net | MCU pin | LQFP | Function | Connector | Goes to | Firmware |
|---|---|---:|---|---|---|---|
| `BARO_CS` | PF6 | 30 | GPIO | MB1.19 | BMP390 | Unchanged |
| `BARO_INT` | PF10 | 34 | EXTI10 | MB1.22 | BMP390 | Unchanged |
| `BARO_MISO` | PF8 | 32 | SPI5_MISO (AF5) | MB1.16 | BMP390 | Unchanged |
| `BARO_MOSI` | PF9 | 33 | SPI5_MOSI (AF5) | MB1.17 | BMP390 | Unchanged |
| `BARO_SCK` | PF7 | 31 | SPI5_SCK (AF5) | MB1.15 | BMP390 | Unchanged |
| `BATT_ALERT` | PF2 | 24 | EXTI2 | MB1.33 | PB1 to INA228 | New |
| `BATT_SCL` | PF1 | 23 | I2C2_SCL (AF4) | MB1.36 | PB1 to INA228 | New |
| `BATT_SDA` | PF0 | 22 | I2C2_SDA (AF4) | MB1.35 | PB1 to INA228 | New |
| `DBG_RX` | PB15 | 104 | USART1_RX (AF4) | MB2.38 | Debug probe UART | Unchanged |
| `DBG_TX` | PB14 | 103 | USART1_TX (AF4) | MB2.36 | Debug probe UART | Unchanged |
| `GPS_SCL` | PB8 | 195 | I2C1_SCL (AF4) | MB1.6 | J_GPS compass | New |
| `GPS_SDA` | PB9 | 196 | I2C1_SDA (AF4) | MB1.7 | J_GPS compass | New |
| `GPS_RX` | PD6 | 176 | USART2_RX (AF7) | MB2.7 | J_GPS | Unchanged |
| `GPS_TX` | PD5 | 175 | USART2_TX (AF7) | MB2.6 | J_GPS | Unchanged |
| `IMU_CS` | PC4 | 61 | GPIO | MB1.55 | BMI323 | Unchanged |
| `IMU_INT` | PC5 | 62 | EXTI5 | MB1.56 | BMI323 | Unchanged |
| `IMU_MISO` | PA6 | 59 | SPI1_MISO (AF5) | MB1.52 | BMI323 | Unchanged |
| `IMU_MOSI` | PA7 | 60 | SPI1_MOSI (AF5) | MB1.54 | BMI323 | Unchanged |
| `IMU_SCK` | PA5 | 58 | SPI1_SCK (AF5) | MB1.51 | BMI323 | Unchanged |
| `LED_CM4` | PE4 | 3 | GPIO | MB1.25 | Status LED | New |
| `LED_CM7` | PE3 | 2 | GPIO | MB1.24 | Status LED | New |
| `LED_FAULT` | PE5 | 4 | GPIO | MB1.27 | Status LED | New |
| `MAG_CS` | PF4 | 26 | GPIO | MB1.20 | MMC5983 | Unchanged |
| `MAG_INT` | PF3 | 25 | EXTI3 | MB1.23 | MMC5983 | Unchanged |
| `MAG_MISO` | PE13 | 84 | SPI4_MISO (AF5) | MB2.55 | MMC5983 | New |
| `MAG_MOSI` | PE14 | 85 | SPI4_MOSI (AF5) | MB2.56 | MMC5983 | New |
| `MAG_SCK` | PE12 | 83 | SPI4_SCK (AF5) | MB2.54 | MMC5983 | New |
| `MOTOR1` | PH6 | 92 | TIM12_CH1 (AF2) | MB2.44 | PB1 to power board | Unchanged |
| `MOTOR2` | PH9 | 95 | TIM12_CH2 (AF2) | MB2.46 | PB1 to power board | Unchanged |
| `PWM_SP1` | PA2 | 47 | TIM5_CH3 (AF2) | MB1.40 | PB1 to power board | Spare |
| `PWM_SP2` | PA3 | 54 | TIM5_CH4 (AF2) | MB1.41 | PB1 to power board | Spare |
| `SERVO1` | PA0 | 45 | TIM5_CH1 (AF2) | MB1.38 | PB1 to power board | Unchanged |
| `SERVO3` | PA1 | 46 | TIM5_CH2 (AF2) | MB1.39 | PB1 to power board | New |
| `SERVO2` | PC6 | 142 | TIM8_CH1 (AF3) | MB2.41 | PB1 to power board | Unchanged |
| `SERVO4` | PC7 | 143 | TIM8_CH2 (AF3) | MB2.43 | PB1 to power board | New |
| `RC_RX` | PD9 | 106 | USART3_RX (AF7) | MB2.28 | J_RC | Moved |
| `RC_TX` | PD8 | 105 | USART3_TX (AF7) | MB2.27 | J_RC | Moved |
| `BOOT0` | BOOT0 | 194 | Dedicated | MB1.14 | Button on sensor board | New |
| `NRST` | NRST | 37 | Dedicated | MB1.12 | Debug probe, buttons | New |
| `SWCLK` | PA14 | 165 | DEBUG_JTCK-SWCLK (AF0) | MB1.11 | Debug probe, buttons | New |
| `SWDIO` | PA13 | 151 | DEBUG_JTMS-SWDIO (AF0) | MB1.9 | Debug probe, buttons | New |
| `SWO` | PB3 | 189 | DEBUG_JTDO-SWO (AF0) | MB1.8 | Debug probe, buttons | New |
| `USB_DM` | PA11 | 149 | USB_OTG_FS_DM (AF10) | MB2.31 | USB-C (0 Ω option) | New |
| `USB_DP` | PA12 | 150 | USB_OTG_FS_DP (AF10) | MB2.32 | USB-C (0 Ω option) | New |
| `USB_VBUS` | PA9 | 147 | USB_OTG_FS_VBUS | MB2.30 | USB-C (0 Ω option) | New |

## MCU pin map: spare signals

Spares reach the sensor board through MB1 and MB2. A sensor-board revision can use them without
changing the MCU board. All interrupt-capable pins use different EXTI line numbers, which the
firmware's EXTI dispatcher requires.

| Net | MCU pin | LQFP | Function | Connector | Goes to | Firmware |
|---|---|---:|---|---|---|---|
| `SP_ADC1` | PC0 | 38 | ADC3_INP10 | MB1.28 | Spare ADC | Spare |
| `SP_ADC2` | PC1 | 39 | ADC3_INP11 | MB1.30 | Spare ADC | Spare |
| `SP_ADC3` | PC2_C | 40 | ADC3_INP0 | MB1.31 | Spare ADC | Spare |
| `SP_ADC4` | PC3_C | 41 | ADC3_INP1 | MB1.32 | Spare ADC | Spare |
| `SP_CAN1_RX` | PD0 | 170 | FDCAN1_RX (AF9) | MB2.8 | Spare CAN | Spare |
| `SP_CAN1_TX` | PD1 | 171 | FDCAN1_TX (AF9) | MB2.9 | Spare CAN | Spare |
| `SP_CAN2_RX` | PB12 | 101 | FDCAN2_RX (AF9) | MB2.47 | Spare CAN | Spare |
| `SP_CAN2_TX` | PB13 | 102 | FDCAN2_TX (AF9) | MB2.48 | Spare CAN | Spare |
| `SP_GPIO1` | PD4 | 174 | EXTI4 | MB2.11 | Spare GPIO/EXTI | Spare |
| `SP_GPIO2` | PD7 | 177 | EXTI7 | MB2.12 | Spare GPIO/EXTI | Spare |
| `SP_GPIO3` | PG12 | 183 | EXTI12 | MB2.33 | Spare GPIO/EXTI | Spare |
| `SP_GPIO4` | PG13 | 184 | EXTI13 | MB2.35 | Spare GPIO/EXTI | Spare |
| `SP_SD_CK` | PC12 | 169 | SDMMC1_CK (AF12) | MB2.24 | Spare microSD | Spare |
| `SP_SD_CMD` | PD2 | 172 | SDMMC1_CMD (AF12) | MB2.25 | Spare microSD | Spare |
| `SP_SD_D0` | PC8 | 144 | SDMMC1_D0 (AF12) | MB2.19 | Spare microSD | Spare |
| `SP_SD_D1` | PC9 | 145 | SDMMC1_D1 (AF12) | MB2.20 | Spare microSD | Spare |
| `SP_SD_D2` | PC10 | 167 | SDMMC1_D2 (AF12) | MB2.22 | Spare microSD | Spare |
| `SP_SD_D3` | PC11 | 168 | SDMMC1_D3 (AF12) | MB2.23 | Spare microSD | Spare |
| `SP_SPI3_CS1` | PE11 | 82 | GPIO | MB2.49 | Spare SPI3 chip select | Spare |
| `SP_SPI3_CS2` | PE10 | 81 | GPIO | MB2.51 | Spare SPI3 chip select | Spare |
| `SP_SPI3_CS3` | PE9 | 78 | GPIO | MB2.52 | Spare SPI3 chip select | Spare |
| `SP_T3C1` | PB4 | 190 | TIM3_CH1 (AF2) | MB1.43 | Spare timer | Spare |
| `SP_T3C2` | PB5 | 191 | TIM3_CH2 (AF2) | MB1.44 | Spare timer | Spare |
| `SP_T3C3` | PB0 | 63 | TIM3_CH3 (AF2) | MB1.46 | Spare timer | Spare |
| `SP_T3C4` | PB1 | 64 | TIM3_CH4 (AF2) | MB1.47 | Spare timer | Spare |
| `SP_T4C3` | PD14 | 115 | TIM4_CH3 (AF2) | MB2.39 | Spare timer | Spare |
| `SP_T4C4` | PD15 | 116 | TIM4_CH4 (AF2) | MB2.40 | Spare timer | Spare |
| `SP_U7_RX` | PE7 | 76 | UART7_RX (AF7) | MB1.49 | Spare UART | Spare |
| `SP_U7_TX` | PE8 | 77 | UART7_TX (AF7) | MB1.48 | Spare UART | Spare |
| `SP_U6_CTS` | PG15 | 188 | USART6_CTS (AF7) | MB2.17 | J_TELEM (spare) | Spare |
| `SP_U6_RTS` | PG8 | 138 | USART6_RTS (AF7) | MB2.16 | J_TELEM (spare) | Spare |
| `SP_U6_RX` | PG9 | 180 | USART6_RX (AF7) | MB2.15 | J_TELEM (spare) | Spare |
| `SP_U6_TX` | PG14 | 185 | USART6_TX (AF7) | MB2.14 | J_TELEM (spare) | Spare |

## MCU pins that stay on the MCU board

| Net | MCU pin | LQFP | Function |
|---|---|---:|---|
| `FLASH_CS` | PI0 | 159 | GPIO |
| `FLASH_SCK` | PI1 | 162 | SPI2_SCK (AF5) |
| `FLASH_MISO` | PI2 | 163 | SPI2_MISO (AF5) |
| `FLASH_MOSI` | PI3 | 164 | SPI2_MOSI (AF5) |
| `HSE_IN` | PH0 | 35 | RCC_OSC_IN |
| `HSE_OUT` | PH1 | 36 | RCC_OSC_OUT |

| Pin | Connection | Source or reason |
|---|---|---|
| `PDR_ON` (201) | 3.3 V. V2 leaves it floating. | Tying it to VSS turns the internal reset off (DS12930 §3.5.2) |
| `BOOT0` (194) | 10 kΩ pulldown on MCU board, button to 3.3 V on sensor board | Enters the ROM bootloader (AN2606 Pattern 10) |
| `NRST` (37) | 100 nF on MCU board, button to ground on sensor board | Same capacitor as V2 |
| `VBAT` (8) | 3.3 V | Required when no battery is used (RM0399 §7.4) |
| `VDDA` (44), `VREF+` (43) | `+3V3` through ferrite FB2, with 1 µF + 100 nF at each pin | Analog supply filtering |
| `VDD33USB` (141), `VDD50USB` (140) | Both to 3.3 V | Required with the USB regulator off (RM0399 §7.4) |
| `VCAPDSI` (119), `VDD12DSI` (120, 126) | Tied together, no capacitor | DSI unused (RM0399 §7.4.8) |
| `VSSDSI` (123, 129) | Ground | DSI unused (RM0399 §7.4.8) |
| DSI data and clock pins | No connect | DSI unused |
| SWD test pads | `+3V3`, `SWDIO`, `SWCLK`, `SWO`, `NRST`, `BOOT0`, `GND` | Standalone bring-up |

## External flash

**Part:** Winbond W25N01GVZEIG, 1 Gbit (128 MB) 3.3 V SLC NAND, WSON-8 (8 × 6 mm), on SPI2.

- **Why NAND:** flight logs are written once in order, which suits its 2 KB pages and 128 KB blocks. Typical page program is 250 µs and block erase is 2 ms.
- **Capacity:** about 55 minutes at an estimated 40 kB/s (100-byte records at 400 Hz).
- **Why SPI instead of QUADSPI:** logging needs far less bandwidth than one SPI lane provides, and firmware runs from internal flash. QUADSPI pins are either spread across the MCU package or would displace the baro and magnetometer buses. SPI2 on `PI0` to `PI3` sits together on one side.
- **Wiring:** place the part beside MCU pins 159 to 164. Pull `/WP` and `/HOLD` to 3.3 V through 10 kΩ, and fit 100 nF at `VCC`.
- **Do not fit the W25N01GW.** The current firmware header names that part, and it is a 1.8 V device.
- **Log download:** logs come out over the debug UART through the USB-C debug probe. A 10-minute log is about 24 MB, which takes about 80 seconds at the 3 Mbaud target and about 9 minutes at today's 460800 baud.

## 3.3 V regulator

`+5V` feeds a TI `TPS62912RPUR` 2 A, 2.2 MHz synchronous buck on the MCU board.

- **Parts:** `XGL4020-222MEC` inductor, `BLE18PS080SN1D` output ferrite, `MPZ1005S121HT000` VDDA and VREF+ ferrite, 2 × `C2012X7S1E106K125AC` + `GRM155R71E222KA01D` input, and 6 × `C2012X7S1A226M125AC` output and post-filter. A 10.7 k / 3.40 k divider sets 3.318 V.
- **Thermal:** at 0.6 A from 5.25 V, the buck loses about 0.22 W and stays at or below a calculated 104 °C junction at 85 °C ambient.
- **Placement:** keep the inductor and switching loop away from the MMC5983.
- **MCU limit:** U1 must stay under 286 mA for a 125 °C junction at 85 °C ambient. The datasheet maximum at 400 MHz with all peripherals (541 mA) would limit ambient to about 49 °C, so measure its current or reduce the clock.

## Core supply

V3 uses the internal LDO only (RM0399 Figure 22, configuration 1). The design clock is 400 MHz at
VOS1, which leaves more thermal margin in the stack than 480 MHz at VOS0.

| Pin | Connection |
|---|---|
| `VDDSMPS` (20) | Ground |
| `VFBSMPS` (21) | Ground |
| `VSSSMPS` (18) | Ground |
| `VLXSMPS` (19) | No connect |
| `VDDLDO` (91, 154, 202) | 3.3 V |
| `VCAP` (89, 152, 199) | 2 × 2.2 µF, ESR under 100 mΩ (DS12930 Table 15) |

The MCU comes out of reset with the SMPS enabled (RM0399 Table 33), so `VDDSMPS` must be grounded.

DS12930 §3.5.1 says `VDDSMPS` must stay at `VDD`, while RM0399 §7.4 and Figure 22 say to ground it
when the SMPS is unused. This design follows RM0399, which covers that case explicitly.

## ROM bootloader protection

With `BOOT0` high, the bootloader checks FDCAN, USART, I2C, and SPI before USB (AN2606 Figure 80),
so noise or traffic on those pins can stop DFU from starting. DFU itself runs from HSI48 with CRS,
so it does not need the crystal or VBUS sensing.

| Pin | Part | Reason |
|---|---|---|
| `PA4`, `PA15`, `PE11` (`SP_SPI3_CS1`) | 10 kΩ pull-up, MCU board | Deselects bootloader SPI1, SPI3, SPI4 |
| `PI0` (`FLASH_CS`) | 10 kΩ pull-up, MCU board | Keeps flash deselected and deselects bootloader SPI2 |
| `PA8`, `PC9` | 10 kΩ pull-up, MCU board | Holds bootloader I2C3 idle |
| `PA3` (`PWM_SP2`) | 10 kΩ pull-up, MCU board | Holds bootloader USART2 RX idle |
| `PB15` (`DBG_RX`) | 10 kΩ pull-up, MCU board | A low level blocks bootloader interfaces (AN2606 Table 146) |
| `PB6` | 10 kΩ pull-up, MCU board | Holds bootloader I2C1 clock idle |
| `IMU_CS`, `BARO_CS`, `MAG_CS` | 10 kΩ pull-up, sensor board | Keeps sensors off their SPI buses |
| `GPS_SCL`, `GPS_SDA`, `BATT_SCL`, `BATT_SDA` | 2.2 kΩ pull-up, sensor board | I2C idle state |
| `MOTOR1`, `MOTOR2` | 10 kΩ pulldown, power board | Holds ESC inputs low during reset (design choice) |

Bootloader V9.0 can need a second reset after power-on. Versions before V9.1 can also lock the
supply configuration, so power-cycle after using DFU on those parts.

## Battery input surge protection

```text
XT60 (+) --> shunt --+--> ESC power pads
                     +--> bulk capacitors to ground
                     +--> TVS to ground
                     +--> servo and electronics regulators
```

- **No reverse-polarity protection:** a reversed pack can destroy the regulators, ESCs,
  capacitors, TVS, INA228, and the boards they feed. The keyed XT60 is the only safeguard. Check
  new battery leads with a meter, and use a fused smoke stopper for the first power-up of each build.
- **Bulk capacitors:** fit one Panasonic `EEH-ZA1H101P` (100 µF, 50 V hybrid, 28 mΩ) and one
  `EEE-FK1H221P` (220 µF, 50 V, about 180 mΩ) from the load side of the shunt to ground. The
  hybrid supplies fast current and the higher-ESR capacitor damps ringing. They do not replace
  the input capacitors on the ESCs.
- **TVS:** fit a Littelfuse `5.0SMDJ26A` (5 kW, SMC) across the battery rail, cathode to the rail
  and anode to ground. Its 26 V standoff suits standard 4.2 V per cell packs, but not 4.35 V per
  cell high-voltage packs.
- **Regulator input capacitors:** use 10 µF, 50 V X7R parts.
- **Simulated surge:** a full 6S pack reaches up to about 45 V at the regulators with ceramics
  alone. With these parts, plug-in peaks at 33.8 V (TVS at 25 °C) to 34.7 V (85 °C), against the
  TPS5430's 36 V operating and 40 V absolute limits. ESC braking at 60 A with the battery connected
  reaches 30.7 V.
- **Battery disconnect during regen:** the rail reaches 35.3 V at 20 A, 36.3 V at 30 A, and 39.0 V
  at 60 A. The TVS is rated for about 6 to 26 J, while two rotors hold an estimated 38 to 128 J, so
  it cannot absorb a full regen stop into a disconnected bus. It typically fails short across the
  battery, and the board has no fuse.
- **ESC path:** no series diode or ideal-diode controller. Motor braking returns current to the
  battery, and blocking it can create a voltage spike that damages the ESCs.

## 5 V regulators

All three TPS5430 regulators use a Coilcraft `XAL7070-153MEC` inductor (15 µH, 7.0 A saturation), a Diodes Inc. `B560C-13-F` catch diode (60 V, 5 A), and a 10 k / 3.24 k feedback divider.

C24 (`+5V_PWR`), C25 (`+5V_SERVO_L`), and C26 (`+5V_SERVO_R`) are do-not-populate 470 pF C0G footprints across each 10 k feedback resistor. Populated, the model raises worst-case phase margin to 43.4° on the electronics rail and 46.2° on the servo rails, assuming an ideal VSENSE input. Populate them only if a bench load step shows inadequate damping.

## Servo rail decoupling

Each of the two servo rails (left: S1 and S3, right: S2 and S4) gets:

| Part | Quantity per rail | Placement |
|---|---:|---|
| `10TPE220ML`, 220 µF, 10 V POSCAP | 2 | One at the regulator output, one near the servo headers |
| 10 µF, 16 V X7R ceramic | 2 | One within a few mm of each servo header |
| 100 nF X7R ceramic | 2 | One directly at each servo header |

- **Why two POSCAPs:** modeling the TPS5430's internal compensation (SLVS632L Equations 7 and 15) with the XAL7070 inductor over component tolerances gives a worst-case phase margin of 28.8° with one POSCAP, 37.2° with two, and 31.3° with three.
- **Do not add more bulk capacitance** without rechecking regulator stability.
- **Capacitors do not cover a stall:** 440 µF supplies 1 A above the regulator's limit for only about 110 µs at 0.25 V of sag. Each rail's sustained servo current must stay within the regulator's 3 A rating.

## USB and external port power

- **Power mux:** a `TPS2116DRLR` selects `+5V_PWR` first, then USB VBUS, and outputs `+5V`.
- **Hold-up:** C28, a KEMET `T520W107M010ATE040` (100 µF, 10 V), limits switchover droop on `+5V` to about 0.17 V at 1.2 A.
- **External ports:** a `TPS2553DBVR` current-limited switch (544 to 673 mA) feeds `+5V_EXT` to J_GPS, J_RC, and J_TELEM.
- **USB-only load:** about 530 mA with no external devices and 850 mA with the estimated GPS, receiver, and telemetry load. Use a USB-C source rated at least 1.5 A for full bench operation.

## Electronics rail decoupling

The electronics regulator gets two `10TPE220ML` 220 µF, 10 V POSCAPs: one at the regulator output
and one near the `+5V_PWR` pins of PB1.

- **Modeled result:** with the full stack (XAL7070, TPS2116 mux, 100 µF hold-up capacitor, and four 10 µF + 100 nF pairs), worst-case phase margin is 36.0°.
- **Do not add more bulk capacitance** to this rail without rechecking regulator stability.

## External connectors

Signal names are from the flight controller's side. Its `TX` connects to the device's `RX`.

**J_GPS, sensor board, JST-GH 6-pin.** Check your BN-880 cable's order before crimping.

| Pin | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| Net | `+5V_EXT` | `GPS_TX` | `GPS_RX` | `GPS_SCL` | `GPS_SDA` | `GND` |

**J_RC, sensor board, JST-GH 4-pin.**

| Pin | 1 | 2 | 3 | 4 |
|---|---|---|---|---|
| Net | `+5V_EXT` | `RC_TX` | `RC_RX` | `GND` |

**J_TELEM, sensor board, JST-GH 6-pin (spare).**

| Pin | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| Net | `+5V_EXT` | `SP_U6_TX` | `SP_U6_RX` | `SP_U6_CTS` | `SP_U6_RTS` | `GND` |

**J_USB, sensor board, USB-C.** `VBUS` feeds the power mux and reaches `USB_VBUS` through a 10 kΩ
/ 15 kΩ divider. That keeps `PA9` under 4 V with its internal pull-up active (DS12930 Table 10) and
limits current if the bootloader drives the pin as USART1 TX (AN2606 Table 146).
`CC1` and `CC2` each get 5.1 kΩ to ground. `D+` and `D-` go to the debug probe by default, with a
0 Ω option to `USB_DP` and `USB_DM` for STM32 DFU.

**Servo outputs, power board edge, right-angle 3-pin 2.54 mm (Samtec `TSW-103-08-G-S-RA`).** Pin 1
`GND`, pin 2 servo rail, pin 3 signal. Vertical headers do not fit under the sensor board.

| Output | Signal | Timer | Rail |
|---|---|---|---|
| S1 (left tilt) | `SERVO1` | TIM5_CH1 | Left |
| S2 (right tilt) | `SERVO2` | TIM8_CH1 | Right |
| S3 | `SERVO3` | TIM5_CH2 | Left |
| S4 | `SERVO4` | TIM8_CH2 | Right |

**ESC signals, power board, JST-GH 2-pin.** Pin 1 `GND`, pin 2 signal. ESC1 is `MOTOR1` and ESC2
is `MOTOR2`. Both must stay on one GPIO port because the DShot driver writes them with a single
DMA transfer. `PWM_SP1` and `PWM_SP2` end at test pads on the power board.

## Firmware changes this map requires

- Add a `flapjack_v3` board target with `USE_PWR_LDO_SUPPLY`.
- Rework the unused flash driver for the W25N01GV on SPI2 (`PI0` to `PI3`), and add read, block erase, and bad-block handling.
- Raise the debug UART (`USART1`) from 460800 baud to a 3 Mbaud target for log download.
- Move `UART_3` (receiver) to `PD8`/`PD9`. The instance and AF7 stay the same.
- Move the magnetometer from SPI5 to SPI4 on `PE12` to `PE14` (AF5). Its chip select and interrupt stay on `PF4` and `PF3`.
- Enable `USB33DEN` before using native USB (RM0399 §7.4).
- Set `HSE_VALUE`, run SYSCLK from the PLL at 400 MHz (VOS1), and set matching flash wait states and bus dividers.
- Add drivers for the INA228 (I2C2, `PF2` alert, `ADCRANGE=1`) and the GPS compass (I2C1).
- Add a status LED driver on `PE3`, `PE4`, and `PE5`. The firmware does not drive LEDs today.
- Enable `SERVO3` and `SERVO4` when the airframe uses them. The firmware drives two servos today.

## Open items

1. **Stack dry-fit.** On the first assembled boards, measure both connector mated heights and confirm the standoffs, JST-GH connectors, and bulk capacitors fit as budgeted.
2. **Plug-in surge measurement.** On the built board, scope the regulator rail while connecting a full pack. It should stay below 36 V.
3. **BN-880 cable pin order.** Check it against J_GPS.
4. **Debug UART speed.** Confirm on the bench that the RP2040 debug probe sustains 3 Mbaud without dropping data, and lower the target if it does not.
5. **Servo rail measurement.** Measure servo stall current, then scope each rail during servo starts, reversals, and a load step.
6. **5 V regulator damping.** Load-step all three rails. Populate C24 to C26 only if damping is inadequate.
7. **3.3 V and MCU thermal.** Measure buck ripple, buck temperature, and U1 supply current at the flight clock.
