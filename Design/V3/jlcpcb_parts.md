# Flapjack V3 JLCPCB Parts Audit

**Audit date:** 2026-11-22
**Design:** branch `design/v3-pinmap`, commit `32485c2`, schematics exported with KiCad 10 CLI
**Build:** 5 sets of three boards
**Scope:** read-only. Every substitute below is a proposal and none has been applied.

## Readiness summary

**Not ready to order.** The BOM is mostly obtainable, but two things stop an order today:

1. **No board layouts exist.** There is no `.kicad_pcb` for any board, so JLCPCB Gerber and placement files cannot be produced.
2. **Three parts have no confirmed purchasable stock:** `STM32H747BIT6`, `BMP390`, and `INA228AIDGSR`.

| Board | Ready | Needs substitute | Global Sourcing | Supply blocker | User-supplied hand-solder | Consign | BOM lines |
|---|---:|---:|---:|---:|---:|---:|---:|
| MCU board | 16 | 2 | 1 | 1 | 1 | 0 | 21 |
| Sensor board | 35 | 0 | 1 | 1 | 4 | 0 | 41 |
| Power board | 11 | 3 | 2 | 1 | 4 | 0 | 21 |

Counts are unique BOM lines. Mounting holes, test points, wire pads, and DNP parts are excluded. DNP parts stay unpopulated: sensor `R15`, `R16` and power `C24` to `C26`.

Sourcing order used: exact stocked part, verified stocked substitute, then Global Sourcing. No consignment is planned. Samtec board-to-board connectors, JST-GH connectors, the XT60, servo headers, and battery/ESC wires are user-supplied and hand-soldered, so they are not JLCPCB blockers.

### Verified facts and estimates

- **Verified on 2026-11-22:** JLCPCB part number, MPN, library class, stock, and price ladder for every linked stocked part, read from its public JLCPCB part page.
- **Verified from manufacturer data:** the accepted substitutes listed in [Proposed substitutes](#proposed-substitutes).
- **Estimated:** Global Sourcing prices, purchase quantities, attrition, and all totals. Global Sourcing availability requires sign-in and was not checked.
- **Snapshot risk:** DigiKey, Mouser, ST, and Octopart figures came from search snapshots and some conflict. Recheck them before buying.
- **Class caveat:** `Extended*` means JLCPCB's page shows Extended and the public index could not confirm whether it is Preferred Extended.
- **Search limit:** `None found` means the exact MPN did not appear in public JLCPCB, LCSC, or parts-index searches. A signed-in library search could differ.

## MCU board

| Refs | Qty/set | Value | Footprint | BOM MPN | JLCPCB part | Class | Stock | Buy for 5 sets | Unit USD | Status | Note |
|---|---:|---|---|---|---|---|---:|---:|---:|---|---|
| U1 | 1 | MCU | STM32H747BIT6-LQFP-208 | STM32H747BIT6 | [C730197](https://jlcpcb.com/partdetail/C730197) | Extended | 0 | 6 | $24.12 est. | Global Sourcing (blocker) | JLCPCB stock 0; DigiKey and ST eStore snapshots also out of stock |
| U2 | 1 | 3.3 V buck | Texas RPU0010A VQFN-HR-10 2x2mm P0 | TPS62912RPUR | [C3198056](https://jlcpcb.com/partdetail/C3198056) | Extended* | 1,409 | 5 | $1.91 | Ready |  |
| U3 | 1 | 1 Gbit NAND | WSON-8-1EP 8x6mm P1.27mm EP3.4x4.3 | W25N01GVZEIG | [C88868](https://jlcpcb.com/partdetail/C88868) | Extended* | 6,508 | 5 | $9.60 | Ready | 3.3 V part (2.7 to 3.6 V), not W25N01GW |
| Y1 | 1 | 25 MHz 10 pF | Crystal SMD Abracon ABM8G-4Pin 3.2 | ABM8-25.000MHZ-10-D1G-T | None found | Not in library | None | 6 | $0.71 est. | Global Sourcing | Not in JLCPCB library. C124313 is an unverified candidate |
| C1,C2 | 2 | 13 pF C0G 50 V | 0402 | generic | [C48936](https://jlcpcb.com/partdetail/C48936) | Extended* | 46,767 | 20 | $0.0023 | Ready | Proposal |
| C3-C20,C26,C28,C30,C31,C42 | 23 | 100 nF X7R 16 V | 0402 | generic | [C1525](https://jlcpcb.com/partdetail/C1525) | Basic | 33,029,621 | 130 | $0.0025 | Ready | Proposal |
| C21,C22 | 2 | 4.7 uF X7R 10 V | 0603 | generic | [C18185770](https://jlcpcb.com/partdetail/C18185770) | Extended* | 270,979 | 20 | $0.0174 | Ready | Proposal |
| C23,C24 | 2 | 2.2 uF X7R 10 V | 0603 | generic | [C268026](https://jlcpcb.com/partdetail/C268026) | Extended | 3,895 | 20 | $0.0207 | Ready | Proposal; VCAP ESR under 100 mOhm not stated by listing |
| C25,C27,C29 | 3 | 1 uF X7R 10 V | 0402 | generic | [C528974](https://jlcpcb.com/partdetail/C528974) | Extended* | 148,318 | 23 | $0.0675 | Ready | Proposal |
| C32,C33 | 2 | 10 uF X7S 25 V | 0805 | C2012X7S1E106K125AC | [C109040](https://jlcpcb.com/partdetail/C109040) | Extended* | 23,235 | 12 | $0.1995 | Needs substitute | Exact C2171872 has stock 0. Murata GRM21BC71E106KE11L |
| C34 | 1 | 2.2 nF X7R 25 V | 0402 | GRM155R71E222KA01D | [C77022](https://jlcpcb.com/partdetail/C77022) | Extended* | 55,835 | 20 | $0.0068 | Needs substitute | Same family, 50 V: GRM155R71H222KA01D |
| C35-C40 | 6 | 22 uF X7S 10 V | 0805 | C2012X7S1A226M125AC | [C342717](https://jlcpcb.com/partdetail/C342717) | Extended* | 5,469 | 32 | $0.3024 | Ready | TDK ordering alias C2012X7S1A226MT000E |
| C41 | 1 | 470 nF X7R 16 V | 0402 | generic | [C471404](https://jlcpcb.com/partdetail/C471404) | Extended* | 107,268 | 20 | $0.0243 | Ready | Proposal |
| FB1 | 1 | 8.5 R at 100 MHz | 0603 | BLE18PS080SN1D | [C3764407](https://jlcpcb.com/partdetail/C3764407) | Extended | 15,494 | 9 | $0.1496 | Ready |  |
| FB2 | 1 | 120 R at 100 MHz | MPZ1005S121HT000 | MPZ1005S121HT000 | [C275478](https://jlcpcb.com/partdetail/C275478) | Extended | 43 | 20 | $0.0243 | Ready | Low stock |
| L1 | 1 | 2.2 uH | L Coilcraft XxL4020 | XGL4020-222MEC | [C6012410](https://jlcpcb.com/partdetail/C6012410) | Extended* | 592 | 5 | $6.12 | Ready |  |
| MB1,MB2 | 2 | 2x30 header | Samtec TFM-130-02-L-D-A MB1 | TFM-130-02-L-D-A | Samtec | User-supplied | See source list | 10 | $7.20 est. | Hand-solder | Keep exact Samtec part |
| R1 | 1 | 0 R | 0402 | generic | [C17168](https://jlcpcb.com/partdetail/C17168) | Basic | 13,866,727 | 20 | $0.0012 | Ready | Proposal |
| R2-R11,R14,R15 | 12 | 10 k 1% | 0402 | generic | [C25744](https://jlcpcb.com/partdetail/C25744) | Basic | 34,457,638 | 75 | $0.001 | Ready | Proposal |
| R12 | 1 | 10.7 k 1% | 0402 | generic | [C138068](https://jlcpcb.com/partdetail/C138068) | Extended* | 2,723 | 20 | $0.0021 | Ready | Proposal |
| R13 | 1 | 3.40 k 1% | 0402 | generic | [C4940](https://jlcpcb.com/partdetail/C4940) | Extended* | 206,307 | 20 | $0.0012 | Ready | Proposal |

## Sensor board

| Refs | Qty/set | Value | Footprint | BOM MPN | JLCPCB part | Class | Stock | Buy for 5 sets | Unit USD | Status | Note |
|---|---:|---|---|---|---|---|---:|---:|---:|---|---|
| C1-C4,C6,C8,C10,C13-C22,C25,C29 | 19 | 100 nF | 0402 | generic | [C1525](https://jlcpcb.com/partdetail/C1525) | Basic | 33,029,621 | 110 | $0.0025 | Ready | Proposal, X7R 16 V |
| C5,C23,C24 | 3 | 1 uF | 0402 | generic | [C52923](https://jlcpcb.com/partdetail/C52923) | Basic | 9,101,050 | 27 | $0.0055 | Ready | Proposal, X5R 25 V (schematic names no dielectric) |
| C7 | 1 | 22 uF 6.3 V X5R | 0805 | generic | [C45783](https://jlcpcb.com/partdetail/C45783) | Basic | 4,348,709 | 9 | $0.1084 | Ready | Proposal, 25 V |
| C9,C11,C12,C30 | 4 | 10 uF 10 V X5R | 0805 | generic | [C15850](https://jlcpcb.com/partdetail/C15850) | Basic | 9,195,511 | 30 | $0.0311 | Ready | Proposal, 25 V |
| C26,C27 | 2 | 15 pF C0G | 0402 | generic | [C1548](https://jlcpcb.com/partdetail/C1548) | Basic | 1,266,136 | 20 | $0.0026 | Ready | Proposal, 50 V |
| C28 | 1 | 100 uF 10 V polymer | CP EIA-7343-15 Kemet-W | T520W107M010ATE040 | None found | Not in library | None | 6 | $3.47 est. | Global Sourcing | No stocked part keeps ESR, case, and model |
| D1 | 1 | Green LED | LED 0603 1608Metric | generic | [C12624](https://jlcpcb.com/partdetail/C12624) | Extended | 200,855 | 21 | $0.0119 | Ready | Proposal |
| D2 | 1 | Blue LED | LED 0603 1608Metric | generic | [C2288](https://jlcpcb.com/partdetail/C2288) | Extended | 47,046 | 20 | $0.0106 | Ready | Proposal |
| D3 | 1 | Red LED | LED 0603 1608Metric | generic | [C2286](https://jlcpcb.com/partdetail/C2286) | Basic | 8,349,146 | 20 | $0.0077 | Ready | Proposal |
| D4 | 1 | Yellow LED | LED 0603 1608Metric | generic | [C2287](https://jlcpcb.com/partdetail/C2287) | Extended | 29,438 | 20 | $0.0106 | Ready | Proposal |
| D5 | 1 | Amber LED | LED 0603 1608Metric | generic | [C165983](https://jlcpcb.com/partdetail/C165983) | Extended | 20,991 | 20 | $0.0233 | Ready | Proposal |
| D6 | 1 | White LED | LED 0603 1608Metric | generic | [C2290](https://jlcpcb.com/partdetail/C2290) | Basic | 2,356,983 | 20 | $0.0123 | Ready | Proposal |
| J_GPS,J_TELEM | 2 | JST-GH 6P | JST GH SM06B-GHS-TB 1x06-1MP P1.25 | SM06B-GHS-TB | LCSC C133065 | User-supplied | See source list | 10 | $0.3062 est. | Hand-solder | User-supplied |
| J_RC | 1 | JST-GH 4P | JST GH SM04B-GHS-TB 1x04-1MP P1.25 | SM04B-GHS-TB | LCSC C189895 | User-supplied | See source list | 5 | $0.451 est. | Hand-solder | User-supplied |
| J_USB | 1 | USB-C | USB C Receptacle GCT USB4105-xx-A  | USB4105-GF-A | [C3020560](https://jlcpcb.com/partdetail/C3020560) | Extended | 5,179 | 5 | $1.02 | Ready | Stays JLCPCB-assembled |
| MB1,MB2 | 2 | 2x30 socket | Samtec SFM-130-02-L-D-A MB1 | SFM-130-02-L-D-A | Samtec | User-supplied | See source list | 10 | $8.46 est. | Hand-solder | Keep exact Samtec part |
| PB1 | 1 | 2x15 socket | Samtec SFM-115-02-L-D-A PB1 | SFM-115-02-L-D-A | Samtec | User-supplied | See source list | 5 | $4.96 est. | Hand-solder | Keep exact Samtec part |
| Q1-Q4 | 4 | N-MOSFET | SOT-23 | BSS138 | [C52895](https://jlcpcb.com/partdetail/C52895) | Extended | 84,387 | 24 | $0.0711 | Ready | onsemi |
| R1,R3,R4,R10,R20,R22,R27,R29,R33 | 9 | 10 k | 0402 | generic | [C25744](https://jlcpcb.com/partdetail/C25744) | Basic | 34,457,638 | 60 | $0.001 | Ready | Proposal, 1% |
| R2,R5,R19,R28,R30-R32 | 7 | 100 k | 0402 | generic | [C25741](https://jlcpcb.com/partdetail/C25741) | Basic | 11,892,549 | 47 | $0.0009 | Ready | Proposal, 1% |
| R6-R9,R34,R36-R38 | 8 | 2.2 k | 0402 | generic | [C25879](https://jlcpcb.com/partdetail/C25879) | Basic | 2,725,828 | 50 | $0.0011 | Ready | Proposal, 1% |
| R11,R12 | 2 | 5.1 k | 0402 | generic | [C25905](https://jlcpcb.com/partdetail/C25905) | Basic | 3,980,976 | 25 | $0.0014 | Ready | Proposal, 1% |
| R13,R14 | 2 | 0 R | 0402 | generic | [C17168](https://jlcpcb.com/partdetail/C17168) | Basic | 13,866,727 | 20 | $0.0012 | Ready | Proposal |
| R17 | 1 | 30.1 k 1% | 0402 | generic | [C29402](https://jlcpcb.com/partdetail/C29402) | Extended* | 198,773 | 20 | $0.0012 | Ready | Proposal |
| R18 | 1 | 10 k 1% | 0402 | generic | [C25744](https://jlcpcb.com/partdetail/C25744) | Basic | 34,457,638 | 20 | $0.001 | Ready | Proposal |
| R21 | 1 | 15 k | 0402 | generic | [C25756](https://jlcpcb.com/partdetail/C25756) | Basic | 1,941,869 | 20 | $0.0011 | Ready | Proposal, 1% |
| R23,R24 | 2 | 1 k | 0402 | generic | [C11702](https://jlcpcb.com/partdetail/C11702) | Basic | 29,161,658 | 20 | $0.0011 | Ready | Proposal, 1% |
| R25,R26 | 2 | 27 R | 0402 | generic | [C25100](https://jlcpcb.com/partdetail/C25100) | Extended* | 597,920 | 22 | $0.0012 | Ready | Proposal, 1% |
| R35 | 1 | 680 R | 0402 | generic | [C25130](https://jlcpcb.com/partdetail/C25130) | Extended* | 372,401 | 20 | $0.0012 | Ready | Proposal, 1% |
| R39 | 1 | 1.5 k | 0402 | generic | [C25867](https://jlcpcb.com/partdetail/C25867) | Basic | 1,455,723 | 20 | $0.0029 | Ready | Proposal, 1% |
| R40 | 1 | 43.2 k 1% | 0402 | generic | [C2097969](https://jlcpcb.com/partdetail/C2097969) | Extended | 10,202 | 20 | $0.0061 | Ready | Proposal |
| SW1-SW3 | 3 | Tactile switch | SW Push 1P1T NO CK KMR2 | KMR221GLFS | [C72443](https://jlcpcb.com/partdetail/C72443) | Extended | 445 | 15 | $0.5679 | Ready |  |
| U1 | 1 | IMU | BMI323 BOS | BMI323 | [C5368700](https://jlcpcb.com/partdetail/C5368700) | Extended | 2,632 | 5 | $3.25 | Ready | Standard PCBA only |
| U2 | 1 | Barometer | BMP390-10LGA 2X2X0p75 BOS | BMP390 | [C5124834](https://jlcpcb.com/partdetail/C5124834) | Extended | 0 | 6 | $5.39 est. | Global Sourcing (blocker) | JLCPCB stock 0; DigiKey and Mouser snapshots out of stock |
| U3 | 1 | Magnetometer | MMC5983MA MEM | MMC5983MA | [C404329](https://jlcpcb.com/partdetail/C404329) | Extended | 3,810 | 5 | $1.99 | Ready | Standard PCBA only |
| U4 | 1 | Power mux | SOT-583-8 | TPS2116DRLR | [C3235557](https://jlcpcb.com/partdetail/C3235557) | Extended | 58,432 | 5 | $0.3764 | Ready |  |
| U5 | 1 | USB ESD | SOT-23-6 | USBLC6-2SC6 | [C7519](https://jlcpcb.com/partdetail/C7519) | Extended* | 39,550 | 7 | $0.1544 | Ready | ST part |
| U6 | 1 | Debug MCU | QFN-56-1EP 7x7mm P0.4mm EP3.2x3.2m | SC0914(13) | [C2040](https://jlcpcb.com/partdetail/C2040) | Extended | 61,740 | 5 | $1.09 | Ready | Listed as RP2040 |
| U7 | 1 | 16 Mbit NOR | SOIC-8 5.3x5.3mm P1.27mm | W25Q16JVSSIQ | [C131025](https://jlcpcb.com/partdetail/C131025) | Extended* | 8,997 | 5 | $1.23 | Ready |  |
| U8 | 1 | Current-limit switch | SOT-23-6 | TPS2553DBVR | [C55266](https://jlcpcb.com/partdetail/C55266) | Extended | 43,502 | 5 | $0.5499 | Ready |  |
| Y1 | 1 | 12 MHz | Crystal SMD 3225-4Pin 3.2x2.5mm | ABM8-272-T3 | [C20625731](https://jlcpcb.com/partdetail/C20625731) | Extended* | 11,930 | 5 | $0.3962 | Ready |  |

## Power board

| Refs | Qty/set | Value | Footprint | BOM MPN | JLCPCB part | Class | Stock | Buy for 5 sets | Unit USD | Status | Note |
|---|---:|---|---|---|---|---|---:|---:|---:|---|---|
| C1,C17,C19,C21,C23 | 5 | 100 nF X7R 16 V | 0402 | GRM155R71C104KA88D | [C71629](https://jlcpcb.com/partdetail/C71629) | Extended* | 1,166,566 | 37 | $0.0032 | Ready |  |
| C2 | 1 | 100 uF 50 V hybrid | CP Elec 10x10.2 Panasonic G | EEH-ZA1H101P | [C109086](https://jlcpcb.com/partdetail/C109086) | Extended | 2,456 | 5 | $0.8248 | Ready | 28 mOhm |
| C3 | 1 | 220 uF 50 V | CP Elec 10x10.2 Panasonic G | EEE-FK1H221P | [C343913](https://jlcpcb.com/partdetail/C343913) | Extended | 5,749 | 5 | $0.5319 | Needs substitute | Exact C178549 has stock 0. EEEFK1H221GP has the same catalog ratings |
| C4,C8,C12 | 3 | 10 uF 50 V X7R | 1210 | C3225X7R1H106K250AC | [C138687](https://jlcpcb.com/partdetail/C138687) | Extended* | 81,287 | 15 | $0.2386 | Needs substitute | Exact C2181731 has stock 0. Samsung CL32B106KBJNNNE |
| C5,C9,C13 | 3 | 10 nF | 0805 | CL21B103KBANNNC | [C1710](https://jlcpcb.com/partdetail/C1710) | Basic | 1,969,317 | 25 | $0.0078 | Ready |  |
| C6,C7,C10,C11,C14,C15 | 6 | 220 uF 10 V | 10TPE220ML | 10TPE220ML | [C147767](https://jlcpcb.com/partdetail/C147767) | Extended* | 1,461 | 30 | $0.9409 | Ready |  |
| C16,C18,C20,C22 | 4 | 10 uF 16 V X7R | 1206 | CL31B106KOHNNNE | [C47971](https://jlcpcb.com/partdetail/C47971) | Extended* | 127,921 | 25 | $0.2342 | Ready |  |
| D1 | 1 | 26 V 5 kW TVS | D SMC | 5.0SMDJ26A | None found | Not in library | None | 6 | $2.65 est. | Global Sourcing | Littelfuse not stocked. BORN C910143 is an unverified candidate |
| D2-D4 | 3 | 60 V 5 A Schottky | D SMC | B560C-13-F | [C85100](https://jlcpcb.com/partdetail/C85100) | Extended | 36,390 | 16 | $0.1928 | Ready |  |
| ESC1,ESC2 | 2 | JST-GH 2P | JST GH SM02B-GHS-TB 1x02-1MP P1.25 | SM02B-GHS-TB | LCSC C189893 | User-supplied | See source list | 10 | $0.4317 est. | Hand-solder | User-supplied |
| J1 | 1 | Battery connector | XT60PW-M | XT60PW-M | LCSC C98732 | User-supplied | See source list | 5 | $0.5809 est. | Hand-solder | User-supplied; JLCPCB lists it for manual soldering |
| L1-L3 | 3 | 15 uH | L Coilcraft XAL7070-XXX | XAL7070-153MEC | [C2045384](https://jlcpcb.com/partdetail/C2045384) | Extended | 1,220 | 15 | $0.738 | Needs substitute | Bourns SRP7050TA-150M, needs footprint and model changes |
| PB1 | 1 | 2x15 header | TFM-115-32-L-D-A | TFM-115-32-L-D-A | Samtec | User-supplied | See source list | 5 | $4.72 est. | Hand-solder | Keep exact Samtec part |
| R1 | 1 | 0.5 mOhm 1% 5 W | R Shunt Isabellenhuette BVR4026 | BVR-Z-R0005-1.0 | None found | Not in library | None | 6 | $4.95 est. | Global Sourcing | No stocked 4-terminal 4026 equivalent |
| R2,R4,R6 | 3 | 10 k | 0805 | 0805W8F1002T5E | [C17414](https://jlcpcb.com/partdetail/C17414) | Basic | 45,106,557 | 25 | $0.0032 | Ready |  |
| R3,R5,R7 | 3 | 3.24 k | 0805 | 0805W8F3241T5E | [C17609](https://jlcpcb.com/partdetail/C17609) | Extended* | 65,308 | 25 | $0.0045 | Ready |  |
| R8-R13 | 6 | 270 R | 0603 | RC0603FR-07270RL | [C137754](https://jlcpcb.com/partdetail/C137754) | Extended* | 2,038,019 | 40 | $0.002 | Ready |  |
| R14,R15 | 2 | 10 k | 0603 | RC0603FR-0710KL | [C98220](https://jlcpcb.com/partdetail/C98220) | Extended* | 9,558,723 | 25 | $0.0023 | Ready |  |
| S1-S4 | 4 | Servo header | PinHeader 1x03 P2.54mm Horizontal | TSW-103-08-G-S-RA | Samtec | User-supplied | See source list | 20 | $0.65 est. | Hand-solder | User-supplied through-hole |
| U1 | 1 | Power monitor | TSSOP-10 3x3mm P0.5mm | INA228AIDGSR | [C2887910](https://jlcpcb.com/partdetail/C2887910) | Extended | 0 | 6 | $6.79 est. | Global Sourcing (blocker) | JLCPCB stock 0; DigiKey restock listed for 2027; Mouser snapshots conflict |
| U2-U4 | 3 | 5 V buck | TPS5430DDAR | TPS5430DDAR | [C9864](https://jlcpcb.com/partdetail/C9864) | Basic | 305,773 | 15 | $0.4991 | Ready |  |

## Parts needing action

### Supply blockers

| Part | Board refs | Finding | Action |
|---|---|---|---|
| `STM32H747BIT6` | MCU `U1` | JLCPCB [C730197](https://jlcpcb.com/partdetail/C730197) stock 0. DigiKey snapshot shows 0 stock and 30-week lead time; ST eStore shows sold out; Octopart reports EOL, which could not be confirmed with ST. | Confirm ST lifecycle and find a purchasable source before layout. Do not swap the MCU without a pin and firmware review. |
| `BMP390` | Sensor `U2` | JLCPCB [C5124834](https://jlcpcb.com/partdetail/C5124834) stock 0. DigiKey and Mouser main listings show 0 stock in snapshots. | Check Global Sourcing after sign-in. `BMP390L` and `BMP388` were not accepted as substitutes. |
| `INA228AIDGSR` | Power `U1` | JLCPCB [C2887910](https://jlcpcb.com/partdetail/C2887910) stock 0. DigiKey lists 2027 restocks. Mouser snapshots conflict. The Q1 listing [C5214669](https://jlcpcb.com/partdetail/C5214669) has stock 1. | Confirm Global Sourcing quantity. |

### Other actions

- **Global Sourcing, exact part:** MCU `Y1`, sensor `C28`, power `D1`, and power `R1`. Public snapshots show the thinnest supply on `D1` (DigiKey 6, an older Mouser snapshot 2,566) and a 14-week DigiKey lead time on `Y1`.
- **Low JLCPCB stock:** `FB2` has 43 available and the estimated purchase is 20. Lock it early.
- **Annotation:** KiCad warns about sensor schematic annotation. `J_GPS`, `J_RC`, `J_TELEM`, and `J_USB` export as `J_GPS?`-style references and must have final designators before BOM and CPL generation.
- **LCSC fields:** most symbols have no LCSC field. Add one when the substitutions are decided.
- **VCAP capacitors:** confirm the chosen `C23`, `C24` meets the under-100 mOhm ESR limit from the manufacturer's curve.

## Proposed substitutes

None of these has been applied.

| Refs | Original | Proposed stocked part | Evidence | Required design change |
|---|---|---|---|---|
| MCU `C32`, `C33` | `C2012X7S1E106K125AC` | [C109040](https://jlcpcb.com/partdetail/C109040) Murata `GRM21BC71E106KE11L` | [Murata datasheet](https://pim.murata.com/asset/pim4/ceramicCapacitorSMD/GRM21BC71E106KE11-01A-EN_PDF_CERAMICCAPACITORSMD?lastModifiedDatetime=20250813151733): 10 uF, 25 V, X7S, 0805 | Change MPN and LCSC fields only |
| MCU `C34` | `GRM155R71E222KA01D` | [C77022](https://jlcpcb.com/partdetail/C77022) Murata `GRM155R71H222KA01D` | [Murata product page](https://www.murata.com/products/productdetail?partno=GRM155R71H222KA01%23): 2.2 nF, X7R, 10%, 0402, higher 50 V rating | Change MPN and LCSC fields only |
| Power `C3` | `EEE-FK1H221P` | [C343913](https://jlcpcb.com/partdetail/C343913) Panasonic `EEEFK1H221GP` | [Panasonic catalog](https://industrial.panasonic.com/cdbs/www-data/pdf/RDE0000/RDE0000COL99.pdf): both 220 uF, 50 V, 10 x 10.2 mm, 10.5 mm height, 670 mA, 0.18 ohm | Change MPN and LCSC fields only |
| Power `C4`, `C8`, `C12` | `C3225X7R1H106K250AC` | [C138687](https://jlcpcb.com/partdetail/C138687) Samsung `CL32B106KBJNNNE` | [Samsung data](https://weblib.samsungsem.com/mlcc/mlcc-ec-data-sheet.do?partNumber=CL32B106KBJNNN): 10 uF, 50 V, X7R, 1210, 2.5 +/-0.2 mm | Change MPN and LCSC fields; check DC-bias capacitance in the surge model |
| Power `L1` to `L3` | `XAL7070-153MEC` | [C2045384](https://jlcpcb.com/partdetail/C2045384) Bourns `SRP7050TA-150M` | [Bourns datasheet](https://bourns.com/docs/Product-Datasheets/srp7050ta.pdf): 15 uH, 81/92 mOhm, 3.5 A Irms, 6 A Isat at 20% drop; [DigiKey](https://www.digikey.com/en/products/detail/bourns-inc/SRP7050TA-150M/5429680): 5.0 mm height | **Replace the footprint**, rerun TPS5430 stability and thermal checks |

The Bourns inductor meets the three stated limits but has about three times the DCR of the XAL7070. At 3 A it dissipates up to 0.83 W versus 0.27 W per inductor. Keeping the exact XAL7070 through Global Sourcing avoids the footprint and model changes and adds about $40.93 for 5 sets, assuming Coilcraft's [$3.25 single-piece price](https://www.coilcraft.com/en-us/products/power/shielded-inductors/molded-inductor/xal/xal7070/xal7070-153/) and one spare.

### Rejected or unverified options

| Candidate | For | Reason |
|---|---|---|
| [C124313](https://jlcpcb.com/partdetail/C124313) TAITIEN 25 MHz | MCU `Y1` | Matches listed frequency, 10 pF, and temperature, but the manufacturer datasheet, ESR, and pad functions could not be read |
| [C910143](https://jlcpcb.com/partdetail/C910143) BORN `5.0SMDJ26A` | Power `D1` | JLCPCB listing matches, but manufacturer data could not be read. Keep Littelfuse until verified |
| [C1975722](https://jlcpcb.com/partdetail/C1975722) `5.0SMDJ26CA` | Power `D1` | Bidirectional, and stock 0 |
| [C19191628](https://jlcpcb.com/partdetail/C19191628) `XAL6060-153MEC` | Power `L1` to `L3` | Isat 5.8 A is below the 6 A minimum |
| [C2688874](https://jlcpcb.com/partdetail/C2688874) `BVN-Z-R0005-1.0` | Power `R1` | Different 1216 case and footprint, not a BVR 4026 replacement |
| [C147766](https://jlcpcb.com/partdetail/C147766) `10TPC100M` | Sensor `C28` | 45 mOhm exceeds the original 40 mOhm ESR |
| [C135491](https://jlcpcb.com/partdetail/C135491) 100 uF polymer | Sensor `C28` | Different ESR would require rerunning the 5 V stability model, and its manufacturer data was not verified |
| [C730186](https://jlcpcb.com/partdetail/C730186) `STM32H745BIT6` | MCU `U1` | Different MCU with 5 in stock. Pin compatibility was not verified, so it is not proposed |
| `W25N01GW` | MCU `U3` | 1.8 V part, prohibited |

## User-supplied, hand-soldered parts

| Part | Qty for 5 sets | Normal source | Unit USD used | Availability seen |
|---|---:|---|---:|---|
| `TFM-130-02-L-D-A` | 10 | [Samtec](https://www.samtec.com/products/tfm-130-02-l-d-a) | $7.20 | Samtec page showed no ready stock. JLCPCB also lists [C17202968](https://jlcpcb.com/partdetail/C17202968): 12 at $12.95 |
| `SFM-130-02-L-D-A` | 10 | [Samtec](https://www.samtec.com/products/sfm-130-02-l-d-a) | $8.46 | 534 ships tomorrow |
| `TFM-115-32-L-D-A` | 5 | [Samtec](https://www.samtec.com/products/tfm-115-32-l-d-a) | $4.72 | Samtec page showed no ready stock. JLCPCB also lists [C17413469](https://jlcpcb.com/partdetail/C17413469): 21 available |
| `SFM-115-02-L-D-A` | 5 | [Samtec](https://www.samtec.com/products/sfm-115-02-l-d-a) | $4.96 | 52 ships tomorrow |
| `TSW-103-08-G-S-RA` | 20 | [Samtec](https://www.samtec.com/products/tsw-103-08-g-s-ra) | $0.65 | 2,090 ships tomorrow |
| `XT60PW-M` | 5 | [LCSC C98732](https://www.lcsc.com/product-detail/C98732.html) | $0.58 | 138,596 |
| `SM06B-GHS-TB` | 10 | [LCSC C133065](https://www.lcsc.com/product-detail/C133065.html) | $0.31 | 17,782 |
| `SM04B-GHS-TB` | 5 | [LCSC C189895](https://www.lcsc.com/product-detail/C189895.html) | $0.45 | 17,304 |
| `SM02B-GHS-TB` | 10 | [LCSC C189893](https://www.lcsc.com/product-detail/C189893.html) | $0.43 | 705 |
| Battery and ESC wire | As needed | Normal hobby or electronics supplier | Not costed | Not checked |

### Hand-soldering risk note

- **Pitch:** Samtec TFM/SFM contacts are 1.27 mm pitch. The project footprints use 0.74 mm pads, leaving 0.53 mm gaps. JST-GH is 1.25 mm pitch.
- **Alignment pins:** the `-A` footprints use 1.42 mm non-plated holes at +/-20.005 mm for 2x30 parts and +/-10.48 mm for 2x15 parts. Keep them solder-free. Check the hole size against Samtec's recommended footprint before layout, because the extracted print did not give a usable hole dimension.
- **Stack height:** solder thickness adds directly to mated height. Samtec lists 0.15 mm coplanarity for 27 to 50 positions and 0.10 mm for up to 26. The `10TPE220ML` under the MCU board has 0.68 mm of budgeted clearance, so measure the first stack.
- **Assembly order:** after JLCPCB assembly, solder `MB1`, `MB2`, and `PB1` headers with a fine iron or tightly local hot air. Tack the sensor-board sockets, dry-fit the full stack on the specified 6.5 mm and 12 mm standoffs, then finish them. Solder JST-GH next, and the XT60, servo headers, and wires last.
- **Do not reflow populated boards** on a hot plate or in an oven. It can disturb JLCPCB-placed parts and reheat the LGA sensors.

### Sensor board side check

**Intended design: yes, JLCPCB assembly can become single-sided on the sensor board.** The schematic marks only `MB1`, `MB2`, and `PB1` as underside sockets, and those are hand-soldered in this plan. This is **not yet confirmed physically** because no sensor layout exists. Confirm that every other part is on the top side when the `.kicad_pcb` is created.

## JLCPCB process constraints

| Topic | Current public rule | Effect on V3 |
|---|---|---|
| Layers | Economic: 2, 4, 6. Standard: 1 to 32 | 4-layer MCU and power, 2-layer sensor are supported |
| Assembly tier | STM32H747, BMI323, BMP390, and MMC5983 are listed **Standard Only** | MCU and sensor boards must use Standard PCBA |
| Power board tier | Economic is single-sided with a fixed 255 +/-5 C reflow; Standard is 240 +/-5 C | Stocked power parts list both tiers. Check the electrolytic and POSCAP reflow limits before choosing Economic |
| Minimum size | Standard: 70 x 70 mm, smaller boards need rails or a panel | The roughly 46 x 50 mm MCU board needs rails or panelization. Sensor and power outlines are not defined yet |
| Double-sided | Standard only | Not needed on the sensor board if the sockets stay hand-soldered |
| Fine pitch | Standard minimum IC pitch 0.35 mm; Economic 0.40 mm | 0.5 mm LQFP208 and 0.4 mm RP2040 are within limits |
| LGA and QFN | Supported, with X-ray for applicable packages | No design change found |
| Alignment-pin connectors | No explicit public rule found | Avoided because Samtec parts are hand-soldered. For future machine placement, ask JLCPCB and use a Samtec `-P` or `-TR` option |
| Through-hole | Supported. Manual parts: $3.50 plus $0.0173 per joint | JLCPCB could solder the XT60 as a manual part. `TSW-103-08-G-S-RA` was not found in its library. Current plan remains hand-solder |
| BOM | CSV, XLS, or XLSX with Comment, Designator, Footprint, LCSC Part # | Requires an LCSC field |
| CPL | Designator, Mid X, Mid Y, Rotation, Layer in mm | KiCad `.pos` headers need renaming, or use the Fabrication Toolkit plugin. Check rotations in the JLCPCB preview |
| Global Sourcing | Review 1 to 3 business days, purchasing typically 9 to 20. Parts must reach the JLCPCB warehouse. Orders cannot be cancelled or returned after payment | Allow time before assembly and buy only after final design |

**Process issues that change the design or release flow:** create the layouts, add rails or panelization for small Standard PCBA boards, finalize sensor designators, and replace the inductor footprint only if the Bourns substitute is accepted.

Sources: [Assembly capabilities](https://jlcpcb.com/capabilities/pcb-assembly-capabilities), [Assembly FAQ](https://jlcpcb.com/help/article/PCB-assembly-faqs), [Global Sourcing](https://jlcpcb.com/help/article/How-to-use-JLCPCB-Global-Sourcing-Parts-Service), [KiCad BOM and CPL guide](https://jlcpcb.com/help/article/how-to-generate-the-bom-and-centroid-file-from-kicad), [Samtec TFM print](https://suddendocs.samtec.com/prints/tfm-1xx-xx-xxx-d-xxx-mkt.pdf), [Samtec SFM print](https://suddendocs.samtec.com/prints/sfm-1xx-xx-xxx-d-xxx-mkt.pdf).

## JLCPCB assembly path

### Proposed part swaps

1. MCU `C32`, `C33`: use `GRM21BC71E106KE11L` ([C109040](https://jlcpcb.com/partdetail/C109040)).
2. MCU `C34`: use `GRM155R71H222KA01D` ([C77022](https://jlcpcb.com/partdetail/C77022)).
3. Power `C3`: use `EEEFK1H221GP` ([C343913](https://jlcpcb.com/partdetail/C343913)).
4. Power `C4`, `C8`, `C12`: use `CL32B106KBJNNNE` ([C138687](https://jlcpcb.com/partdetail/C138687)).
5. Power `L1` to `L3`: use `SRP7050TA-150M` ([C2045384](https://jlcpcb.com/partdetail/C2045384)) only after the footprint, stability, and thermal changes. Otherwise buy the exact XAL7070 through Global Sourcing.

Use Global Sourcing for the exact `ABM8-25.000MHZ-10-D1G-T`, `T520W107M010ATE040`, `5.0SMDJ26A`, and `BVR-Z-R0005-1.0`.

### Remaining blockers

- No `.kicad_pcb`, so no Gerber or CPL package can be created.
- `STM32H747BIT6`, `BMP390`, and `INA228AIDGSR` have no confirmed purchasable stock.

### Estimated 5-set parts cost

| Board | JLCPCB stocked parts | Global Sourcing estimate | JLCPCB-assembled subtotal | User-supplied subtotal | Parts total | Extended loading fee, worst case |
|---|---:|---:|---:|---:|---:|---:|
| MCU board | $105.51 | $148.98 | $254.49 | $72.00 | $326.49 | $51.00 |
| Sensor board | $65.24 | $53.13 | $118.37 | $114.76 | $233.13 | $63.00 |
| Power board | $66.73 | $86.32 | $153.04 | $43.80 | $196.85 | $42.00 |
| **Total** | **$237.48** | **$288.43** | **$525.91** | **$230.56** | **$756.46** | **$156.00** |

The JLCPCB-assembled subtotal is **$525.91** and the user-supplied subtotal is **$230.56**, for **$756.46** in parts.

These totals exclude PCB fabrication, stencils, setup, placement, solder joints, shipping, duties, Global Sourcing fees, and wire. The loading-fee column assumes $3 for every non-Basic JLCPCB-assembled line on each board order and is not included in parts cost. Stocked purchase quantities use each part page's listed minimum placement and attrition values. Global Sourcing lines use public single-piece prices and one spare, so actual minimum-order costs may be higher.
