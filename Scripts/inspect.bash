#!/bin/bash

# Inspect the symbol tables of the built ELF files for both CM7 and CM4
arm-none-eabi-objdump -t CM7/Debug/Drone_CM7.elf > Scripts/build/cm7_inspect.txt
arm-none-eabi-objdump -t CM4/Debug/Drone_CM4.elf > Scripts/build/cm4_inspect.txt