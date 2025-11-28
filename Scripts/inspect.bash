#!/bin/bash

# Inspect the symbol tables of the built ELF files for both CM7 and CM4
mkdir -p ${SCRIPT_DIR}/build
${ARM_GNU_BIN_PATH}/${ARM_GNU_PREFIX}-objdump -t ${STM32_CM7_ELF_PATH} > ${SCRIPT_DIR}/build/cm7_inspect.txt
${ARM_GNU_BIN_PATH}/${ARM_GNU_PREFIX}-objdump -t ${STM32_CM4_ELF_PATH} > ${SCRIPT_DIR}/build/cm4_inspect.txt