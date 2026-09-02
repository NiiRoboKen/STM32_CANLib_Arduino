cd ./.pio/build

BOARD=$(ls -d */)

#echo "${BOARD}"

/home/taisei/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer.sh -c port=SWD -w "${BOARD}/firmware.elf" -rst