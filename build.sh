#!/bin/bash
clear
rm a.out

OUTPUT="lab4.bin"
FILES="lab3.c startup_gcc.c linkedqueue.c pwm.c"
LINKERF="lab3.o startup_gcc.o linkedqueue.o pwm.o"
ROOTDIR="/home/dev/TI/StellarisWare"
LM4FLASH="lm4flash"

echo "Compiling..."
arm-none-eabi-gcc $FILES -g -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -Os -ffunction-sections -fdata-sections -MD -std=c99 -Wall -pedantic -DPART_LM4F120H5QR -c -I$ROOTDIR -DTARGET_IS_BLIZZARD_RA1

echo "Linking..."
arm-none-eabi-ld -T linker.ld --entry ResetISR -o a.out $LINKERF --gc-sections

echo "Copying object files..."
arm-none-eabi-objcopy -O binary a.out $OUTPUT

echo "Flashing to rom..."
#lm4flash timerled.bin
sudo $LM4FLASH $OUTPUT

echo "Build terminated."

#ta info: yuanye@uvic.ca
