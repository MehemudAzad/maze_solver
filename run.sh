#!/bin/bash
cd .pio
cd build
cd atmega32
rm -rf firmware.hex firmware.elf
cd ..
cd ..
cd ..
pio run
avrdude -c usbasp -p m32 -e  
pio run --target upload