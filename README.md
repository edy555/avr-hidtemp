# AVR HIDTemp

HIDTemp is implementation of USB temperature sensor device
with AVR ATtiny45/85 microcontroller based on V-USB.

- V-USB is a software-only implementation of a low-speed USB device for AVR microcontrollers. See http://www.obdev.at/products/vusb/index.html

# Building and Flashing HIDTemp

## Requirements
- avr-gcc
- avrdude
- usbasp
- AVRstick from [sparkfun](http://www.sparkfun.com/products/9147)

## Building
    $ cd hidtemp/firmware
    $ make 

## Flashing
    $ make flash
