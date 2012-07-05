# AVR HIDTemp

HIDTemp is implementation of USB temperature sensor device
with AVR ATtiny45/85 microcontroller based on vusb.

- [vusb] (http://www.obdev.at/products/vusb/index.html)

# Building and Flashing HIDTemp

## Requirements
- avr-gcc
- avrdude
- usbasp
- AVRstick [sparkfun](http://www.sparkfun.com/products/9147)

## Building
    $ cd hidtemp/firmware
    $ make 

## Flasing
    $ make flash
