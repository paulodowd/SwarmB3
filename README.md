# SwarmB3

Working on version 3... so I decided the tinkering hadn't finished yet :)

Currently work in progress.  
- PCB designed but not tested.
- Initial code drafted but not tested.


## Development Notes

**25/03/26:** Started the design of version 3 of the swarm board.  The plan is to use the Adafruit ItsyBitsy M4, providing a hardware UART interface for each IR reciever on the pcb. When drafting this PCB, I couldn't find a convenient symbol or footprint, so I've used two rows of pin headers and labels.  I also noticed that the symbol for the double row header for the M5Stack Core2 doesn't correspond with the physical footprint - but the enumeration is correct (just a bit confusing). 

**27/03/26:** Using the Adafruit SAMD51 board config, they have a wrapped class named sercom around the original defintions SERCOM (difference being lower and upper case).  I wanted to invert Tx (maybe Rx) and the Adafruit wrapper hasn't got this implemented yet.  These are useful links for low level structures and definitions for SERCOM (SAMD51 in general).
- https://github.com/ace22293/SAMD51/tree/master
- https://github.com/arduino/ArduinoCore-samd/tree/master/cores/arduino
- https://github.com/arduino/ArduinoModule-CMSIS-Atmel/tree/master/CMSIS-Atmel/CMSIS/Device/ATMEL/samd51/include/instance
- https://github.com/arduino/ArduinoModule-CMSIS-Atmel/tree/master/CMSIS-Atmel/CMSIS/Device/ATMEL/samd51/include

