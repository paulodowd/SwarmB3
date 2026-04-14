# SwarmB3

Working on version 3... so I decided the tinkering hadn't finished yet :)

Currently work in progress.  
- PCB designed but not tested.
- Initial code drafted but not tested.


## Development Notes

**10/04/26:** The TSDP34156 is active low output when receiving an IR burst.  I had thought I would use the M4 SERCOM interface to invert the UART signal logic.  It seems like only the start,stop and data are inverted, but the idle signal remains high in either case. I spent some time trying to find a software solution to pull the tx line low, with no luck.  Even with no idle, or long bursts of transmission, I was getting gibberish on the receiving end regardless of whether I was inverting Tx or not.  With the assumption of inverting in software, the first pcb design had a 74hc08 AND gate to combine the 58khz clock and data.  Abandoning this, I first tried a NAND, but that actually just negates the effect of the clock - since a changing clock qualifies as not-and against data or no data.  Instead, an OR gate works. With Tx not inverted, if I draw out the truth table, then it means that for a data logic 0 a 58khz burst is created, and for data logic 1 a constant IR signal is made (not modulated, just on).     

**08/04/26:** Some pretty annoying problems with the first PCB prototype!  The 74hc08 ground was isolated.  The pots for current limiting were not in logical physical locations, not next to their LEDs.  I labelled the ItsyBitsy pin 7 wrong, so it wasn't conencted to the 74hc08.  Fixed all of that.  I've installed two different types of IR LEDs to compare the difference.  One is the Vishay TSAL4400 (https://www.vishay.com/en/product/81006/), and the other is an Onsemi QEE113 (https://www.onsemi.com/products/interfaces/infrared/emitting-diodes/qee113).  Both have similar characteristics, slightly different radiance patterns. I prefer the form factor of the Onsemi device.

**27/03/26:** Using the Adafruit SAMD51 board config, they have a wrapped class named sercom around the original defintions SERCOM (difference being lower and upper case).  I wanted to invert Tx (maybe Rx) and the Adafruit wrapper hasn't got this implemented yet.  These are useful links for low level structures and definitions for SERCOM (SAMD51 in general).
- https://github.com/ace22293/SAMD51/tree/master
- https://github.com/arduino/ArduinoCore-samd/tree/master/cores/arduino
- https://github.com/arduino/ArduinoModule-CMSIS-Atmel/tree/master/CMSIS-Atmel/CMSIS/Device/ATMEL/samd51/include/instance
- https://github.com/arduino/ArduinoModule-CMSIS-Atmel/tree/master/CMSIS-Atmel/CMSIS/Device/ATMEL/samd51/include

- **25/03/26:** Started the design of version 3 of the swarm board.  The plan is to use the Adafruit ItsyBitsy M4, providing a hardware UART interface for each IR reciever on the pcb. When drafting this PCB, I couldn't find a convenient symbol or footprint, so I've used two rows of pin headers and labels.  I also noticed that the symbol for the double row header for the M5Stack Core2 doesn't correspond with the physical footprint - but the enumeration is correct (just a bit confusing). 


