This repo houses all code from my final project for my ME461: Computer Control of Mechanical Systems. The final project consisted of a skittle sorter which sorted skittles based upon intensity of reflected red/green/blue light. 

The sorter was driven by 2 separate servos and all control and sensing on the sorter was run through a TI MSP430 chip with peripheral circuitry soldered into a prototype board which was built up throughout the class.

The number of skittles sorted was determined by the roll of a die, where frames read from a camera by an Orange Pi Zero microcomputer performed simple image manipulation and blob detection and the number of pips on the die face were communicated to the MSP430 via I2C.
