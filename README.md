Sensors
=======

The Ecocar sensors network code. All C code running on the PIC Microprocessors.

Setting up MPLABX
=================

In order to develop for the PIC microprocessors and compile/upload the code you will need to grab Microchip's MPLABX IDE (based on Netbeans). You will also need a Microchip C compiler.

MPLABX IDE: http://www.microchip.com/pagehandler/en-us/family/mplabx/

C Compiler: 
- Windows: http://www.microchip.com/stellent/idcplg?IdcService=SS_GET_PAGE&nodeId=1406&dDocName=en010014
See the "MPLAB C for PIC18 v3.46 in LITE mode"

Configuring MPLABX Projects
===========================

MPLABX Projects need to be configured for the compiler you are using, the programmer you'll be using and the exact PIC processor you are building for.

Compiler: Free Microchip XC 8 compiler
Programmer: PicKit 3
PIC Processor: PIC18f2685
