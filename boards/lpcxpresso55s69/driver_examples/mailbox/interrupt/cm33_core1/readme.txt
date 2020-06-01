Overview
========
The mailbox_interrupt example shows how to use mailbox to exchange message.

In this example:
The primary core writes a value to the secondary core mailbox, it causes mailbox interrupt
on the secondary core side. The secondary core reads the value from mailbox, it increments and writes it to mailbox register
for the primary core, which causes mailbox interrupt on the primary core side. The primary core reads value from mailbox, 
it increments and writes it to the mailbox register for the secondary core again.

Toolchain supported
===================
- IAR embedded Workbench  8.40.2
- MCUXpresso  11.1.0
- GCC ARM Embedded  8.3.1
- Keil MDK  5.29

Hardware requirements
=====================
- Mini/micro USB cable
- LPCXpresso55s69 board
- Personal Computer

Board settings
==============
No special settings are required.

Prepare the Demo
================
1.  Connect a micro USB cable between the PC host and the CMSIS DAP USB port (P6) on the board
2.  Open a serial terminal with the following settings (See Appendix A in Getting started guide for description how to determine serial port number):
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Running the demo
================
The log below shows example output of the Mailbox interrupt example in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Mailbox interrupt example
Copy CORE1 image to address: 0x20033000, size: 1388
Write to the secondary core mailbox register: 1
Read value from the primary core mailbox register: 2
Write to the secondary core mailbox register: 3
Read value from the primary core mailbox register: 4
Write to the secondary core mailbox register: 5
Read value from the primary core mailbox register: 6
Write to the secondary core mailbox register: 7
.
.
.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Customization options
=====================

