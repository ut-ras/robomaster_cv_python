The Beaglebone should have UART ports available. Port 0 is activated by default, but is being used as the kernel debug log so a lot of information is dumped here into terminal. 

**Here is how to activate a UART port.**

https://forum.beagleboard.org/t/need-to-enable-7-uart-on-beaglebone-ai-64/33936/2

What is an overlay and dtsi file?

https://forum.beagleboard.org/t/enable-uart-1-on-bb-ai-64/32997/8

May have some important information.

**Interfacing with a UART port**

In order to interface with UART ports, we will be using the **serial** library from Python.

1. In order to use this library, first run this command in command line.

	**pip install serial**

2. There should be some scripts already written in the **communication** branch of our Github repository. Please read the code and available comments in order to familiarize yourself with this library.

In order to read data, we may use a logic analyzer in order to connect jumper wires to the pins and read UART information from there.

Currently attempting to grant the Beaglebone Internet access