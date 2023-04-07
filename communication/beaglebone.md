The Beaglebone should have UART ports available. Port 0 is activated by default, but is being used as the kernel debug log so a lot of information is dumped here into terminal. 

**Here is how to activate a UART port.**

https://forum.beagleboard.org/t/need-to-enable-7-uart-on-beaglebone-ai-64/33936/2

What is an overlay and dtsi file?

https://forum.beagleboard.org/t/enable-uart-1-on-bb-ai-64/32997/8

May have some important information.

**Interfacing with a UART port**

In order to interface with UART ports, we will be using the **serial** library from Python.

1. In order to use this library, first run this command in command line.

	**pip install pyserial**

2. There should be some scripts already written in the **communication** branch of our Github repository. Please read the code and available comments in order to familiarize yourself with this library.

https://pyserial.readthedocs.io/en/latest/pyserial.html

In order to read data, we may use a logic analyzer in order to connect jumper wires to the pins and read UART information from there.

Currently attempting to grant the Beaglebone Internet access.

**Setting the System Clock on the Beaglebone**

As the Beaglebone does not have a battery, it cannot keep track of the clock when turned off and defaults to whatever it was on last. In order to access the Internet and pass the HTTPS security protocols, we must set the system clock to be synced when we turn on the Beaglebone.

To check the time for correctness, run **date** in the terminal.

http://derekmolloy.ie/automatically-setting-the-beaglebone-black-time-using-ntp/

This guide should help

https://askubuntu.com/questions/1058593/how-to-sync-the-time-to-network-with-timedatectl-on-ubuntu-18-04

This one as well

**Testing**

Both the read and write scripts are written.

To test, two terminals can be opened. One terminal runs the read script and the other terminal runs the write script.
The result of the transmission can be printed onto the terminal. If this doesn't work, using a logic analyzer on the pins could be a valid strategy. 

**Pins**

https://docs.beagleboard.org/latest/boards/beaglebone/ai-64/ch07.html#p8-17-p8-19


**UART Pins**

In order to determine which pins are mapped to the symbolic link that can be found by running *tree /dev/bone*, we must access the dtsi file and configure the pins ourselves.

For whatever reason, the symbolic link for UART1 is actually connected to UART2's pins.

