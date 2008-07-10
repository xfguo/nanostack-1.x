Bootloader v0.1
--------------------------------------------------
by Rasmus Raag at Tallinn University of Technology
Tallinn, 2008

Contents:
1. What is this project?
2. Parts of the project:
	2.1 bootloader
	2.2 rSend
	2.3 nano_skeleton
3. Contact information


1. What is this project?
------------------------

This project enables wireless reprogramming of Texas Instruments' CC2430/31
SoCs (System on Chip), using Sensinode's NanoStack. In other words: after
putting the boot loader on your CC2430 chip, you never need to use the hardware
programmer anymore, as long as you use NanoStack.

This project is divided into three parts. First of all, the boot loader, which
resides in CC2430 program memory. Secondly, rSend, which sits on your PC and
communicates with the boot loader, sending programs and other stuff to be
written into device's flash memory. Finally, a modified version on NanoStack's
nano_skeleton, which shows non-elegantly how to write apps that have inside
them the required switch to turn on the boot loader.

It has been tested with:
SDCC 2.7
GCC 4.1
NanoStack v1.0.3


2. Parts of the project:
------------------------
	2.1 bootloader
	--------------

bootloader is the boot loader program inside the CC2430 program memory. It
takes up 2 pages of the memory and has the power to change and read program
memory, communicate over radio link and perform CRC16 (16-bit Cyclic Redundancy
Check) on the memory. Currently, it has to be triggered by application software
with a simple LJMP instruction. Once triggered, it halts all other processes
and starts waiting for incoming programs. It can be remotely instructed to
read or program CC2430's flash or reset the device.

Build with SDCC 2.7 (with banking patch)

	2.2 rSend
	---------

rSend is the PC side of the reprogramming software. It uses NanoStack's nRoute
daemon to communicate with the remote device. Its input is a Intel Hex file
produced by SDCC compiler. rSend sends the boot loader on the remote device the
program described in the ihex file. It can also read the entire contents of the
remote device's flash and put it in a ihex file, which can be then used as
input.

	2.3 nano_skeleton
	-----------------

nano_skeleton is a modified version of NanoStack's nano_skeleton. It includes a
callback function that starts the boot loader. If you do not include such
mechanism into your application, then next time the boot loader can not start
and you must use hardware programmer to reprogram your CC2430. Maybe in the
future, someone can get the bootloader to not need to be executed within user
application, but for now, this is the only way.


3. Contact information
----------------------

If anyone is interested in this project and needs more information about the
internals of it (e.g. the communication protocol), they can reach me on e-mail:

rasmus_raag@users.sourceforge.net

This e-mail address redirects all your mails to my personal e-mail address. I
can reply in Estonian or English, but really simple questions can also be asked
in German, Swedish or French.

My personal documentation of this project is in Estonian and is available if
requested. Any English questions will be answered in English based on the
documentation, but I will not try to translate the full Estonian text into
English, as it can become obsolete too quickly.

Also, I will not be developing this project any further. All feature requests
are ignored. Instead, this project is published on NanoStack's profile on
SourceForge.net, where everyone can download the source code and modify it.

