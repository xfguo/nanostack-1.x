Ideasilo is an application platform for Nokia N770 for utilizing
information embedded on RFID tags. The program uses the USB port on 
N770 to connect to a wireless radio transmitter which communicates 
with a wireless RFID reader. When a tag is read, the program augments 
the data with data obtained from a remote Internet server. The user 
is then able to utilize the data with applications built into Ideasilo.

Currently implemented applications:
- Ordering application
- Data management application

Required hardware:
- Casing containing power source for the USB port and the radio 
  transmitter
- The wireless RFID reader developed in ISG-WILHO project
- Nokia N770

The software contained in this package:
- Ideasilo binaries for ARMEL architecture
- Ideasilo sources
- libglade2-0 for ARMEL architecture
- Installation instructions
- User manual

In order to compile the program, you need to set up Scratchbox and
install Maemo development platform there. Scratchbox and installation
instructions can be found from:

http://www.scratchbox.org/

Maemo API and many tutorials and instructions can be found from

http://maemo.org/

In order to compile, extract the contents of the source archive inside 
scratchbox environment. You must have the cross compilation target set 
up properly. Run the commands:

sh autogen.sh   (to create the Makefile)
make all        (to compile)
make deb        (to create a debian package of the software in 
                 /debian-build subdirectory)
fakeroot dpkg -i /debian-build/ideasilo_<version>_<architecture>.deb
                (to install the program inside maemo API)

The following commands may also prove useful:

./runtest       (to run the program. You must have maemo running 
                 properly in a X server window)
make clean      (to remove all object files. This is necessary when you 
                 change the compilation architecture)
make distclean	(to remove all generated files. After this you must run 
                 sh autogen.sh again to compile. This step is necessary 
                 when you add or remove files to/from the project)

Troubleshooting:
If it appears N770 isn't receiving anything from the RFID reader, and
the nRoute status indicator tells everything is ok, nRoute might not
have succeeded in contacting the radio transmitter properly. In this
case, restart Ideasilo until it works.				 
				 