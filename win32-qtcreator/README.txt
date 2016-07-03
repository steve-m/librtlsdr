
there is an outdated "How to compile new releases of librtlsdr (and tools) on Windows" at
https://www.reddit.com/r/RTLSDR/comments/uce3e/how_to_compile_new_releases_of_librtlsdr_and/
unfortunately the link to the CMakeLists.txt is broken!

so, i needed to find another solution ..


1) aquire and install Qt 5.5.x for Windows 32-bit (MinGW 4.9.2) from
https://www.qt.io/download-open-source/#section-2

2) aquire and install QtCreator 4.0.x
from same site as 1)
probably this step is not necessary and you can use the qtcreator IDE from 1)

3) aquire LibUSB 1.0.20 from
https://sourceforge.net/projects/libusb/files/
last tested: libusb-1.0.20.7z

and place the file at C:/src/_foreign/libusb-1.0.20

or replace LIBUSBBASE path in CMakeLists.txt

4) start qtcreator and open the (modified) CMakeLists.txt
configure compiler/environment and compile


the resulting executables have no other dependencies than libwinpthread-1.dll
from the MINGW system at C:\Qt\Qt5.5.1\Tools\mingw492_32\bin\
  or C:\Qt\Qt5.5.1\5.5\mingw492_32\bin

