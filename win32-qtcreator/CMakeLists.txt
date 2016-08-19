project(rtlsdr)
cmake_minimum_required(VERSION 2.8)

# created and tested with
# Qt 5.5.1 for Windows 32-bit (MinGW 4.9.2) from
# https://www.qt.io/download-open-source/#section-2
#
# and QtCreator 4.0.0
#
# and LibUSB 1.0.20 from
# https://sourceforge.net/projects/libusb/files/
#   libusb-1.0.20.7z
#

# edit this path
SET( LIBUSBBASE C:/src/_foreign/libusb-1.0.20 )


add_definitions( -DWIN32 -D_WIN32 -DNDEBUG )

include_directories(
    ../include
    ${LIBUSBBASE}/include/libusb-1.0
)

SET( LIBUSB ${LIBUSBBASE}/MinGW32/static/libusb-1.0.a )

SET( SOCKLIBS ws2_32 wsock32 )

SET( RTLLIBFILES
    ../include/rtl_tcp.h
    ../include/reg_field.h
    ../include/rtlsdr_i2c.h

    ../src/convenience/convenience.c
    ../src/convenience/convenience.h

    ../src/getopt/getopt.c
    ../src/getopt/getopt.h

    ../include/rtl-sdr_export.h
    ../include/rtl-sdr.h
    ../src/librtlsdr.c

    ../include/tuner_e4k.h
    ../src/tuner_e4k.c

    ../include/tuner_fc0012.h
    ../src/tuner_fc0012.c

    ../include/tuner_fc0013.h
    ../src/tuner_fc0013.c

    ../include/tuner_fc2580.h
    ../src/tuner_fc2580.c

    ../include/tuner_r82xx.h
    ../src/tuner_r82xx.c
)

add_executable( rtl_test ../src/rtl_test.c  ${RTLLIBFILES} )
target_link_libraries( rtl_test ${LIBUSB} )

add_executable( rtl_fm ../src/rtl_fm.c ${RTLLIBFILES} )
target_link_libraries( rtl_fm ${LIBUSB} )

add_executable( rtl_tcp ../src/rtl_tcp.c ${RTLLIBFILES} )
target_link_libraries( rtl_tcp ${LIBUSB} ${SOCKLIBS} )

add_executable( rtl_adsb ../src/rtl_adsb.c ${RTLLIBFILES} )
target_link_libraries( rtl_adsb ${LIBUSB} )

add_executable( rtl_power ../src/rtl_power.c ${RTLLIBFILES} )
target_link_libraries( rtl_power ${LIBUSB} )

