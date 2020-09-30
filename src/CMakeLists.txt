# Copyright 2012 OSMOCOM Project
#
# This file is part of rtl-sdr
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

MACRO(RTLSDR_APPEND_SRCS)
    LIST(APPEND rtlsdr_srcs ${ARGV})
ENDMACRO(RTLSDR_APPEND_SRCS)

RTLSDR_APPEND_SRCS(
    librtlsdr.c
    tuner_e4k.c
    tuner_fc0012.c
    tuner_fc0013.c
    tuner_fc2580.c
    tuner_r82xx.c
)
if(WITH_RPC)
    RTLSDR_APPEND_SRCS(
        rtlsdr_rpc.c
        rtlsdr_rpc_msg.c
    )
endif()

########################################################################
# Set up Windows DLL resource files
########################################################################
IF(MSVC)
    include(${CMAKE_SOURCE_DIR}/cmake/Modules/Version.cmake)

    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/rtlsdr.rc.in
        ${CMAKE_CURRENT_BINARY_DIR}/rtlsdr.rc
    @ONLY)

    RTLSDR_APPEND_SRCS(${CMAKE_CURRENT_BINARY_DIR}/rtlsdr.rc)
ENDIF(MSVC)

########################################################################
# Setup shared library variant
########################################################################
add_library(rtlsdr_shared SHARED ${rtlsdr_srcs})
if(NOT WIN32)
    target_link_libraries(rtlsdr_shared ${LIBUSB_LIBRARIES})
else()
    target_link_libraries(rtlsdr_shared ws2_32 ${LIBUSB_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
endif()
if (WITH_RPC)
    target_compile_definitions(rtlsdr_shared PUBLIC _ENABLE_RPC)
endif()

set_target_properties(rtlsdr_shared PROPERTIES DEFINE_SYMBOL "rtlsdr_EXPORTS")
set_target_properties(rtlsdr_shared PROPERTIES OUTPUT_NAME rtlsdr)
set_target_properties(rtlsdr_shared PROPERTIES SOVERSION ${MAJOR_VERSION})
set_target_properties(rtlsdr_shared PROPERTIES VERSION ${LIBVER})

########################################################################
# Setup static library variant
########################################################################
add_library(rtlsdr_static STATIC ${rtlsdr_srcs})
if(NOT WIN32)
    target_link_libraries(rtlsdr_static ${LIBUSB_LIBRARIES})
else()
    target_link_libraries(rtlsdr_static ws2_32 ${LIBUSB_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
endif()

if (WITH_RPC)
    target_compile_definitions(rtlsdr_static PUBLIC _ENABLE_RPC)
endif()

set_property(TARGET rtlsdr_static APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
if(NOT WIN32)
    # Force same library filename for static and shared variants of the library
    set_target_properties(rtlsdr_static PROPERTIES OUTPUT_NAME rtlsdr)
endif()


########################################################################
# Set link library
########################################################################

if(LINK_RTLTOOLS_AGAINST_STATIC_LIB)
    set(RTLSDR_TOOL_LIB rtlsdr_static)
else()
    set(RTLSDR_TOOL_LIB rtlsdr_shared)
endif()

########################################################################
# Setup libraries used in executables
########################################################################
add_library(convenience_static STATIC
    convenience/rtl_convenience.c  convenience/convenience.c  convenience/wavewrite.c
)

if(WIN32)
add_library(libgetopt_static STATIC
    getopt/getopt.c
)
target_link_libraries(convenience_static
    ${RTLSDR_TOOL_LIB}
)
endif()

########################################################################
# Build utility
########################################################################
add_executable(rtl_sdr rtl_sdr.c)
add_executable(rtl_tcp rtl_tcp.c controlThread.c)
add_executable(rtl_udp rtl_udp.c)
add_executable(rtl_test rtl_test.c)
add_executable(rtl_fm rtl_fm.c)
add_executable(rtl_ir rtl_ir.c)
add_executable(rtl_eeprom rtl_eeprom.c)
add_executable(rtl_adsb rtl_adsb.c)
add_executable(rtl_power rtl_power.c)
add_executable(rtl_biast rtl_biast.c)
add_executable(rtl_raw2wav    rtl_raw2wav.c     convenience/convenience.c  convenience/wavewrite.c)
add_executable(rtl_wavestat   rtl_wavestat.c    convenience/convenience.c  convenience/waveread.c)
add_executable(rtl_wavestream rtl_wavestream.c  convenience/convenience.c  convenience/waveread.c)

if (WITH_RPC)
    add_executable(rtl_rpcd rtl_rpcd.c rtlsdr_rpc_msg.c)
    set(INSTALL_TARGETS rtlsdr_shared rtlsdr_static rtl_sdr rtl_tcp rtl_udp rtl_test rtl_fm rtl_ir rtl_eeprom rtl_adsb rtl_power rtl_biast rtl_raw2wav rtl_wavestat rtl_wavestream rtl_rpcd)
else()
    set(INSTALL_TARGETS rtlsdr_shared rtlsdr_static rtl_sdr rtl_tcp rtl_udp rtl_test rtl_fm rtl_ir rtl_eeprom rtl_adsb rtl_power rtl_biast rtl_raw2wav rtl_wavestat rtl_wavestream)
endif()

target_link_libraries(rtl_sdr ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_tcp ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_test ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_fm ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_ir ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_eeprom ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_adsb ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_udp ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_power ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_biast ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
if(WITH_RPC)
    target_link_libraries(rtl_rpcd ${RTLSDR_TOOL_LIB} convenience_static
        ${LIBUSB_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
    )
endif()

if(UNIX)
    target_link_libraries(rtl_fm m)
    target_link_libraries(rtl_ir m)
    target_link_libraries(rtl_adsb m)
    target_link_libraries(rtl_power m)
    target_link_libraries(rtl_raw2wav m)
    target_link_libraries(rtl_wavestat m)
    target_link_libraries(rtl_wavestream m)
    if(APPLE)
        target_link_libraries(rtl_test m)
    else()
        target_link_libraries(rtl_test m rt)
    endif()
endif()

if(WIN32)
    target_link_libraries(rtl_sdr libgetopt_static)
    target_link_libraries(rtl_tcp ws2_32 libgetopt_static)
    target_link_libraries(rtl_udp ws2_32 libgetopt_static)
    target_link_libraries(rtl_test libgetopt_static)
    target_link_libraries(rtl_fm libgetopt_static)
    target_link_libraries(rtl_ir libgetopt_static)
    target_link_libraries(rtl_eeprom libgetopt_static)
    target_link_libraries(rtl_adsb libgetopt_static)
    target_link_libraries(rtl_power libgetopt_static)
    target_link_libraries(rtl_raw2wav libgetopt_static m)
    target_link_libraries(rtl_wavestat libgetopt_static m)
    target_link_libraries(rtl_wavestream libgetopt_static m)
    set_property(TARGET rtl_sdr APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
    set_property(TARGET rtl_tcp APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
    set_property(TARGET rtl_udp APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
    set_property(TARGET rtl_test APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
    set_property(TARGET rtl_fm APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
    set_property(TARGET rtl_ir APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
    set_property(TARGET rtl_eeprom APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
    set_property(TARGET rtl_adsb APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
    set_property(TARGET rtl_power APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
    set_property(TARGET rtl_raw2wav APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
    set_property(TARGET rtl_wavestat APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
    set_property(TARGET rtl_wavestream APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
    if (WITH_RPC)
        target_link_libraries(rtl_rpcd ws2_32 libgetopt_static)
        set_property(TARGET rtl_rpcd APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
    endif()
endif()
########################################################################
# Install built library files & utilities
########################################################################
install(TARGETS ${INSTALL_TARGETS}
    LIBRARY DESTINATION ${LIB_INSTALL_DIR} # .so/.dylib file
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)
