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


########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.6)
project(rtlsdr C)

# quite old cmake version - probably for compatibility with old OS versions?
# see https://cmake.org/cmake/help/cmake2.6docs.html


#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

if(NOT LIB_INSTALL_DIR)
   set(LIB_INSTALL_DIR lib)
endif()

OPTION(LINK_RTLTOOLS_AGAINST_STATIC_LIB "Link rtl-tools statically against librtlsdr" OFF)

OPTION(PROVIDE_UDP_SERVER "Provide UDP server for tests" OFF)

OPTION(WITH_RPC "RPC for non-Windows" OFF)

# Set the version information here
set(VERSION_INFO_MAJOR_VERSION 0) # increment major on api compatibility changes
set(VERSION_INFO_MINOR_VERSION 8) # increment minor on feature-level changes
set(VERSION_INFO_PATCH_VERSION git) # increment patch for bug fixes and docs
include(Version) # setup version info

########################################################################
# Version defines to include in library and tools
########################################################################

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rtl_app_ver.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/src/rtl_app_ver.h
    @ONLY )


if(PROVIDE_UDP_SERVER)
    add_definitions(-DWITH_UDP_SERVER)
endif()

########################################################################
# Compiler specific setup
########################################################################
if(CMAKE_COMPILER_IS_GNUCC AND NOT WIN32)
    ADD_DEFINITIONS(-Wall)
    ADD_DEFINITIONS(-Wextra)
    ADD_DEFINITIONS(-Wno-unused-parameter)
    ADD_DEFINITIONS(-Wno-unused)
    ADD_DEFINITIONS(-Wsign-compare)
    ADD_DEFINITIONS(-Wdeclaration-after-statement)
    #http://gcc.gnu.org/wiki/Visibility
    add_definitions(-fvisibility=hidden)
elseif(MSVC14 OR MSVC14)
#pthread-w32 issue, timespec is now part of time.h
    ADD_DEFINITIONS(-D_TIMESPEC_DEFINED)
endif()

OPTION(RTL_STATIC_BUILD "Build rtl-tools static (except RTLSDR.DLL) on MinGW/Win32" ON)
if(RTL_STATIC_BUILD)
    if (WIN32)
        if(MINGW)
            # Special MINGW stuff here
            # see https://cmake.org/pipermail/cmake/2012-September/051970.html
            # see http://stackoverflow.com/questions/13768515/how-to-do-static-linking-of-libwinpthread-1-dll-in-mingw
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static-libgcc -static")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libgcc -static-libstdc++")
            set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_C_FLAGS} -static-libgcc -s")
            set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -static-libgcc -static-libstdc++ -s")
        endif()
    endif()
endif()


#########################################################################
# Bug Fix
#########################################################################
OPTION(NEED_PTHREADS_WORKARROUND "PThreads Workarround for timespec")
IF (NEED_PTHREADS_WORKARROUND)
    ADD_DEFINITIONS(-DNEED_PTHREADS_WORKARROUND)
ENDIF()

########################################################################
# Find build dependencies
########################################################################
find_package(PkgConfig)
find_package(LibUSB)
if(WIN32 AND NOT MINGW)
    set(THREADS_USE_PTHREADS_WIN32 true)
endif()
find_package(Threads)

if(NOT LIBUSB_FOUND)
    message(FATAL_ERROR "LibUSB 1.0 required to compile rtl-sdr")
endif()
if(NOT THREADS_FOUND)
    message(FATAL_ERROR "pthreads(-win32) required to compile rtl-sdr")
endif()
########################################################################
# Setup the include and linker paths
########################################################################
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${LIBUSB_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/src
    ${THREADS_PTHREADS_INCLUDE_DIR}
)

#link_directories(
#    ...
#)

# Set component parameters
#set(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "" FORCE)

########################################################################
# Create uninstall target
########################################################################
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

add_custom_target(uninstall
    ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)

########################################################################
# Install udev rules
########################################################################
option(INSTALL_UDEV_RULES "Install udev rules for RTL-SDR" OFF)
if (INSTALL_UDEV_RULES)
    install (
        FILES rtl-sdr.rules
        DESTINATION "/etc/udev/rules.d"
        COMPONENT "udev"
        )
else (INSTALL_UDEV_RULES)
    message (STATUS "Udev rules not being installed, install them with -DINSTALL_UDEV_RULES=ON")
endif (INSTALL_UDEV_RULES)

option(DETACH_KERNEL_DRIVER "Detach kernel driver if loaded" OFF)
if (DETACH_KERNEL_DRIVER)
    message (STATUS "Building with kernel driver detaching enabled")
    add_definitions(-DDETACH_KERNEL_DRIVER=1)
else (DETACH_KERNEL_DRIVER)
    message (STATUS "Building with kernel driver detaching disabled, use -DDETACH_KERNEL_DRIVER=ON to enable")
endif (DETACH_KERNEL_DRIVER)

########################################################################
# Add subdirectories
########################################################################
add_subdirectory(include)
add_subdirectory(src)

########################################################################
# Create Pkg Config File
########################################################################
FOREACH(inc ${LIBUSB_INCLUDE_DIR})
    LIST(APPEND RTLSDR_PC_CFLAGS "-I${inc}")
ENDFOREACH(inc)

FOREACH(lib ${LIBUSB_LIBRARY_DIRS})
    LIST(APPEND RTLSDR_PC_LIBS "-L${lib}")
ENDFOREACH(lib)

# use space-separation format for the pc file
STRING(REPLACE ";" " " RTLSDR_PC_CFLAGS "${RTLSDR_PC_CFLAGS}")
STRING(REPLACE ";" " " RTLSDR_PC_LIBS "${RTLSDR_PC_LIBS}")

# unset these vars to avoid hard-coded paths to cross environment
IF(CMAKE_CROSSCOMPILING)
    UNSET(RTLSDR_PC_CFLAGS)
    UNSET(RTLSDR_PC_LIBS)
ENDIF(CMAKE_CROSSCOMPILING)

set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix \${prefix})
set(libdir \${exec_prefix}/${LIB_INSTALL_DIR})
set(includedir \${prefix}/include)

CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/librtlsdr.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/librtlsdr.pc
@ONLY)

INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/librtlsdr.pc
    DESTINATION ${LIB_INSTALL_DIR}/pkgconfig
)

########################################################################
# Print Summary
########################################################################
MESSAGE(STATUS "Building for version: ${VERSION} / ${LIBVER}")
MESSAGE(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
