# Android build config for libusb examples
# Copyright © 2012-2013 RealVNC Ltd. <toby.gray@realvnc.com>
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
#

LOCAL_PATH:= $(call my-dir)
LIBUSB_ROOT_REL:= ../..
LIBUSB_ROOT_ABS:= $(LOCAL_PATH)/../..


include $(CLEAR_VARS)

LOCAL_MODULE := rtl_test

LOCAL_SRC_FILES := \
  $(LIBRTLSDR_ROOT_REL)/src/rtl_test.c \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience/convenience.c \

LOCAL_C_INCLUDES += \
  $(LIBRTLSDR_ROOT_ABS)/include \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience \

LOCAL_LDLIBS := -llog

LOCAL_SHARED_LIBRARIES += libusb
LOCAL_SHARED_LIBRARIES += librtlsdr

include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)

LOCAL_MODULE := rtl_sdr

LOCAL_SRC_FILES := \
  $(LIBRTLSDR_ROOT_REL)/src/rtl_sdr.c \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience/convenience.c \

LOCAL_C_INCLUDES += \
  $(LIBRTLSDR_ROOT_ABS)/include \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience \

LOCAL_LDLIBS := -llog

LOCAL_SHARED_LIBRARIES += libusb
LOCAL_SHARED_LIBRARIES += librtlsdr

include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)

LOCAL_MODULE := rtl_tcp

LOCAL_SRC_FILES := \
  $(LIBRTLSDR_ROOT_REL)/src/rtl_tcp.c \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience/convenience.c \

LOCAL_C_INCLUDES += \
  $(LIBRTLSDR_ROOT_ABS)/include \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience \

LOCAL_LDLIBS := -llog

LOCAL_SHARED_LIBRARIES += libusb
LOCAL_SHARED_LIBRARIES += librtlsdr

include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)

LOCAL_MODULE := rtl_power

LOCAL_SRC_FILES := \
  $(LIBRTLSDR_ROOT_REL)/src/rtl_power.c \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience/convenience.c \

LOCAL_C_INCLUDES += \
  $(LIBRTLSDR_ROOT_ABS)/include \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience \

LOCAL_LDLIBS := -llog

LOCAL_SHARED_LIBRARIES += libusb
LOCAL_SHARED_LIBRARIES += librtlsdr

include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)

LOCAL_MODULE := rtl_fm

LOCAL_SRC_FILES := \
  $(LIBRTLSDR_ROOT_REL)/src/rtl_fm.c \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience/convenience.c \

LOCAL_C_INCLUDES += \
  $(LIBRTLSDR_ROOT_ABS)/include \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience \

LOCAL_LDLIBS := -llog

LOCAL_SHARED_LIBRARIES += libusb
LOCAL_SHARED_LIBRARIES += librtlsdr

include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)

LOCAL_MODULE := rtl_eeprom

LOCAL_SRC_FILES := \
  $(LIBRTLSDR_ROOT_REL)/src/rtl_eeprom.c \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience/convenience.c \

LOCAL_C_INCLUDES += \
  $(LIBRTLSDR_ROOT_ABS)/include \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience \

LOCAL_LDLIBS := -llog

LOCAL_SHARED_LIBRARIES += libusb
LOCAL_SHARED_LIBRARIES += librtlsdr

include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)

LOCAL_MODULE := rtl_adsb

LOCAL_SRC_FILES := \
  $(LIBRTLSDR_ROOT_REL)/src/rtl_adsb.c \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience/convenience.c \

LOCAL_C_INCLUDES += \
  $(LIBRTLSDR_ROOT_ABS)/include \
  $(LIBRTLSDR_ROOT_ABS)/src/convenience \

LOCAL_LDLIBS := -llog

LOCAL_SHARED_LIBRARIES += libusb
LOCAL_SHARED_LIBRARIES += librtlsdr

include $(BUILD_EXECUTABLE)
