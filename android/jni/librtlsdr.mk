# Android build config for libusb
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
LIBRTLSDR_ROOT_REL:= ../..
LIBRTLSDR_ROOT_ABS:= $(LOCAL_PATH)/../..


# libusb 
include $(CLEAR_VARS)

LOCAL_MODULE := libusb

LOCAL_SRC_FILES := $(LIBUSB_PREBUILDS)/$(TARGET_ARCH_ABI)/libusb1.0.so

LOCAL_EXPORT_C_INCLUDES := \
  $(LIBUSB_INCLUDES)/libusb \

include $(PREBUILT_SHARED_LIBRARY)


# librtlsdr
include $(CLEAR_VARS)

LOCAL_MODULE := librtlsdr

LOCAL_SRC_FILES := \
  $(LIBRTLSDR_ROOT_REL)/src/librtlsdr.c \
  $(LIBRTLSDR_ROOT_REL)/src/tuner_e4k.c \
  $(LIBRTLSDR_ROOT_REL)/src/tuner_fc0012.c \
  $(LIBRTLSDR_ROOT_REL)/src/tuner_fc0013.c \
  $(LIBRTLSDR_ROOT_REL)/src/tuner_fc2580.c \
  $(LIBRTLSDR_ROOT_REL)/src/tuner_r82xx.c \

LOCAL_C_INCLUDES += \
  $(LIBRTLSDR_ROOT_ABS)/include \

LOCAL_LDLIBS := -llog

LOCAL_SHARED_LIBRARIES += libusb

include $(BUILD_SHARED_LIBRARY)
