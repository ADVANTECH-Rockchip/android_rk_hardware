# Copyright 2006 The Android Open Source Project

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	rild.c

LOCAL_SHARED_LIBRARIES := \
	libcutils \
	libdl \
	liblog \
	libril

# Temporary hack for broken vendor RILs.
LOCAL_WHOLE_STATIC_LIBRARIES := \
	librilutils_static

LOCAL_CFLAGS := -DRIL_SHLIB
LOCAL_CFLAGS += -Wall -Wextra -Werror

ifeq ($(SIM_COUNT), 2)
    LOCAL_CFLAGS += -DANDROID_MULTI_SIM
    LOCAL_CFLAGS += -DANDROID_SIM_COUNT_2
endif

LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE:= rild
ifeq ($(strip $(TARGET_PRODUCT)), usm110)
    LOCAL_INIT_RC := usm110/rild.rc
endif
ifeq ($(strip $(TARGET_PRODUCT)), rsb4680)
    LOCAL_INIT_RC := rsb4680/rild.rc
endif

include $(BUILD_EXECUTABLE)

