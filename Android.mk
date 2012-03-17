# Copyright (C) 2012 zathrasorama@gmail.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

ifeq ($(TARGET_BOARD_PLATFORM),msm7x30)

LOCAL_PATH:= $(call my-dir)
	
include $(CLEAR_VARS)
	
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_MODULE := camera.$(TARGET_BOARD_PLATFORM)
	
LOCAL_MODULE_TAGS := optional
	
LOCAL_SRC_FILES := \
    HalModule.cpp \
    AbstractedDevice.cpp \
    AbstractedCamera.cpp \
    CameraMessages.cpp \
    CameraAdapter.cpp \
    Memory.cpp \
    ParameterHelper.cpp \
    CallbackNotifier.cpp

LOCAL_SHARED_LIBRARIES := liblog libutils libcutils libbinder
LOCAL_SHARED_LIBRARIES += libui libhardware libcamera_client
LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)
	
endif
