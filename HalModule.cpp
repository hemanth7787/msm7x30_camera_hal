/*
 * Copyright (C) 2012 zathrasorama@gmail.com
 *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* This file provides the structures and functions required for an Android
 * HAL Module. All of the actual work is done by the CameraFactory class.
 */

#define LOG_TAG "HalCameraModule"

#include "Debug.h"
#include "CameraFactory.h"

camera_module_t HAL_MODULE_INFO_SYM = {
    common: {
        tag:           HARDWARE_MODULE_TAG,
        version_major: 1,
        version_minor: 0,
        id:            CAMERA_HARDWARE_MODULE_ID,
        name:          "msm7x30 Camera HAL Module",
        author:        "",
        methods:       &android::CameraFactory::mCameraModuleMethods,
        dso:           NULL, /* remove compilation warnings */
        reserved:      {0}, /* remove compilation warnings */
    },
    get_number_of_cameras: android::CameraFactory::get_number_of_cameras,
    get_camera_info:       android::CameraFactory::get_camera_info
};

