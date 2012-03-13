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
 * HAL Module.
 */

#define LOG_TAG "HalCameraModule"

#include <fcntl.h>
#include <strings.h>
#include <stdlib.h>
#include <utils/Errors.h>
#include <sys/ioctl.h>

#include <hardware/camera.h>
#include <cutils/log.h>

/* The function to actually open a device is in HalDevice.cpp. */
int camera_device_open(const hw_module_t*, const char *, hw_device_t**);

#define MAX_SENSORS   5   /* from the msm_camera kernel module */
static int nCameras = 0;

static int camera_get_number_of_cameras(void)
{
    LOGD("%s()", __FUNCTION__);
#define MSM_CAMERA_DEVICE_PATH   "/dev/msm_camera"
#define MSM_CAMERA_SENSOR_PATH "/dev/msm_camera/config%d"
    nCameras = 0;

    if (access(MSM_CAMERA_DEVICE_PATH, R_OK)) {
        LOGD("No camera devices listed.");
        return 0;
    }
    for (int i = 0; i < MAX_SENSORS; i++) {
        char sensor_fn[24];
        snprintf(sensor_fn, sizeof(sensor_fn), MSM_CAMERA_SENSOR_PATH, i);
        if (access(sensor_fn, R_OK))
            break;
        nCameras++;
    }
    LOGD("Total of %d cameras found", nCameras);
    return nCameras;
}

static int camera_get_camera_info(int cameraid, struct camera_info *info)
{
    int rv = 0;
    
    LOGD("%s(%d)", __FUNCTION__, cameraid);

    if (cameraid > nCameras) {
        LOGE("%s: requested camera id out of bounds.", __FUNCTION__);
        return -EINVAL;
    }

    /* ??? Can we determine this from the exposed ioctl's? */
    info->facing = CAMERA_FACING_BACK;
    info->orientation = 90;
    
    return rv;
}

static struct hw_module_methods_t camera_module_methods = {
    open: camera_device_open
};

camera_module_t HAL_MODULE_INFO_SYM = {
    common: {
        tag:           HARDWARE_MODULE_TAG,
        version_major: 1,
        version_minor: 0,
        id:            CAMERA_HARDWARE_MODULE_ID,
        name:          "msm7x30 Camera HAL Module",
        author:        "",
        methods:       &camera_module_methods,
        dso:           NULL, /* remove compilation warnings */
        reserved:      {0}, /* remove compilation warnings */
    },
    get_number_of_cameras: camera_get_number_of_cameras,
    get_camera_info:       camera_get_camera_info,
};

