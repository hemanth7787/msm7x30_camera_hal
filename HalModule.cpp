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

#include "Camera.h"
#include "MsmCamera.h"

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

static int camera_device_close(hw_device_t* device)
{
    LOGD("%s()", __FUNCTION__);

    if (!device)
        return -EINVAL;
    camera_device_t * dev = (camera_device_t *) device;

    if (dev->priv)
        delete (android::Camera *)(dev->priv);

    free(dev);
    return 0;
}

extern camera_device_ops_t generic_camera_ops;

int msm_camera_open(const hw_module_t* module, const char* name,
                    hw_device_t** device)
{
    int cameraid = atoi(name);

    android::Camera *ac = new android::Camera(cameraid);
    android::MsmCamera *msm = new android::MsmCamera(cameraid, ac);
        
    camera_device_t *cameraDev = NULL;
       
    LOGD("%s(%s)", __FUNCTION__, name);
    *device = NULL;

    /* ??? Should we check that the requested camera exists? */
    cameraDev = (camera_device_t *)malloc(sizeof(*cameraDev));
    if (!cameraDev) {
        LOGE("Failed to allocate memory for camera_device_t structure");
        return -ENOMEM;
    }
    /* Set the correct information in the device structure. */
    memset(cameraDev, 0, sizeof(*cameraDev));
    cameraDev->common.tag = HARDWARE_DEVICE_TAG;
    cameraDev->common.version = 0;
    cameraDev->common.module = (hw_module_t *)(module);
    cameraDev->common.close = camera_device_close;

    /* Set the function pointers for the camera operations */

    cameraDev->ops = &generic_camera_ops;
    cameraDev->priv = ac;
    
    *device = &cameraDev->common;
    return 0;
}

static struct hw_module_methods_t camera_module_methods = {
    open: msm_camera_open
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

