/*
 * Copyright (C) 2011 zathrasorama@gmail.com
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

//#define LOG_NDEBUG 0
#define LOG_TAG "CameraFactory"

#include "Debug.h"
#include "CameraFactory.h"
#include "MsmCamera.h"

/* todo - move these to seperate header? */
#define MAX_SENSORS 5

#define MSM_CAMERA_DEVICE_PATH  "/dev/msm_camera"
#define MSM_CAMERA_SENSOR_PATH  "/dev/msm_camera/config%d"

android::CameraFactory gCameraFactory;
extern camera_module_t HAL_MODULE_INFO_SYM;

namespace android {

CameraFactory::CameraFactory()
        : mCameras(NULL), mCameraNum(0), mDiscoveryComplete(false) {}

CameraFactory::~CameraFactory()
{
    if (mCameras != NULL) {
        for (int n = 0; n < mCameraNum; n++) {
            if (mCameras[n] != NULL) {
                delete mCameras[n];
            }
        }
        delete[] mCameras;
    }
}

/****************************************************************************
 * Camera HAL API handlers.
 *
 * Each handler simply verifies existence of an appropriate EmulatedCamera
 * instance, and dispatches the call to that instance.
 *
 ***************************************************************************/

int CameraFactory::cameraDeviceOpen(int camera_id, hw_device_t** device)
{
    LOGD("%s: id = %d", __FUNCTION__, camera_id);

    *device = NULL;

    if (!mDiscoveryComplete)
        findCameras();

    if (camera_id < 0 || camera_id >= getCameraNum()) {
        LOGE("%s: Camera id %d is out of bounds (%d)",
             __FUNCTION__, camera_id, getCameraNum());
        return -EINVAL;
    }

    return mCameras[camera_id]->connectCamera(device);
}

int CameraFactory::getCameraNum()
{
    LOGD("%s()", __FUNCTION__);
    if (!mDiscoveryComplete)
        findCameras();
    return mCameraNum; 
}

int CameraFactory::getCameraInfo(int camera_id, struct camera_info* info)
{
    LOGD("%s: id = %d", __FUNCTION__, camera_id);
    if (!mDiscoveryComplete)
        findCameras();

    if (camera_id < 0 || camera_id >= getCameraNum()) {
        LOGE("%s: Camera id %d is out of bounds (%d)", __FUNCTION__, 
             camera_id, getCameraNum());
        return -EINVAL;
    }

    return mCameras[camera_id]->getCameraInfo(info);
}


void CameraFactory::findCameras()
{
    int found = 0;

    if (access(MSM_CAMERA_DEVICE_PATH, R_OK)) {
        LOGD("No camera devices listed.");
        return;
    }
    for (int i = 0; i < MAX_SENSORS; i++) {
        char sensor_fn[24];
        snprintf(sensor_fn, sizeof(sensor_fn), MSM_CAMERA_SENSOR_PATH, i);
        if (access(sensor_fn, R_OK))
            break;
        found++;
    }
    LOGD("Total of %d cameras found", found);

    if (found > 0) {
        /* Allocate the mCameras array, but zero it out. Actual objects
         * will only be allocated when requested.
         */
        mCameras = new Camera*[found];
        if (mCameras == NULL) {
            LOGE("%s: Unable to allocate emulated camera array for %d entries",
                 __FUNCTION__, found);
            return;
        }
        memset(mCameras, 0, sizeof(Camera*) * (found));
        for (int i = 0; i < found; i++) {
            MsmCamera* cam = new MsmCamera(i, &HAL_MODULE_INFO_SYM.common);
            if (NULL != cam) {
                status_t res = cam->Initialize();
                if (NO_ERROR == res)
                    mCameras[i] = cam;
                else
                    delete cam;
            }
        }
        mCameraNum = found;
    }
    mDiscoveryComplete = true;
}


int CameraFactory::get_number_of_cameras(void)
{
    return gCameraFactory.getCameraNum();
}

int CameraFactory::get_camera_info(int camera_id, struct camera_info* info)
{
    return gCameraFactory.getCameraInfo(camera_id, info);
}

int CameraFactory::device_open(const hw_module_t* module, const char* name,
                               hw_device_t** device)
{
    /* This called with
     * - module - pointer to a hw_module_t
     * - name   - usually a number string
     * - device - holder for pointer to created hw_device_t structure
     */
    /* Check whether the request came from the expected module. */
    if (module != &HAL_MODULE_INFO_SYM.common) {
        LOGE("%s: Invalid module %p vs %p", __FUNCTION__, module, 
             &HAL_MODULE_INFO_SYM.common);
        return -EINVAL;
    }
    /* Have we been given a valid name? */
    if (name == NULL) {
        LOGE("%s: NULL name is not expected here", __FUNCTION__);
        return -EINVAL;
    }
    LOGD("%s: Trying to open camera %s", __FUNCTION__, name);
    return gCameraFactory.cameraDeviceOpen(atoi(name), device);
}

struct hw_module_methods_t CameraFactory::mCameraModuleMethods = {
    open: device_open
};

}; /* namespace android */

