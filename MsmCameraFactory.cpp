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

#define LOG_TAG "MsmCameraFactory"

#include "MsmCameraFactory.h"
#include "MsmCamera.h"

/* A global instance of MsmCameraFactory is statically instantiated and
 * initialized when camera emulation HAL is loaded.
 */
android::MsmCameraFactory  gCameraFactory;

namespace android {

MsmCameraFactory::MsmCameraFactory() : CameraFactory()
{
    int found = 0;
#define MAX_SENSORS 5
#define MSM_CAMERA_DEVICE_PATH   "/dev/msm_camera"
#define MSM_CAMERA_SENSOR_PATH "/dev/msm_camera/config%d"
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
        mCameras = new Camera*[found];
        if (mCameras == NULL) {
            LOGE("%s: Unable to allocate emulated camera array for %d entries",
                 __FUNCTION__, found);
            return;
        }
        memset(mCameras, 0, sizeof(Camera*) * (found));

        for (int i = 0; i < found; i++) {
            MsmCamera *cam = new MsmCamera(i);
            if (NULL != cam)
                mCameras[i] = cam;
        }
        mCameraNum = found;
    }
    mConstructedOK = true;
}


}

