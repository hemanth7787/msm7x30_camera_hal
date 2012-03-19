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


#ifndef MSM_CAMERA_H
#define MSM_CAMERA_H

#include "Camera.h"
#include "msm_camera.h"
#include "Memory.h"

namespace android {

class MsmCamera : public CameraDevice
{
    typedef enum {
        MSM_CONFIG,     /* configuration */
        MSM_CONTROL,    /* control */
        MSM_FB          /* framebuffer */
    } msm_endpoint_e;

    public:
        MsmCamera(int id, Camera *cam);

        status_t connectDevice();
        status_t disconnectDevice();
        status_t startDevice(int width, int height, uint32_t pix_fmt);
        status_t stopDevice();

    public:
        /* API for this class */
        bool configIoctl(int cmd, void *ptr);
        bool controlIoctl(int cmd, void *ptr);
        bool configCommand(int cmd, void *ptr);
        bool controlCommand(uint16_t type, void *ptr, uint16_t ptrlen);

    private:
        int openEndpoint(msm_endpoint_e which);

        /* Functions to extract information from the sensor. */
        bool getVendorId();
        bool getSensorInformation();
        /* Control memory allocations for sensor operation */
        bool setupMemory();
        void removeMemory();
        /* Issue ioctl requests with suitable checks */
        bool __ioctl(int which, int cmd, void *ptr);

    private:
        int mCameraId;   /* The camera id. */
        
        /* Kernel access points */
        int mControlFd;
        int mConfigFd;
        /* Sensor information */
        struct msm_camsensor_info mSensorInfo;
        int mVendorId;
        /* Sensor memory pool */
        PhysicalMemoryPool *mSetupMemory;

};

}; /* ! namespace android */

#endif /* MSM_CAMERA_H */
