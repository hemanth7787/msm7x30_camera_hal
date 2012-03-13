/*
 * Copyright (C) 2012 zathrasorama@gmail.com
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



#ifndef CAMERA_ADAPTER_H
#define CAMERA_ADAPTER_H

#include <fcntl.h>

#include <binder/MemoryBase.h>
#include <binder/MemoryHeapBase.h>
#include <utils/threads.h>
#include <camera/CameraParameters.h>

extern "C" {
    #include <linux/android_pmem.h>
}

/* Include the file from the Linux kernel module. */
#include "msm_camera.h"
#include "Memory.h"

typedef enum {
	AF_MODE_NORMAL,
	AF_MODE_MACRO,
	AF_MODE_AUTO,
} af_mode_t;

typedef enum {
    MSM_CONFIG, MSM_CONTROL, MSM_FB
} msm_endpoint_e;

namespace android {

class SensorConfig;

class CameraAdapter: public virtual RefBase {
    public:
        CameraAdapter();
        ~CameraAdapter();

        bool initialise(int);
        bool setPreviewSize(int, int);
        bool setPreviewMemory();
//        void enableMsgType(int32_t msgs, frame_callback callback, 
//                                     event_callback eventCb, void* cookie);
//        void disableMsgType(int32_t msgs, void* cookie);

        int openEndpoint(msm_endpoint_e);
        bool configCommand(int, void *);
        bool controlCommand(uint16_t, void *, uint16_t);

        bool configIoctl(int, void *);
        bool controlIoctl(int, void *);
        
    private:
        
        /* Functions to extract information from the sensor directly. */
        void getVendorId();
        bool getSensorInformation();

        bool getMaximumZoom();
        bool getMaximumAFSteps();
        bool getPictureFPS();
        
        /* Functions to change settings for the sensor. */
        bool setMode(uint16_t);
        bool setAFMode(af_mode_t af_type);

        bool __ioctl(int, int, void *);
        bool setupMemory();

    private:
        int mCameraId;
        
        struct msm_camsensor_info mSensorInfo;
        int mVendorId;
        
        mutable Mutex mSubscriberLock;

        int mConfigFd;
        int mControlFd;
        
        int previewWidth;
        int previewHeight;

        int pictWidth;
        int pictHeight;

        int prevl_pf;
        int prevp_pl;

        PhysicalMemoryPool *mSetupMemory;
};

} /* ! android */

#endif /* ! CAMERA_ADAPTER_H */
