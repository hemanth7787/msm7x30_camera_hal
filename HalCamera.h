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



#ifndef HAL_CAMERA_H
#define HAL_CAMERA_H

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <utils/Log.h>
#include <utils/threads.h>
#include <linux/videodev2.h>
#include <camera/CameraParameters.h>
#include <hardware/camera.h>

#include "CameraAdapter.h"

namespace android {

#define LOG_FUNCTION_NAME       LOGD("%d: %s() ENTER", __LINE__, __FUNCTION__);
#define LOG_FUNCTION_NAME_EXIT  LOGD("%d: %s() EXIT", __LINE__, __FUNCTION__);

class HalCamera {
    public:
        /** Constructor */
        HalCamera(int cameraId);

        /** Destructor */
        ~HalCamera();

        void setCallbacks(camera_notify_callback, camera_data_callback,
                              camera_data_timestamp_callback,
                              camera_request_memory, void *);
                              
        char *getParameters();

        int setParameters(const char *parameters);
        int setParameters(const CameraParameters& params);

        bool previewEnabled();
        status_t setPreviewWindow(struct preview_stream_ops *window);
        status_t startPreview();


        int beginAutoFocusThread(void *);
        int autoFocusThread();
        status_t autoFocus();
        status_t cancelAutoFocus();
        
        //@}

    private:

        //Index of current camera adapter
        int mCameraIndex;
       
        /* The parameters of this camera will be stored in this class,
         * but will be set/manipulated by the CameraAdapter class.
         */
        CameraParameters mParameters;
        /* The CameraAdapter class actually interfaces with the physical
         * device.
         */
        CameraAdapter mAdapter;
        
        /* Preview. */
        bool mPreviewEnabled;
        bool mPreviewStartInProgress;
        uint32_t mPreviewWidth;
        uint32_t mPreviewHeight;

        int mVideoWidth;
        int mVideoHeight;

        /* callback support */
        camera_notify_callback mNotifyCb;
        camera_data_callback   mDataCb;
        camera_data_timestamp_callback mDataCbTimestamp;
        camera_request_memory mRequestMemory;
        void *mCallbackCookie;

};

}

#endif /* HAL_CAMERA_H */
