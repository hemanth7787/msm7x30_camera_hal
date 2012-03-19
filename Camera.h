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



#ifndef CAMERA_H
#define CAMERA_H

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

#include "CameraDevice.h"
#include "PreviewWindow.h"
#include "CallbackNotifier.h"

namespace android {

#define LOG_FUNCTION_NAME       LOGD("%d: %s() ENTER", __LINE__, __FUNCTION__);
#define LOG_FUNCTION_NAME_EXIT  LOGD("%d: %s() EXIT", __LINE__, __FUNCTION__);

class Camera {
    public:
        /** Constructor */
        Camera(int cameraId);

        /** Destructor */
        ~Camera();

        /* Callbacks */
        void setCallbacks(camera_notify_callback, camera_data_callback,
                              camera_data_timestamp_callback,
                              camera_request_memory, void *);
        void enableMsgType(int32_t msg_type);
        void disableMsgType(int32_t msg_type);
        int isMsgTypeEnabled(int32_t msg_type);

        /* Camera Parameters */      
        char *getParameters();
        int setParameters(const char *parameters);
        void putParameters(const char *parameters);
        int setParameters(const CameraParameters& params);

        /* Preview */
        bool isPreviewEnabled();
        status_t setPreviewWindow(struct preview_stream_ops *window);
        status_t startPreview();
        void stopPreview();

        /* Metadata */        
        status_t storeMetaDataInBuffers(int enable);

        /* Recording */
        status_t startRecording();
        void stopRecording();
        int isRecordingEnabled();
        void releaseRecordingFrame(const void* opaque);

        /* Autofocus */
        int beginAutoFocusThread(void *);
        int autoFocusThread();
        status_t autoFocus();
        status_t cancelAutoFocus();

        status_t takePicture();
        status_t cancelPicture();

        /* Camera Management */
        status_t sendCommand(int32_t cmd, int32_t a1, int32_t a2);
        void releaseCamera();
        status_t dumpCamera(int fd);

        
        //@}

    private:
        status_t allocPreviewBuffers(int width, int height, 
                                     const char* previewFormat,
                                     unsigned int buffercount, 
                                     unsigned int &max_queueable);

    private:

        static const int32_t MAX_BUFFERS = 8;

        //Index of current camera adapter
        int mCameraIndex;
       
        /* The parameters of this camera will be stored in this class,
         * but will be set/manipulated by the CameraAdapter class.
         */
        CameraParameters mParameters;
        /* The CameraAdapter class actually interfaces with the physical
         * device.
         */
        CameraDevice *mAdapter;
        sp<PreviewWindow> mPreviewWindow;

        CallbackNotifier mCallbackNotifier;        
        int mVideoWidth;
        int mVideoHeight;

        /* callback support */
        camera_notify_callback mNotifyCb;
        camera_data_callback   mDataCb;
        camera_data_timestamp_callback mDataCbTimestamp;
        camera_request_memory mRequestMemory;
        void *mCallbackCookie;

        bool mPreviewing;
        camera_memory_t* mPreviewMemory;
        int mPreviewBufCount;
        const char *mPreviewPixelFormat;
//        KeyedVector<unsigned int, sp<MemoryHeapBase> > mSharedPreviewHeaps;
//        KeyedVector<unsigned int, sp<MemoryBase> > mSharedPreviewBuffers;

        /* Preview. */
        bool mPreviewEnabled;
        bool mPreviewStartInProgress;
        bool mSetPreviewWindowCalled;
//        BufferProvider *mPreviewBufProvider;
        int32_t *mPreviewBufs;
        uint32_t *mPreviewOffsets;
        int mPreviewLength;
        int mPreviewFd;

        int mPreviewWidth;
        int mPreviewHeight;
        int mPreviewFormat; /* HAL_PIXEL_FORMAT_xxx constant */

        /* Picture */
        int mPictureWidth;
        int mPictureHeight;
        int mPictureFormat; /* HAL_PIXEL_FORMAT_xxx constant */
                
        /* Display */
        bool mDisplayPaused;
};

}

#endif /* HAL_CAMERA_H */
