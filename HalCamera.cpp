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

#define LOG_TAG "HalCamera"

#include "HalCamera.h"

typedef struct {
    int width;
    int height;
} preview_size_type;

// These sizes have to be a multiple of 16 in each dimension
static preview_size_type preview_sizes[] = {
    { 480, 320 }, // HVGA
    { 432, 320 }, // 1.35-to-1, for photos. (Rounded up from 1.3333 to 1)
    { 352, 288 }, // CIF
    { 320, 240 }, // QVGA
    { 240, 160 }, // SQVGA
    { 176, 144 }, // QCIF
};
#define PREVIEW_SIZE_COUNT (sizeof(preview_sizes)/sizeof(preview_size_type))

// default preview size is QVGA
#define DEFAULT_PREVIEW_SETTING 0

namespace android {


HalCamera::HalCamera(int cameraId) 
{
    LOGD("%s", __FUNCTION__);

    
    
    mCameraIndex = cameraId;
    mPreviewEnabled = false;
    mPreviewStartInProgress = false;

    mParameters.setPreviewFrameRate(15);
    mParameters.setPreviewFormat("yuv420sp");

    preview_size_type* ps = &preview_sizes[DEFAULT_PREVIEW_SETTING];
    mParameters.setPreviewSize(ps->width, ps->height);
    mParameters.setPictureFormat("jpeg"); // we do not look at this currently
    mParameters.setPictureSize(2048, 1536);
    mParameters.set("jpeg-quality", "100"); // maximum quality

    mParameters.set("nightshot-mode", "0"); // off
    mParameters.set("luma-adaptation", "0"); // FIXME: turning it on causes a crash
    mParameters.set("antibanding", "auto"); // flicker detection and elimination
    mParameters.set("whitebalance", "auto");
    mParameters.set("rotation", "0");

    mParameters.set("picture-size-values", "2048x1536,1600x1200,1024x768");

        // List supported antibanding values
    mParameters.set("antibanding-values", "off,50hz,60hz,auto");

        // List supported effects:
    mParameters.set("effect-values",
              "off,mono,negative,solarize,sepia,posterize,whiteboard,"\
                                                        "blackboard,aqua");

    // List supported exposure-offset:
    mParameters.set("exposure-offset-values", "0,1,2,3,4,5,6,7,8,9,10");

    // List of whitebalance values
    mParameters.set("whitebalance-values",
                          "auto,incandescent,fluorescent,daylight,cloudy");

    // List of ISO values
    mParameters.set("iso-values", "auto,high");

    mParameters.set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES,
    "480x320,432x320,352x288,320x240,240x160,176x144");

//LOGD("infinity = %s", CameraParameters::FOCUS_MODE_INFINITY);
//LOGD("continuous = %s", CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE);
    mParameters.set(CameraParameters::KEY_FOCUS_MODE, "infinity");
    mParameters.set(CameraParameters::KEY_SUPPORTED_FOCUS_MODES, "infinity");

    mAdapter.initialise(cameraId);
}

HalCamera::~HalCamera() 
{
    LOG_FUNCTION_NAME;
}

void HalCamera::setCallbacks(camera_notify_callback notify_cb,
                              camera_data_callback data_cb,
                              camera_data_timestamp_callback data_ts_cb,
                              camera_request_memory get_memory,
                              void *user)
{
    
    LOG_FUNCTION_NAME;
    mNotifyCb = notify_cb;
    mDataCb = data_cb;
    mDataCbTimestamp = data_ts_cb;
    mRequestMemory = get_memory;
    mCallbackCookie = user;
    LOG_FUNCTION_NAME_EXIT;
}

char* HalCamera::getParameters()
{
    String8 params_str8;
    char* params_string = NULL;
    const char * valstr = NULL;

    LOG_FUNCTION_NAME;

    params_str8 = mParameters.flatten();

    // camera service frees this string...
    params_string = (char*) malloc(sizeof(char) * (params_str8.length()+1));
    strcpy(params_string, params_str8.string());

    ///Return the current set of parameters
    return params_string;
}

int HalCamera::setParameters(const char* parameters)
{
    LOGD("%s(%s) [string]", __FUNCTION__, parameters);

    CameraParameters params;

    String8 str_params(parameters);
    params.unflatten(str_params);

    return setParameters(params);
}

int HalCamera::setParameters(const CameraParameters& params)
{
    LOGD("%s() [CameraParameters]", __FUNCTION__);

    int width, height;
    params.getPreviewSize(&width, &height);
//    LOGV("setParameters: requested size %d x %d", width, height);
    mAdapter.setPreviewSize(width, height);
    
    return 0;
}

/**
   @brief Returns true if preview is enabled

   @param none
   @return true  If preview is running currently
           false If preview has been stopped

 */
bool HalCamera::previewEnabled()
{
    LOGD("%s() -> %d", __FUNCTION__, (mPreviewEnabled || mPreviewStartInProgress));
    return (mPreviewEnabled || mPreviewStartInProgress);
}

/**
   @brief Sets ANativeWindow object that preview frames are sent to.

   Preview buffers provided to CameraHal via this object. DisplayAdapter 
   will be interfacing with it to render buffers to display.

   @param[in] window The ANativeWindow object created by Surface flinger
   @return NO_ERROR If the ANativeWindow object passes validation criteria
   @todo Define validation criteria for ANativeWindow object. 
         Define error codes for scenarios

 */
status_t HalCamera::setPreviewWindow(struct preview_stream_ops *window)
{
/*
typedef struct preview_stream_ops {
    int (*dequeue_buffer)(struct preview_stream_ops* w,
                                    buffer_handle_t** buffer, int *stride);
    int (*enqueue_buffer)(struct preview_stream_ops* w,
                                                  buffer_handle_t* buffer);
    int (*cancel_buffer)(struct preview_stream_ops* w,
                                                  buffer_handle_t* buffer);
    int (*set_buffer_count)(struct preview_stream_ops* w, int count);
    int (*set_buffers_geometry)(struct preview_stream_ops* pw,
                                                 int w, int h, int format);
    int (*set_crop)(struct preview_stream_ops *w, int left, int top, 
                                                    int right, int bottom);
    int (*set_usage)(struct preview_stream_ops* w, int usage);
    int (*set_swap_interval)(struct preview_stream_ops *w, int interval);
    int (*get_min_undequeued_buffer_count)(const struct preview_stream_ops *w,
                                                               int *count);
    int (*lock_buffer)(struct preview_stream_ops* w,
                                                  buffer_handle_t* buffer);
} preview_stream_ops_t;
*/
    status_t ret = NO_ERROR;

    LOG_FUNCTION_NAME;
    
    if (window) {
        LOGD("window: dequeue_buffer: %p", window->dequeue_buffer);
        LOGD("window: set_crop: %p", window->set_crop);
    }
    
    ///If the Camera service passes a null window, we destroy existing window and free the DisplayAdapter
    if(!window) {
        return NO_ERROR;
    }
    
    /* lie for now... */
    return NO_ERROR;
}

/**
   @brief Start preview mode.

   @param none
   @return NO_ERROR Camera switched to VF mode
   @todo Update function header with the different errors that are possible

 */
status_t HalCamera::startPreview()
{
    LOGD("%s()", __FUNCTION__);

    if (mPreviewEnabled){
      LOGD("Preview already running");
      return ALREADY_EXISTS;
    }

    /* lie about what we've done for now... */
    mPreviewEnabled = true;
    mPreviewStartInProgress = false;
    return NO_ERROR;
}


int HalCamera::beginAutoFocusThread(void *cookie)
{
    LOG_FUNCTION_NAME;
//    CameraHardwareStub *c = (CameraHardwareStub *)cookie;
//    return c->autoFocusThread();
    return NO_ERROR;
}

int HalCamera::autoFocusThread()
{
    LOG_FUNCTION_NAME;
//    if (mMsgEnabled & CAMERA_MSG_FOCUS)
//        mNotifyCb(CAMERA_MSG_FOCUS, true, 0, mCallbackCookie);
    return NO_ERROR;
}

status_t HalCamera::autoFocus()
{
    LOG_FUNCTION_NAME;
//    Mutex::Autolock lock(mLock);
//    if (createThread(beginAutoFocusThread, this) == false)
//        return UNKNOWN_ERROR;
    return NO_ERROR;
}

status_t HalCamera::cancelAutoFocus()
{
    LOG_FUNCTION_NAME;
    return NO_ERROR;
}

} /* ! namespace android */

