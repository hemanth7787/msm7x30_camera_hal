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

#define LOG_TAG "AbstractedCam"

#include "AbstractedCamera.h"
#include "ParameterHelper.h"

namespace android {

AbstractedCamera::AbstractedCamera(int cameraId) : mCallbackNotifier()
{
    LOGD("%s", __FUNCTION__);
  
    mCameraIndex = cameraId;
    
    mPreviewEnabled = false;
    mPreviewStartInProgress = false;
    mSetPreviewWindowCalled = false;
    mPreviewWidth = 0;
    mPreviewHeight = 0;

    mDisplayPaused = false;  

    mPreviewBufs = NULL;
    
    size_entry_t previews[] = {
        { 640, 480, "640x480" },
        { 320, 240, "320x240" }
    };
    size_entry_t sizes[] = {
        { 2048, 1536, "2048x1536" },
        { 1600, 1200, "1600x1200" },
        { 1024, 768, "1024x768" }
    };
    const char *antibanding[] = { "off", "50hz", "60hz", "auto" };
    const char *effects[] = { "off", "mono", "negative", "solarize", "sepia",
                              "posterize", "whiteboard" "blackboard","aqua"};
    const char *iso[] = { "auto", "high" };
    const char *scenes[] = { "auto","night","fireworks" };
    const char *awb[] = { "auto","incandescent","fluorescent","daylight",
                          "cloudy" };
    const char *focus[] = { "infinity" };
    int exp_offset[] = { 0,1,2,3,4,5,6,7,8,9,10 };

#define ARRAY_SIZE(x)   (sizeof((x)) / sizeof((x[0])))

    ParameterHelper helper(&mParameters);   
    helper.setPreviewSizes(previews, ARRAY_SIZE(previews));
    helper.setPreviewSize(previews[0]);
    helper.setPictureSizes(sizes, ARRAY_SIZE(sizes));
    helper.setPictureSize(sizes[0]);   
    helper.setStringList(CameraParameters::KEY_SUPPORTED_ANTIBANDING, antibanding, ARRAY_SIZE(antibanding));
    helper.setStringList(CameraParameters::KEY_SUPPORTED_EFFECTS, effects, ARRAY_SIZE(effects));
    helper.setStringList(CameraParameters::KEY_SUPPORTED_SCENE_MODES, scenes, ARRAY_SIZE(scenes));
    helper.setStringList(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE, awb, ARRAY_SIZE(awb));
    helper.setStringList(CameraParameters::KEY_SUPPORTED_FOCUS_MODES, focus, ARRAY_SIZE(focus));

//    helper.setIntList(CameraParameters::KEY_SUPPORTED_EFFECTS, antibanding);

    /* Set Defaults */    
    mParameters.setPreviewFrameRate(15);
    mParameters.setPreviewFormat("yuv420sp");
    mParameters.setPictureFormat("jpeg");
    mParameters.set(CameraParameters::KEY_JPEG_QUALITY, 90);
    mParameters.set(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY, 90);
    mParameters.set(CameraParameters::KEY_ANTIBANDING, "auto");
    mParameters.set(CameraParameters::KEY_WHITE_BALANCE, "auto");
    mParameters.set(CameraParameters::KEY_ROTATION, 0);
    mParameters.set(CameraParameters::KEY_FOCUS_MODE, "infinity");
    mParameters.set(CameraParameters::KEY_SCENE_MODE, 
                    CameraParameters::SCENE_MODE_AUTO);

    mParameters.dump();
//    mParameters.set("luma-adaptation", "0"); // FIXME: turning it on causes a crash
    // List supported exposure-offset:
//    mParameters.set("exposure-offset-values", "0,1,2,3,4,5,6,7,8,9,10");
    // List of ISO values
//    mParameters.set("iso-values", "auto,high");

    mAdapter.initialise(cameraId);
}

AbstractedCamera::~AbstractedCamera() 
{
    LOG_FUNCTION_NAME;
}

/**************************************************************************
 *  Callbacks
 *************************************************************************/
void AbstractedCamera::setCallbacks(camera_notify_callback notify_cb,
                             camera_data_callback data_cb,
                             camera_data_timestamp_callback data_cb_timestamp,
                             camera_request_memory get_memory,
                             void* user)
{
    mCallbackNotifier.setCallbacks(notify_cb, data_cb, data_cb_timestamp,
                                    get_memory, user);
}

void AbstractedCamera::enableMsgType(int32_t msg_type)
{
    mCallbackNotifier.enableMessage(msg_type);
}

void AbstractedCamera::disableMsgType(int32_t msg_type)
{
    mCallbackNotifier.disableMessage(msg_type);
}

int AbstractedCamera::isMsgTypeEnabled(int32_t msg_type)
{
    return mCallbackNotifier.isMessageEnabled(msg_type);
}

/**************************************************************************
 *  Metadata
 *************************************************************************/
status_t AbstractedCamera::storeMetaDataInBuffers(int enable)
{
    /* Callback should return a negative errno. */
    return -mCallbackNotifier.storeMetaDataInBuffers(enable);
}

/**************************************************************************
 *  Recording (aka video)
 *************************************************************************/
status_t AbstractedCamera::startRecording()
{
    /* Callback should return a negative errno. */
    return -mCallbackNotifier.enableVideoRecording(mParameters.getPreviewFrameRate());
}

void AbstractedCamera::stopRecording()
{
    mCallbackNotifier.disableVideoRecording();
}

int AbstractedCamera::isRecordingEnabled()
{
    return mCallbackNotifier.isVideoRecordingEnabled();
}

void AbstractedCamera::releaseRecordingFrame(const void* opaque)
{
    mCallbackNotifier.releaseRecordingFrame(opaque);
}

/**************************************************************************
 *  Parameters
 *************************************************************************/
char* AbstractedCamera::getParameters()
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

int AbstractedCamera::setParameters(const char* parameters)
{
    LOGD("%s(%s) [string]", __FUNCTION__, parameters);

    CameraParameters params;

    String8 str_params(parameters);
    params.unflatten(str_params);

    return setParameters(params);
}

void AbstractedCamera::putParameters(const char* parameters)
{
    LOG_FUNCTION_NAME
    CameraParameters params;

    String8 str_params(parameters);
    params.unflatten(str_params);

    params.dump();
    setParameters(params);
}    

int AbstractedCamera::setParameters(const CameraParameters& params)
{
    LOGD("%s() [CameraParameters]", __FUNCTION__);

    int width, height;
    params.getPreviewSize(&width, &height);
//    LOGV("setParameters: requested size %d x %d", width, height);
    mAdapter.setPreviewSize(width, height);

    const char *keys[] = {
        CameraParameters::KEY_PREVIEW_SIZE,
        CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES,
        CameraParameters::KEY_PREVIEW_FPS_RANGE,
        CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE,
        CameraParameters::KEY_PREVIEW_FORMAT,
        CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS,
        CameraParameters::KEY_PREVIEW_FRAME_RATE,
        CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES,
        CameraParameters::KEY_PICTURE_SIZE,
        CameraParameters::KEY_SUPPORTED_PICTURE_SIZES,
        CameraParameters::KEY_PICTURE_FORMAT,
        CameraParameters::KEY_SUPPORTED_PICTURE_FORMATS,
        CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH,
        CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT,
        CameraParameters::KEY_SUPPORTED_JPEG_THUMBNAIL_SIZES,
        CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY,
        CameraParameters::KEY_JPEG_QUALITY,
        CameraParameters::KEY_ROTATION,
        CameraParameters::KEY_GPS_LATITUDE,
        CameraParameters::KEY_GPS_LONGITUDE,
        CameraParameters::KEY_GPS_ALTITUDE,
        CameraParameters::KEY_GPS_TIMESTAMP,
        CameraParameters::KEY_GPS_PROCESSING_METHOD,
        CameraParameters::KEY_WHITE_BALANCE,
        CameraParameters::KEY_SUPPORTED_WHITE_BALANCE,
        CameraParameters::KEY_EFFECT,
        CameraParameters::KEY_SUPPORTED_EFFECTS,
        CameraParameters::KEY_ANTIBANDING,
        CameraParameters::KEY_SUPPORTED_ANTIBANDING,
        CameraParameters::KEY_SCENE_MODE,
        CameraParameters::KEY_SUPPORTED_SCENE_MODES,
        CameraParameters::KEY_FLASH_MODE,
        CameraParameters::KEY_SUPPORTED_FLASH_MODES,
        CameraParameters::KEY_FOCUS_MODE,
        CameraParameters::KEY_SUPPORTED_FOCUS_MODES,
        CameraParameters::KEY_MAX_NUM_FOCUS_AREAS,
        CameraParameters::KEY_FOCUS_AREAS,
        CameraParameters::KEY_FOCAL_LENGTH,
        CameraParameters::KEY_HORIZONTAL_VIEW_ANGLE,
        CameraParameters::KEY_VERTICAL_VIEW_ANGLE,
        CameraParameters::KEY_EXPOSURE_COMPENSATION,
        CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION,
        CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION,
        CameraParameters::KEY_EXPOSURE_COMPENSATION_STEP,
        CameraParameters::KEY_AUTO_EXPOSURE_LOCK,
        CameraParameters::KEY_AUTO_EXPOSURE_LOCK_SUPPORTED,
        CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK,
        CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK_SUPPORTED,
        CameraParameters::KEY_MAX_NUM_METERING_AREAS,
        CameraParameters::KEY_METERING_AREAS,
        CameraParameters::KEY_ZOOM,
        CameraParameters::KEY_MAX_ZOOM,
        CameraParameters::KEY_ZOOM_RATIOS,
        CameraParameters::KEY_ZOOM_SUPPORTED,
        CameraParameters::KEY_SMOOTH_ZOOM_SUPPORTED,
        CameraParameters::KEY_FOCUS_DISTANCES,
        CameraParameters::KEY_VIDEO_SIZE,
        CameraParameters::KEY_SUPPORTED_VIDEO_SIZES,
        CameraParameters::KEY_MAX_NUM_DETECTED_FACES_HW,
        CameraParameters::KEY_MAX_NUM_DETECTED_FACES_SW,
        CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO,
        CameraParameters::KEY_VIDEO_FRAME_FORMAT,
        CameraParameters::KEY_RECORDING_HINT,
        CameraParameters::KEY_VIDEO_SNAPSHOT_SUPPORTED,
        CameraParameters::KEY_VIDEO_STABILIZATION,
        CameraParameters::KEY_VIDEO_STABILIZATION_SUPPORTED,     
    };
    
    /* We only make changes when we're not in preview mode. */
    if (! isPreviewEnabled()) {
        int nkeys = sizeof(keys) / sizeof(keys[0]);
        for (int i = 0; i < nkeys ; i++) {
            const char *valstr = params.get(keys[i]);
            LOGD("Key %d: %s => %s", i, keys[i], valstr);
            if (valstr != NULL)
                mParameters.set(keys[i], valstr);
        }
    }
    
    return 0;
}

/**
   @brief Returns true if preview is enabled

   @param none
   @return true  If preview is running currently
           false If preview has been stopped

 */
 
/**************************************************************************
 *  Preview
 *************************************************************************/
bool AbstractedCamera::isPreviewEnabled()
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
status_t AbstractedCamera::setPreviewWindow(struct preview_stream_ops *window)
{
    LOG_FUNCTION_NAME;
    status_t ret = NO_ERROR;
    mSetPreviewWindowCalled = true;

    /* If we are passed a NULL window pointer then we are being asked to
     * reset, so destory the existing window and free our current display
     * adapter.
     */
    if (NULL == window) {
        LOGD("NULL window passed, resetting display.");
/*
        if (mDisplayAdapter.get() != NULL) {
            mDisplayAdapter.clear();
        }
*/
        mSetPreviewWindowCalled = false;
        return NO_ERROR;
    }
    
    /* If we already have a display adapter then we don't need to do
     * anything as we can configure the display within the display adapter
     * to allow for changes of dimensions, etc 
     */
/*
    if(NULL == mDisplayAdapter.get()) {
        mDisplayAdapter.clear();
        LOGE("Unable to allocate a display adapter.");
        return NO_MEMORY;
    }
    status_t ret = mDisplayAdapter->initialize();
    if (ret != NO_ERROR) {
        mDisplayAdapter.clear();
        LOGE("Initialisation of display adapter failed.");
        return ret;
    }
*/
    /* The display adapter interfaces with the surface to get the required
     * memory, which will be filled by the camera adapter. To allow the
     * display adapter to know where to send the frame details, we need to
     * set the frame provider here.
     */
//    mDisplayAdapter->setCameraAdapter(&mAdapter);

    /* Errors that occur in the display adapter have to be propogated back
     * to the application, which will be done via the callbacks that have
     * been configured. As these are controlled via the callback notifier,
     * set the reference here.
     */
//    mDisplayAdapter->setErrorHandler(mAppCallbackNotifier.get());

    /* The newly created display adapter will use the window callbacks that
     * were passed to this function, so set them here.
     */
//    ret = mDisplayAdapter->setPreviewWindow(window);
    if (NO_ERROR != ret) {
        LOGE("Failed to set the preview window for display adapter. Error: %d", ret);
    }
    
    /* If we have been asked to start the preview already, then we are now
     * in a position to display something, so start the preview.
     */
    if (mPreviewStartInProgress) {
        LOGD("Preview start is in progress, so starting preview.");
        // Start the preview since the window is now available
        ret = startPreview();
    }
    
    return ret;
}

/**
   @brief Start preview mode.

   @param none
   @return NO_ERROR Camera switched to VF mode
   @todo Update function header with the different errors that are possible

 */
status_t AbstractedCamera::startPreview()
{
    LOGD("%s()", __FUNCTION__);

    if (mPreviewEnabled){
      LOGD("Preview already running");
      return ALREADY_EXISTS;
    }

    /* Update the camera adapter with parameters we'll use... */

    if ((mPreviewStartInProgress == false) && (mDisplayPaused == false)) {
        /* We need to set the preview sizes here... */
    }
    
    unsigned int required_buffer_count = 4;
    unsigned int max_queueble_buffers = 4;

    if (!mSetPreviewWindowCalled) { // || (mDisplayAdapter.get() == NULL)) {
        LOGD("Preview not started. Preview in progress flag set");
        mPreviewStartInProgress = true;
        /* Start the camera adapter */
        return NO_ERROR;
    }

    ///Allocate the preview buffers
    status_t ret = allocPreviewBuffers(mPreviewWidth, mPreviewHeight, 
                                       mParameters.getPreviewFormat(), 
                                       required_buffer_count, 
                                       max_queueble_buffers);

/*
    BuffersDescriptor desc;
    desc.mBuffers = mPreviewBufs;
    desc.mOffsets = mPreviewOffsets;
    desc.mFd = mPreviewFd;
    desc.mLength = mPreviewLength;
    desc.mCount = ( size_t ) required_buffer_count;
    desc.mMaxQueueable = (size_t) max_queueble_buffers;

    ret = mAdapter.setBuffers(CameraFrame::PREVIEW_FRAME_SYNC, &desc);
    LOGD("setBuffers = %d", ret);
*/    
    /* lie about what we've done for now... */
    mPreviewEnabled = true;
    mPreviewStartInProgress = false;
    return ret;
}

void AbstractedCamera::stopPreview()
{
}

status_t AbstractedCamera::allocPreviewBuffers(int width, int height, 
                                        const char* previewFormat,
                                        unsigned int buffercount, 
                                        unsigned int &max_queueable)
{
    status_t ret = NO_ERROR;
    LOG_FUNCTION_NAME;

    /* We cannot allocate memory if we don't have a display adapter. */
/*
    if(mDisplayAdapter.get() == NULL) {
        LOGD("There is no display adapter set, unable to allocate buffers");
        return NO_MEMORY;
    }

    if(!mPreviewBufs) {
        mPreviewLength = 0;
        mPreviewBufs = (int32_t *) mDisplayAdapter->allocateBuffers(width, height,
                                                                    previewFormat,
                                                                    mPreviewLength,
                                                                    buffercount);

	    if (NULL == mPreviewBufs ) {
            LOGE("Couldn't allocate preview buffers");
            return NO_MEMORY;
        }

        mPreviewOffsets = (uint32_t *) mDisplayAdapter->getOffsets();
        if (NULL == mPreviewOffsets) {
            LOGE("Buffer mapping failed");
            return BAD_VALUE;
        }

        mPreviewFd = mDisplayAdapter->getFd();
        if (-1 == mPreviewFd) {
            LOGE("Invalid handle returned from display adapter");
            return BAD_VALUE;
        }

        mPreviewBufProvider = (BufferProvider*) mDisplayAdapter.get();
        ret = mDisplayAdapter->maxQueueableBuffers(max_queueable);
        if (ret != NO_ERROR) {
            return ret;
        }
    }
*/
    LOG_FUNCTION_NAME_EXIT;
    return ret;
}


int AbstractedCamera::beginAutoFocusThread(void *cookie)
{
    LOG_FUNCTION_NAME;
//    CameraHardwareStub *c = (CameraHardwareStub *)cookie;
//    return c->autoFocusThread();
    return NO_ERROR;
}

int AbstractedCamera::autoFocusThread()
{
    LOG_FUNCTION_NAME;
//    if (mMsgEnabled & CAMERA_MSG_FOCUS)
//        mNotifyCb(CAMERA_MSG_FOCUS, true, 0, mCallbackCookie);
    return NO_ERROR;
}

status_t AbstractedCamera::autoFocus()
{
    LOG_FUNCTION_NAME;
//    Mutex::Autolock lock(mLock);
//    if (createThread(beginAutoFocusThread, this) == false)
//        return UNKNOWN_ERROR;
    return NO_ERROR;
}

status_t AbstractedCamera::cancelAutoFocus()
{
    LOG_FUNCTION_NAME;
    return NO_ERROR;
}

status_t AbstractedCamera::takePicture()
{
    return NO_ERROR;
}

status_t AbstractedCamera::cancelPicture()
{
    return NO_ERROR;
}

void AbstractedCamera::releaseCamera()
{
    LOGV("%s() - not implemented yet", __FUNCTION__);
}

status_t AbstractedCamera::dumpCamera(int fd)
{
    LOGD("%s(%d) - not implemented yet ", __FUNCTION__, fd);
    return NO_ERROR;
}

status_t AbstractedCamera::sendCommand(int32_t cmd, int32_t arg1, int32_t arg2)
{
    LOGV("%s(%d, %d, %d) - not implemented yet ", __FUNCTION__, cmd, arg1, arg2);
    return NO_ERROR;
}

} /* ! namespace android */

