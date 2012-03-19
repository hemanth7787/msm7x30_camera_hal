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

#include <linux/videodev2.h>

#include "Camera.h"
#include "ParameterHelper.h"
#include "Debug.h"

namespace android {

Camera::Camera(int cameraId, struct hw_module_t* module) 
       : mPreviewWindow(),
         mCallbackNotifier(),
         mCameraID(cameraId)
{
    LOG_FUNCTION_NAME
  
    /* Set the fields needed for the camera_device structure. */
    common.tag = HARDWARE_DEVICE_TAG;
    common.version = 0;
    common.module = module;
    common.close = Camera::closeDevice;
    /* camera_device fields. */
    ops = &mDeviceOps;
    priv = this;
}

Camera::~Camera() 
{
    LOG_FUNCTION_NAME;
}

/* This is called from CameraFactory::cameraDeviceOpen. The Initialization
 * functions will have been called prior to this being called.
 */
status_t Camera::connectCamera(hw_device_t** device)
{
    LOG_FUNCTION_NAME

    status_t res = EINVAL;
    CameraDevice* const camera_dev = getCameraDevice();
    LOGE_IF(camera_dev == NULL, "%s: No camera device instance.", __FUNCTION__);

    if (camera_dev != NULL) {
        /* Connect to the camera device. */
        res = camera_dev->connectDevice();
        if (res == NO_ERROR) {
            *device = &common;
        }
    }
    return -res;
}

status_t Camera::closeCamera()
{
    LOG_FUNCTION_NAME
    return cleanupCamera();
}

status_t Camera::getCameraInfo(struct camera_info* info)
{
    LOG_FUNCTION_NAME

    const char* valstr = NULL;

    valstr = mParameters.get(Camera::KEY_FACING);
    if (valstr != NULL) {
        if (strcmp(valstr, Camera::FACING_FRONT) == 0) {
            info->facing = CAMERA_FACING_FRONT;
        }
        else if (strcmp(valstr, Camera::FACING_BACK) == 0) {
            info->facing = CAMERA_FACING_BACK;
        }
    } else {
        info->facing = CAMERA_FACING_BACK;
    }

    valstr = mParameters.get(Camera::KEY_ORIENTATION);
    if (valstr != NULL) {
        info->orientation = atoi(valstr);
    } else {
        info->orientation = 0;
    }

    return NO_ERROR;
}

void Camera::onNextFrameAvailable(const void* frame, nsecs_t timestamp,
                                  CameraDevice* camera_dev)
{
    LOG_FUNCTION_NAME
    
    /* Notify the preview window first. */
    mPreviewWindow.onNextFrameAvailable(frame, timestamp, camera_dev);

    /* Notify callback notifier next. */
    mCallbackNotifier.onNextFrameAvailable(frame, timestamp, camera_dev);
}

/**************************************************************************
 *  Callbacks & Messages
 *************************************************************************/
void Camera::setCallbacks(camera_notify_callback notify_cb,
                             camera_data_callback data_cb,
                             camera_data_timestamp_callback data_cb_timestamp,
                             camera_request_memory get_memory,
                             void* user)
{
    mCallbackNotifier.setCallbacks(notify_cb, data_cb, data_cb_timestamp,
                                    get_memory, user);
}

void Camera::enableMsgType(int32_t msg_type)
{
    mCallbackNotifier.enableMessage(msg_type);
}

void Camera::disableMsgType(int32_t msg_type)
{
    mCallbackNotifier.disableMessage(msg_type);
}

int Camera::isMsgTypeEnabled(int32_t msg_type)
{
    return mCallbackNotifier.isMessageEnabled(msg_type);
}

void Camera::onCameraDeviceError(int err)
{
    /* Errors are reported through the callback notifier */
    mCallbackNotifier.onCameraDeviceError(err);
}

/**************************************************************************
 *  Metadata
 *************************************************************************/
status_t Camera::storeMetaDataInBuffers(int enable)
{
    /* Callback should return a negative errno. */
    return -mCallbackNotifier.storeMetaDataInBuffers(enable);
}

/**************************************************************************
 *  Recording (aka video)
 *************************************************************************/
status_t Camera::startRecording()
{
    /* Callback should return a negative errno. */
    return -mCallbackNotifier.enableVideoRecording(mParameters.getPreviewFrameRate());
}

void Camera::stopRecording()
{
    mCallbackNotifier.disableVideoRecording();
}

int Camera::isRecordingEnabled()
{
    return mCallbackNotifier.isVideoRecordingEnabled();
}

void Camera::releaseRecordingFrame(const void* opaque)
{
    mCallbackNotifier.releaseRecordingFrame(opaque);
}

/*************************************************************************
 *  Parameters
 *************************************************************************/
static char lNoParam = '\0';
char* Camera::getParameters()
{
    String8 params(mParameters.flatten());

    /* The string we return is freed by a call to putParameters() */
    char *params_string = (char*) malloc(sizeof(char) * (params.length()+1));
    if (NULL == params_string) {
        LOGE("%s(): Unable to allocate memory for return string.", 
             __FUNCTION__);
        return &lNoParam;
    }
    strncpy(params_string, params.string(), params.length() + 1);
    return params_string;
}

/* Free the parameter string we suppplied via getParameters() */
void Camera::putParameters(char* params)
{
    /* This method simply frees parameters allocated in getParameters(). */
    if (params != NULL && params != &lNoParam) {
        free(params);
    }
}    

int Camera::setParameters(const char* parameters)
{
    CameraParameters params;

    String8 str_params(parameters);
    params.unflatten(str_params);

    return setParameters(params);
}

int Camera::setParameters(const CameraParameters& params)
{
    LOG_FUNCTION_NAME

    const char *keys[] = {
        CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES,
        CameraParameters::KEY_PREVIEW_FPS_RANGE,
        CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE,
        CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS,
        CameraParameters::KEY_PREVIEW_FRAME_RATE,
        CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES,
        CameraParameters::KEY_SUPPORTED_PICTURE_SIZES,
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

/*************************************************************************
 *  Preview
 *************************************************************************/

int Camera::isPreviewEnabled()
{
    LOGD("%s() -> %d", __FUNCTION__, mPreviewWindow.isPreviewEnabled());
    return mPreviewWindow.isPreviewEnabled();
}

status_t Camera::setPreviewWindow(struct preview_stream_ops *window)
{
    return -mPreviewWindow.setPreviewWindow(window,
                                        mParameters.getPreviewFrameRate());
}

status_t Camera::startPreview()
{
    LOG_FUNCTION_NAME
    return -doStartPreview();
}

status_t Camera::doStartPreview()
{
    LOG_FUNCTION_NAME

    CameraDevice* camera_dev = getCameraDevice();
    if (camera_dev->isStarted()) {
        camera_dev->stopDeliveringFrames();
        camera_dev->stopDevice();
    }

    status_t res = mPreviewWindow.startPreview();
    if (res != NO_ERROR) {
        return res;
    }

    /* Make sure camera device is connected. */
    if (!camera_dev->isConnected()) {
        res = camera_dev->connectDevice();
        if (res != NO_ERROR) {
            mPreviewWindow.stopPreview();
            return res;
        }
    }

    int width, height;
    /* Lets see what should we use for frame width, and height. */
    if (mParameters.get(CameraParameters::KEY_VIDEO_SIZE) != NULL) {
        mParameters.getVideoSize(&width, &height);
    } else {
        mParameters.getPreviewSize(&width, &height);
    }
    /* Lets see what should we use for the frame pixel format. Note that there
     * are two parameters that define pixel formats for frames sent to the
     * application via notification callbacks:
     * - KEY_VIDEO_FRAME_FORMAT, that is used when recording video, and
     * - KEY_PREVIEW_FORMAT, that is used for preview frame notification.
     * We choose one or the other, depending on "recording-hint" property set by
     * the framework that indicating intention: video, or preview. */
    const char* pix_fmt = NULL;
    const char* is_video = mParameters.get(Camera::KEY_RECORDING_HINT);
    if (is_video == NULL) {
        is_video = CameraParameters::FALSE;
    }
    if (strcmp(is_video, CameraParameters::TRUE) == 0) {
        /* Video recording is requested. Lets see if video frame format is set. */
        pix_fmt = mParameters.get(CameraParameters::KEY_VIDEO_FRAME_FORMAT);
    }
    /* If this was not video recording, or video frame format is not set, lets
     * use preview pixel format for the main framebuffer. */
    if (pix_fmt == NULL) {
        pix_fmt = mParameters.getPreviewFormat();
    }
    if (pix_fmt == NULL) {
        LOGE("%s: Unable to obtain video format", __FUNCTION__);
        mPreviewWindow.stopPreview();
        return EINVAL;
    }

    /* Convert framework's pixel format to the FOURCC one. */
    uint32_t org_fmt;
    if (strcmp(pix_fmt, CameraParameters::PIXEL_FORMAT_YUV420P) == 0) {
        org_fmt = V4L2_PIX_FMT_YUV420;
    } else if (strcmp(pix_fmt, CameraParameters::PIXEL_FORMAT_RGBA8888) == 0) {
        org_fmt = V4L2_PIX_FMT_RGB32;
    } else if (strcmp(pix_fmt, CameraParameters::PIXEL_FORMAT_YUV420SP) == 0) {
        org_fmt = V4L2_PIX_FMT_NV21;
    } else {
        LOGE("%s: Unsupported pixel format %s", __FUNCTION__, pix_fmt);
        mPreviewWindow.stopPreview();
        return EINVAL;
    }
    LOGD("Starting camera: %dx%d -> %.4s(%s)",
         width, height, reinterpret_cast<const char*>(&org_fmt), pix_fmt);
    res = camera_dev->startDevice(width, height, org_fmt);
    if (res != NO_ERROR) {
        mPreviewWindow.stopPreview();
        return res;
    }

    res = camera_dev->startDeliveringFrames(false);
    if (res != NO_ERROR) {
        camera_dev->stopDevice();
        mPreviewWindow.stopPreview();
    }

    return res;
}

void Camera::stopPreview()
{
    doStopPreview();
}

status_t Camera::doStopPreview()
{
    LOG_FUNCTION_NAME
    status_t res = NO_ERROR;
    if (mPreviewWindow.isPreviewEnabled()) {
        /* Stop the camera. */
        if (getCameraDevice()->isStarted()) {
            getCameraDevice()->stopDeliveringFrames();
            res = getCameraDevice()->stopDevice();
        }

        if (res == NO_ERROR) {
            /* Disable preview as well. */
            mPreviewWindow.stopPreview();
        }
    }

    return NO_ERROR;
}

/*************************************************************************
 *  Auto Focus
 *************************************************************************/
status_t Camera::setAutoFocus()
{
    LOGV("%s", __FUNCTION__);

    /* TODO: Future enhancements. */
    return NO_ERROR;
}

status_t Camera::cancelAutoFocus()
{
    LOG_FUNCTION_NAME;
    return NO_ERROR;
}

/*************************************************************************
 *  Pictures
 *************************************************************************/

status_t Camera::takePicture()
{
    LOG_FUNCTION_NAME
    
    status_t res;
    int width, height;
    uint32_t org_fmt;

    /* Collect frame info for the picture. */
    mParameters.getPictureSize(&width, &height);
    const char* pix_fmt = mParameters.getPictureFormat();
    if (strcmp(pix_fmt, CameraParameters::PIXEL_FORMAT_YUV420P) == 0) {
        org_fmt = V4L2_PIX_FMT_YUV420;
    } else if (strcmp(pix_fmt, CameraParameters::PIXEL_FORMAT_RGBA8888) == 0) {
        org_fmt = V4L2_PIX_FMT_RGB32;
    } else if (strcmp(pix_fmt, CameraParameters::PIXEL_FORMAT_YUV420SP) == 0) {
        org_fmt = V4L2_PIX_FMT_NV21;
    } else if (strcmp(pix_fmt, CameraParameters::PIXEL_FORMAT_JPEG) == 0) {
        /* We only have JPEG converted for NV21 format. */
        org_fmt = V4L2_PIX_FMT_NV21;
    } else {
        LOGE("%s: Unsupported pixel format %s", __FUNCTION__, pix_fmt);
        return EINVAL;
    }
    /* Get JPEG quality. */
    int jpeg_quality = mParameters.getInt(CameraParameters::KEY_JPEG_QUALITY);
    if (jpeg_quality <= 0) {
        jpeg_quality = 90;  /* Fall back to default. */
    }

    /*
     * Make sure preview is not running, and device is stopped before taking
     * picture.
     */

    const bool preview_on = mPreviewWindow.isPreviewEnabled();
    if (preview_on) {
        doStopPreview();
    }

    /* Camera device should have been stopped when the shutter message has been
     * enabled. */
    CameraDevice* const camera_dev = getCameraDevice();
    if (camera_dev->isStarted()) {
        LOGW("%s: Camera device is started", __FUNCTION__);
        camera_dev->stopDeliveringFrames();
        camera_dev->stopDevice();
    }

    /*
     * Take the picture now.
     */

    /* Start camera device for the picture frame. */
    LOGD("Starting camera for picture: %.4s(%s)[%dx%d]",
         reinterpret_cast<const char*>(&org_fmt), pix_fmt, width, height);
    res = camera_dev->startDevice(width, height, org_fmt);
    if (res != NO_ERROR) {
        if (preview_on) {
            doStartPreview();
        }
        return res;
    }

    /* Deliver one frame only. */
    mCallbackNotifier.setJpegQuality(jpeg_quality);
    mCallbackNotifier.setTakingPicture(true);
    res = camera_dev->startDeliveringFrames(true);
    if (res != NO_ERROR) {
        mCallbackNotifier.setTakingPicture(false);
        if (preview_on) {
            doStartPreview();
        }
    }
    return res;
}

status_t Camera::cancelPicture()
{
    return NO_ERROR;
}

/*************************************************************************
 *  Cleanup
 *************************************************************************/

status_t Camera::cleanupCamera()
{
    status_t res = NO_ERROR;

    /* If preview is running - stop it. */
    res = doStopPreview();
    if (res != NO_ERROR) {
        return -res;
    }

    /* Stop and disconnect the camera device. */
    CameraDevice* const camera_dev = getCameraDevice();
    if (camera_dev != NULL) {
        if (camera_dev->isStarted()) {
            camera_dev->stopDeliveringFrames();
            res = camera_dev->stopDevice();
            if (res != NO_ERROR) {
                return -res;
            }
        }
        if (camera_dev->isConnected()) {
            res = camera_dev->disconnectDevice();
            if (res != NO_ERROR) {
                return -res;
            }
        }
    }

    mCallbackNotifier.cleanupCBNotifier();

    return NO_ERROR;
}

void Camera::releaseCamera()
{
    LOG_FUNCTION_NAME
    cleanupCamera();
}

status_t Camera::dumpCamera(int fd)
{
    LOGD("%s(%d) - not implemented yet ", __FUNCTION__, fd);
    return NO_ERROR;
}

status_t Camera::sendCommand(int32_t cmd, int32_t arg1, int32_t arg2)
{
    LOGV("%s(%d, %d, %d) - not implemented yet ", __FUNCTION__, cmd, arg1, arg2);
    return NO_ERROR;
}

/*************************************************************************
 * Camera API callbacks as defined by camera_device_ops structure.
 *
 * Callbacks here simply dispatch the calls to an appropriate method inside
 * Camera instance, defined by the 'dev' parameter.
 *************************************************************************/

int Camera::set_preview_window(struct camera_device* dev,
                                       struct preview_stream_ops* window)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->setPreviewWindow(window);
}

void Camera::set_callbacks(
        struct camera_device* dev,
        camera_notify_callback notify_cb,
        camera_data_callback data_cb,
        camera_data_timestamp_callback data_cb_timestamp,
        camera_request_memory get_memory,
        void* user)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return;
    }
    ec->setCallbacks(notify_cb, data_cb, data_cb_timestamp, get_memory, user);
}

void Camera::enable_msg_type(struct camera_device* dev, int32_t msg_type)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return;
    }
    ec->enableMsgType(msg_type);
}

void Camera::disable_msg_type(struct camera_device* dev, int32_t msg_type)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return;
    }
    ec->disableMsgType(msg_type);
}

int Camera::msg_type_enabled(struct camera_device* dev, int32_t msg_type)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->isMsgTypeEnabled(msg_type);
}

int Camera::start_preview(struct camera_device* dev)
{
    LOG_FUNCTION_NAME
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->startPreview();
}

void Camera::stop_preview(struct camera_device* dev)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return;
    }
    ec->stopPreview();
}

int Camera::preview_enabled(struct camera_device* dev)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->isPreviewEnabled();
}

int Camera::store_meta_data_in_buffers(struct camera_device* dev,
                                               int enable)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->storeMetaDataInBuffers(enable);
}

int Camera::start_recording(struct camera_device* dev)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->startRecording();
}

void Camera::stop_recording(struct camera_device* dev)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return;
    }
    ec->stopRecording();
}

int Camera::recording_enabled(struct camera_device* dev)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->isRecordingEnabled();
}

void Camera::release_recording_frame(struct camera_device* dev,
                                             const void* opaque)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return;
    }
    ec->releaseRecordingFrame(opaque);
}

int Camera::auto_focus(struct camera_device* dev)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->setAutoFocus();
}

int Camera::cancel_auto_focus(struct camera_device* dev)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->cancelAutoFocus();
}

int Camera::take_picture(struct camera_device* dev)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->takePicture();
}

int Camera::cancel_picture(struct camera_device* dev)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->cancelPicture();
}

int Camera::set_parameters(struct camera_device* dev, const char* parms)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->setParameters(parms);
}

char* Camera::get_parameters(struct camera_device* dev)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return NULL;
    }
    return ec->getParameters();
}

void Camera::put_parameters(struct camera_device* dev, char* params)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return;
    }
    ec->putParameters(params);
}

int Camera::send_command(struct camera_device* dev,
                                 int32_t cmd,
                                 int32_t arg1,
                                 int32_t arg2)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->sendCommand(cmd, arg1, arg2);
}

void Camera::release(struct camera_device* dev)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return;
    }
    ec->releaseCamera();
}

int Camera::dump(struct camera_device* dev, int fd)
{
    Camera* ec = reinterpret_cast<Camera*>(dev->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->dumpCamera(fd);
}

int Camera::closeDevice(struct hw_device_t* device)
{
    Camera* ec = reinterpret_cast<Camera*>(reinterpret_cast<struct camera_device*>(device)->priv);
    if (ec == NULL) {
        LOGE("%s: Unexpected NULL camera device", __FUNCTION__);
        return -EINVAL;
    }
    return ec->closeCamera();
}

/*************************************************************************
 * Static initializer for the camera callback API
 *************************************************************************/
camera_device_ops_t Camera::mDeviceOps = {
    Camera::set_preview_window,
    Camera::set_callbacks,
    Camera::enable_msg_type,
    Camera::disable_msg_type,
    Camera::msg_type_enabled,
    Camera::start_preview,
    Camera::stop_preview,
    Camera::preview_enabled,
    Camera::store_meta_data_in_buffers,
    Camera::start_recording,
    Camera::stop_recording,
    Camera::recording_enabled,
    Camera::release_recording_frame,
    Camera::auto_focus,
    Camera::cancel_auto_focus,
    Camera::take_picture,
    Camera::cancel_picture,
    Camera::set_parameters,
    Camera::get_parameters,
    Camera::put_parameters,
    Camera::send_command,
    Camera::release,
    Camera::dump
};



/****************************************************************************
 * Common keys
 ***************************************************************************/

const char Camera::KEY_FACING[]         = "prop-facing";
const char Camera::KEY_ORIENTATION[]    = "prop-orientation";
const char Camera::KEY_RECORDING_HINT[] = "recording-hint";
const char Camera::KEY_ISO[] = "iso";

/****************************************************************************
 * Common string values
 ***************************************************************************/

const char Camera::FACING_BACK[]      = "back";
const char Camera::FACING_FRONT[]     = "front";


} /* ! namespace android */

