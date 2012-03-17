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

/*
 * This file contains a simple class that acts a container for the functions
 * required to provide a camera_device_t structure for Android HAL.
 */

#define LOG_TAG "HalCameraDevice"

#include <fcntl.h>
#include <strings.h>
#include <stdlib.h>
#include <utils/Errors.h>

#include <hardware/camera.h>
#include <cutils/log.h>

#include "AbstractedCamera.h"

#define CHECK_VOID if (NULL == device || NULL == device->priv) return;
#define CHECK_INT  if (NULL == device || NULL == device->priv) return -EINVAL;
#define CAM_PTR    ((android::AbstractedCamera *)(device->priv))

/** Set the ANativeWindow to which preview frames are sent */
static int camera_set_preview_window(camera_device_t *device,
                                         struct preview_stream_ops *window)
{
    LOGD("%s(%p)", __FUNCTION__, window);
    CHECK_INT
    return CAM_PTR->setPreviewWindow(window);
}

/** Set the notification and data callbacks */
static void camera_set_callbacks(camera_device_t *device,
                                 camera_notify_callback notify_cb,
                                 camera_data_callback data_cb,
                                 camera_data_timestamp_callback data_ts_cb,
                                 camera_request_memory get_memory,
                                 void *user)
{
    LOGD("%s (%p, %p, %p, %p, %p)", __FUNCTION__, notify_cb, data_cb, 
                                             data_ts_cb, get_memory, user);
    CHECK_VOID
    return CAM_PTR->setCallbacks(notify_cb, data_cb, data_ts_cb, get_memory, user);
}

/**
 * Enable a message, or set of messages.
 */
static void camera_enable_msg_type(camera_device_t *device, int32_t msg_type)
{
    LOGD("%s(%08x)", __FUNCTION__, msg_type);
    logMessages(msg_type);
    android::HalCamera *cam = (android::HalCamera *)device->priv;
}

/**
 * Disable a message, or a set of messages.
 *
 * Once received a call to disableMsgType(CAMERA_MSG_VIDEO_FRAME), camera HAL
 * should not rely on its client to call releaseRecordingFrame() to release
 * video recording frames sent out by the cameral HAL before and after the
 * disableMsgType(CAMERA_MSG_VIDEO_FRAME) call. Camera HAL clients must not
 * modify/access any video recording frame after calling
 * disableMsgType(CAMERA_MSG_VIDEO_FRAME).
 */
static void camera_disable_msg_type(camera_device_t *device, int32_t msg_type)
{
    LOGD("%s(%08x)", __FUNCTION__, msg_type);
    logMessages(msg_type);
    android::HalCamera *cam = (android::HalCamera *)device->priv;
}

/*
 * Query whether a message, or a set of messages, is enabled.
 * Note that this is operates as an AND, if any of the messages
 * queried are off, this will return false.
 */
static int camera_msg_type_enabled(camera_device_t *device, int32_t msg_type)
{
    LOGD("%s", __FUNCTION__);

    if(!device)
        return false;
    android::HalCamera *cam = (android::HalCamera *)device->priv;
    return false;
}

static int camera_start_preview(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    if (NULL == device || NULL == device->priv)
        return -EINVAL;
    android::HalCamera *cam = (android::HalCamera *)device->priv;
    return cam->startPreview();
}

static void camera_stop_preview(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);

    if(!device)
        return;
}

/* Returns whether the camera preview is currently enabled. */
static int camera_preview_enabled(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    if (NULL == device || NULL == device->priv)
        return -EINVAL;
    android::HalCamera *cam = (android::HalCamera *)device->priv;
    return cam->previewEnabled();
}

static int camera_store_meta_data_in_buffers(camera_device_t *device, int enable)
{
    int rv = -EINVAL;

    LOGD("%s", __FUNCTION__);

    if(!device)
        return rv;

    return rv;
    //return enable ? android::INVALID_OPERATION: android::OK;
}

static int camera_start_recording(camera_device_t *device)
{
    int rv = -EINVAL;

    LOGD("%s", __FUNCTION__);

    if(!device)
        return rv;

    return rv;
}

static void camera_stop_recording(camera_device_t *device)
{
    LOGD("%s", __FUNCTION__);

    if(!device)
        return;

}

static int camera_recording_enabled(camera_device_t *device)
{
    int rv = -EINVAL;

    LOGD("%s", __FUNCTION__);

    if(!device)
        return rv;

    return rv;
}

static void camera_release_recording_frame(camera_device_t *device,
                                                        const void *opaque)
{
    LOGD("%s", __FUNCTION__);

    if(!device)
        return;

}

static int camera_auto_focus(camera_device_t *device)
{
    LOGD("%s", __FUNCTION__);
    if (NULL == device || NULL == device->priv)
        return -EINVAL;
    android::HalCamera *cam = (android::HalCamera *)device->priv;
    return cam->autoFocus();
}

static int camera_cancel_auto_focus(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    if (NULL == device || NULL == device->priv)
        return -EINVAL;
    android::HalCamera *cam = (android::HalCamera *)device->priv;
    return cam->cancelAutoFocus();
}

static int camera_take_picture(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    if (NULL == device || NULL == device->priv)
        return -EINVAL;
    android::HalCamera *cam = (android::HalCamera *)device->priv;
    return -EINVAL;
}

static int camera_cancel_picture(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    if (NULL == device || NULL == device->priv)
        return -EINVAL;
    android::HalCamera *cam = (android::HalCamera *)device->priv;
    return android::NO_ERROR;
}

static int camera_set_parameters(camera_device_t *device, const char *params)
{
    LOGD("%s (%s)", __FUNCTION__, params);
    if (NULL == device || NULL == device->priv)
        return -EINVAL;
    android::HalCamera *cam = (android::HalCamera *)device->priv;
    return cam->setParameters(params);
}

static char *camera_get_parameters(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);

    if(NULL == device)
        return NULL; /* ??? Should we return an empty string? */

    android::HalCamera *cam = (android::HalCamera *)device->priv;
    LOGD("params = %s", cam->getParameters());
    return cam->getParameters();
}

static void camera_put_parameters(camera_device_t *device, char *params)
{
    LOGD("%s(%s)", __FUNCTION__, params);
    /* ??? The ti CameraHal module has this as a no-op? */
    free(params);
}

static int camera_send_command(camera_device_t *device, int32_t cmd, 
                                                int32_t arg1, int32_t arg2)
{
    int rv = -EINVAL;

    LOGD("%s(%i, %i, %i)", __FUNCTION__, cmd, arg1, arg2);

    if (NULL == device)
        return -EINVAL;

    return rv;
}

static void camera_release(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    if (NULL == device || NULL == device->priv)
        return;
}

static int camera_dump(camera_device_t *device, int fd)
{
    int rv = -EINVAL;

    if(!device)
        return rv;

    return rv;
}

static int camera_device_close(hw_device_t* device)
{
    LOGD("%s()", __FUNCTION__);

    if (!device)
        return -EINVAL;
    camera_device_t * dev = (camera_device_t *) device;

    if (dev->ops)
        free(dev->ops);
    if (dev->priv)
        delete (android::HalCamera *)(dev->priv);

    free(dev);
    return 0;
}

int camera_device_open(const hw_module_t* module, const char* name,
                       hw_device_t** device)
{
    int cameraid = atoi(name);
    camera_device_t *cameraDev = NULL;
    camera_device_ops_t *cameraOps = NULL;
       
    LOGD("%s(%s)", __FUNCTION__, name);
    *device = NULL;

    /* ??? Should we check that the requested camera exists? */
    cameraDev = (camera_device_t *)malloc(sizeof(*cameraDev));
    if (!cameraDev) {
        LOGE("Failed to allocate memory for camera_device_t structure");
        return -ENOMEM;
    }
    cameraOps = (camera_device_ops_t *)malloc(sizeof(*cameraOps));
    if (!cameraOps) {
        LOGE("Failed to allocate memory for cameraOps structure");
        free(cameraDev);
        return -ENOMEM;
    }

    /* Set the correct information in the device structure. */
    memset(cameraDev, 0, sizeof(*cameraDev));
    cameraDev->common.tag = HARDWARE_DEVICE_TAG;
    cameraDev->common.version = 0;
    cameraDev->common.module = (hw_module_t *)(module);
    cameraDev->common.close = camera_device_close;

    /* Set the function pointers for the camera operations */
    memset(cameraOps, 0, sizeof(*cameraOps));
    cameraOps->set_preview_window = camera_set_preview_window;
    cameraOps->set_callbacks = camera_set_callbacks;
    cameraOps->enable_msg_type = camera_enable_msg_type;
    cameraOps->disable_msg_type = camera_disable_msg_type;
    cameraOps->msg_type_enabled = camera_msg_type_enabled;
    cameraOps->start_preview = camera_start_preview;
    cameraOps->stop_preview = camera_stop_preview;
    cameraOps->preview_enabled = camera_preview_enabled;
    cameraOps->store_meta_data_in_buffers = camera_store_meta_data_in_buffers;
    cameraOps->start_recording = camera_start_recording;
    cameraOps->stop_recording = camera_stop_recording;
    cameraOps->recording_enabled = camera_recording_enabled;
    cameraOps->release_recording_frame = camera_release_recording_frame;
    cameraOps->auto_focus = camera_auto_focus;
    cameraOps->cancel_auto_focus = camera_cancel_auto_focus;
    cameraOps->take_picture = camera_take_picture;
    cameraOps->cancel_picture = camera_cancel_picture;
    cameraOps->set_parameters = camera_set_parameters;
    cameraOps->get_parameters = camera_get_parameters;
    cameraOps->put_parameters = camera_put_parameters;
    cameraOps->send_command = camera_send_command;
    cameraOps->release = camera_release;
    cameraOps->dump = camera_dump;

    cameraDev->ops = cameraOps;

    cameraDev->priv = new android::HalCamera(cameraid);
    
    *device = &cameraDev->common;
    return 0;
}

