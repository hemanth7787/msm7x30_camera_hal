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

#define LOG_TAG "AbstractedDevice"

#include <fcntl.h>
#include <strings.h>
#include <stdlib.h>
#include <utils/Errors.h>

#include <hardware/camera.h>
#include <cutils/log.h>

#include "Camera.h"

#define CHECK_VOID if (NULL == device || NULL == device->priv) return;
#define CHECK_INT  if (NULL == device || NULL == device->priv) return -EINVAL;
#define CHECK_CHAR if (NULL == device || NULL == device->priv) return NULL;
#define CAM_PTR    ((android::Camera *)(device->priv))

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
    CHECK_VOID
    CAM_PTR->enableMsgType(msg_type);
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
    CHECK_VOID
    CAM_PTR->disableMsgType(msg_type);
}

/*
 * Query whether a message, or a set of messages, is enabled.
 * Note that this is operates as an AND, if any of the messages
 * queried are off, this will return false.
 */
static int camera_msg_type_enabled(camera_device_t *device, int32_t msg_type)
{
    LOGD("%s(%08x)", __FUNCTION__, msg_type);
    CHECK_INT
    return CAM_PTR->isMsgTypeEnabled(msg_type);
}

static int camera_start_preview(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    CHECK_INT
    return CAM_PTR->startPreview();
}

static void camera_stop_preview(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    CHECK_VOID
    CAM_PTR->stopPreview();
}

/* Returns whether the camera preview is currently enabled. */
static int camera_preview_enabled(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    CHECK_INT
    return CAM_PTR->isPreviewEnabled();
}

static int camera_store_meta_data_in_buffers(camera_device_t *device, int enable)
{
    LOGD("%s(%d)", __FUNCTION__, enable);
    CHECK_INT
    return CAM_PTR->storeMetaDataInBuffers(enable);
}

static int camera_start_recording(camera_device_t *device)
{
    LOGD("%s", __FUNCTION__);
    CHECK_INT
    return CAM_PTR->startRecording();
}

static void camera_stop_recording(camera_device_t *device)
{
    LOGD("%s", __FUNCTION__);
    CHECK_VOID
    return CAM_PTR->stopRecording();
}

static int camera_recording_enabled(camera_device_t *device)
{
    LOGD("%s", __FUNCTION__);
    CHECK_INT
    return CAM_PTR->isRecordingEnabled();
}

static void camera_release_recording_frame(camera_device_t *device,
                                           const void *opaque)
{
    LOGD("%s(%p)", __FUNCTION__, opaque);
    CHECK_VOID
    CAM_PTR->releaseRecordingFrame(opaque);
}

static int camera_auto_focus(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    CHECK_INT
    return CAM_PTR->autoFocus();
}

static int camera_cancel_auto_focus(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    CHECK_INT
    return CAM_PTR->cancelAutoFocus();
}

static int camera_take_picture(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    CHECK_INT
    return CAM_PTR->takePicture();
}

static int camera_cancel_picture(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    CHECK_INT
    return CAM_PTR->cancelPicture();
}

static int camera_set_parameters(camera_device_t *device, const char *params)
{
    LOGD("%s (%s)", __FUNCTION__, params);
    CHECK_INT
    return CAM_PTR->setParameters(params);
}

static char *camera_get_parameters(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    CHECK_CHAR
    return CAM_PTR->getParameters();
}

static void camera_put_parameters(camera_device_t *device, char *params)
{
    LOGD("%s(%s)", __FUNCTION__, params);
    CHECK_VOID
    CAM_PTR->putParameters(params);
    free(params);
}

static int camera_send_command(camera_device_t *device, int32_t cmd, 
                                                int32_t arg1, int32_t arg2)
{
    LOGD("%s(%i, %i, %i)", __FUNCTION__, cmd, arg1, arg2);
    CHECK_INT
    return CAM_PTR->sendCommand(cmd, arg1, arg2);
}

static void camera_release(camera_device_t *device)
{
    LOGD("%s()", __FUNCTION__);
    CHECK_VOID
    CAM_PTR->releaseCamera();
}

static int camera_dump(camera_device_t *device, int fd)
{
    CHECK_INT
    return CAM_PTR->dumpCamera(fd);
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
        delete (android::Camera *)(dev->priv);

    free(dev);
    return 0;
}

camera_device_ops_t generic_camera_ops = {
    camera_set_preview_window,
    camera_set_callbacks,
    camera_enable_msg_type,
    camera_disable_msg_type,
    camera_msg_type_enabled,
    camera_start_preview,
    camera_stop_preview,
    camera_preview_enabled,
    camera_store_meta_data_in_buffers,
    camera_start_recording,
    camera_stop_recording,
    camera_recording_enabled,
    camera_release_recording_frame,
    camera_auto_focus,
    camera_cancel_auto_focus,
    camera_take_picture,
    camera_cancel_picture,
    camera_set_parameters,
    camera_get_parameters,
    camera_put_parameters,
    camera_send_command,
    camera_release,
    camera_dump
};

