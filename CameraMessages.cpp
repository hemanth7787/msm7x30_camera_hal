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

#include <utils/Log.h>
#include "system/camera.h"

static struct {
    int type;
    const char *text;
} msg_map[] = {
    {CAMERA_MSG_ERROR,            "CAMERA_MSG_ERROR"},
    {CAMERA_MSG_SHUTTER,          "CAMERA_MSG_SHUTTER"},
    {CAMERA_MSG_FOCUS,            "CAMERA_MSG_FOCUS"},
    {CAMERA_MSG_ZOOM,             "CAMERA_MSG_ZOOM"},
    {CAMERA_MSG_PREVIEW_FRAME,    "CAMERA_MSG_PREVIEW_FRAME"},
    {CAMERA_MSG_VIDEO_FRAME,      "CAMERA_MSG_VIDEO_FRAME"},
    {CAMERA_MSG_POSTVIEW_FRAME,   "CAMERA_MSG_POSTVIEW_FRAME"},
    {CAMERA_MSG_RAW_IMAGE,        "CAMERA_MSG_RAW_IMAGE"},
    {CAMERA_MSG_COMPRESSED_IMAGE, "CAMERA_MSG_COMPRESSED_IMAGE"},
    {CAMERA_MSG_RAW_IMAGE_NOTIFY, "CAMERA_MSG_RAW_IMAGE_NOTIFY"},
    {CAMERA_MSG_PREVIEW_METADATA, "CAMERA_MSG_PREVIEW_METADATA"},
    {CAMERA_MSG_ALL_MSGS,         "CAMERA_MSG_ALL_MSGS"},
    {0x0000, "NULL"},
};

void logMessages(int32_t msgs)
{
    for (int i = 0; msg_map[i].type != 0; i++) {
        bool on = (msgs & msg_map[i].type);
        LOGD("    %-30s : %s", msg_map[i].text, on ? "ON" : "OFF");
    }
}

