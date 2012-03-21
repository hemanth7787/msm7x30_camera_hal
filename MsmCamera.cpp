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

#define LOG_TAG "MsmCamera"

#include "MsmCamera.h"
#include "ParameterHelper.h"
#include "Debug.h"

namespace android {

/* This is called prior to the object being made available to the system,
 * so information required for the camera_info structure should be
 * set here (if possible).
 */
status_t MsmCamera::Initialize()
{
    LOG_FUNCTION_NAME
    status_t res = mCameraDevice.Initialize();
    if (res != NO_ERROR) {
        return res;
    }

    /* ??? - is there any way we can determine this form the hardware? */
    mParameters.set(Camera::KEY_FACING, Camera::FACING_BACK);
    mParameters.set(Camera::KEY_ORIENTATION, 90);

    /* TODO - move these as they should be sensor specific and don't need
     * to be set at this point.
     */    
    size_entry_t previews[] = {
        { 640, 480, "640x480" },
        { 320, 240, "320x240" }
    };
    size_entry_t sizes[] = {
        { 3280, 2464, "3280x2464" },
        { 2048, 1536, "2048x1536" },
        { 1600, 1200, "1600x1200" },
        { 1024, 768, "1024x768" }
    };
    size_entry_t video_sizes[] = {
        { 3280, 1848, "3280x1848" },
        { 1640, 916, "1640x916" }
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

    helper.setStringList(CameraParameters::KEY_SUPPORTED_ANTIBANDING, 
                         antibanding, ARRAY_SIZE(antibanding));
    helper.setStringList(CameraParameters::KEY_SUPPORTED_EFFECTS, 
                         effects, ARRAY_SIZE(effects));
    helper.setStringList(CameraParameters::KEY_SUPPORTED_SCENE_MODES, 
                         scenes, ARRAY_SIZE(scenes));
    helper.setStringList(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE, 
                         awb, ARRAY_SIZE(awb));
    helper.setStringList(CameraParameters::KEY_SUPPORTED_FOCUS_MODES, 
                         focus, ARRAY_SIZE(focus));
    helper.setStringList(Camera::KEY_ISO, iso, ARRAY_SIZE(iso));

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
    mParameters.set(CameraParameters::KEY_ZOOM_SUPPORTED, "true");
    mParameters.set(CameraParameters::KEY_VIDEO_STABILIZATION_SUPPORTED,
                    "true");

    return res;
}

} /* ! namespace android */

