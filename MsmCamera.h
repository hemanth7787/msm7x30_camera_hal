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

/* The MsmCamera class is a wrapper around the Camera class that allows us
 * to add some camera specific initialisation and to link an instance of
 * the MsmCameraDevice class.
 *
 * The actual device specific code lives in the the MsmCameraDevice class.
 */

#ifndef MSM_CAMERA_H
#define MSM_CAMERA_H

#include "Camera.h"
#include "MsmCameraDevice.h"

namespace android {

class MsmCamera : public Camera
{
    public:
        MsmCamera(int cameraId, struct hw_module_t *module);

        /* Destructor */
        ~MsmCamera();

        int getCameraId() const { return mCameraID; }
        
    /*********************************************************************
     * Camera virtual overrides.
     *********************************************************************/

    public:
        /* Initializes an MsmCamera instance. */
        status_t Initialize();

    /*********************************************************************
     * Camera abstract API implementation.
     *********************************************************************/

    protected:
        /* Gets the camera device used by this instance of the camera. */
        CameraDevice* getCameraDevice();

    /*********************************************************************
     * Data memebers.
     *********************************************************************/

    protected:
        /* Contained fake camera device object. */
        MsmCameraDevice    mCameraDevice;

};

}; /* ! namespace android */

#endif /* MSM_CAMERA_H */
