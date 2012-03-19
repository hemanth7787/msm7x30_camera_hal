/*
 * Copyright (C) 2011 The Android Open Source Project
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

#ifndef CAMERA_FACTORY_H
#define CAMERA_FACTORY_H

#include "Camera.h"

namespace android {

/* CameraFactory class.
 *
 * The role of the CameraFactory class is to act as a manager for access to
 * the cameras available on the system for the HAL subsystem. When the
 * module is loaded a static instance of this class will be created. At
 * that point it will simple be constructed.
 *
 * Calls to get_number_of_cameras or get_camera_info via the HAL
 * module will return information, prompting the static instance to call
 * the find_cameras function. At this point the function will determine
 * how many cameras are available and enough information that the required
 * information can be supplied.
 */
 
class CameraFactory {
    public:
        /* Constructs CameraFactory instance.
         * In this constructor the factory will create and initialize a list of
         * emulated cameras. All errors that occur on this constructor are reported
         * via mConstructedOK data member of this class.
         */
        CameraFactory();

        /* Destructs CameraFactory instance. */
        ~CameraFactory();

    /****************************************************************************
     * Camera HAL API handlers.
     ***************************************************************************/

    public:
        /* Opens (connects to) a camera device.
         * This method is called in response to hw_module_methods_t::open callback.
         */
        int cameraDeviceOpen(int camera_id, hw_device_t** device);

        /* Gets number of cameras available. */
        int getCameraNum();

        /* Gets camera information.
         * This method is called via the camera_module_t::get_camera_info callback.
         */
        int getCameraInfo(int camera_id, struct camera_info *info);

    /****************************************************************************
     * Camera HAL API callbacks & entry points.
     ***************************************************************************/

    public:
        /* camera_module_t::get_number_of_cameras callback entry point. */
        static int get_number_of_cameras(void);

        /* camera_module_t::get_camera_info callback entry point. */
        static int get_camera_info(int camera_id, struct camera_info *info);

    public:
        /* Contains device open entry point, as required by HAL API. */
        static struct hw_module_methods_t   mCameraModuleMethods;

    private:
        /* hw_module_methods_t::open callback entry point. */
        static int device_open(const hw_module_t* module,
                               const char* name,
                               hw_device_t** device);

    /****************************************************************************
     * Private API.
     ***************************************************************************/

    private:
        /* Actually check for cameras that are available. This should
         * set mDiscoveryComplete to true after a succesful check.
         */
        void findCameras();

    /*********************************************************************
     * Data members.
     *********************************************************************/

    private:
        /* Array of available cameras. */
        Camera**    mCameras;
        /* Number of emulated cameras (including the fake one). */
        int         mCameraNum;
        /* Flag to show whether we have looked for cameras. */
        bool        mDiscoveryComplete;

};

}; /* namespace android */

extern android::CameraFactory gCameraFactory;

#endif  /* CAMERA_FACTORY_H */
