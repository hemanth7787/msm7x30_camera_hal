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

#ifndef MSM_CAMERA_FACTORY_H
#define MSM_CAMERA_FACTORY_H

#include "CameraFactory.h"

namespace android {

class MsmCameraFactory : public CameraFactory
{
    public:
        MsmCameraFactory();
        
};

}; /* ! namespace android */

extern android::MsmCameraFactory gCameraFactory;

#endif /* ! MSM_CAMERA_FACTORY_H */
