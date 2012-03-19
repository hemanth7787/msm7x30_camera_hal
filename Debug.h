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
 
/* todo - add checks for system defines to switch on/off debugging */

#ifndef CAMERA_DEBUG_H
#define CAMERA_DEBUG_H

#include <utils/Log.h>

#define LOG_FUNCTION_NAME       LOGD("%s() ENTER @ line %d in %s", __FUNCTION__, __LINE__, __FILE__);
#define LOG_FUNCTION_NAME_EXIT  LOGD("%s() EXIT", __FUNCTION__);


#endif /* ! CAMERA_DEBUG_H */
