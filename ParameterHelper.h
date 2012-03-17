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

#ifndef PARAMETER_HELPER_H
#define PARAMETER_HELPER_H

#include <camera/CameraParameters.h>

namespace android {

typedef struct {
    int width, height;  /* width and height of this size */
    const char *str;    /* string of form widthxheight */
} size_entry_t;

class ParameterHelper
{
    public:
        ParameterHelper(CameraParameters *cm) : mParams(cm) {}
        
        void parameterSizes(const char *key, 
                                     size_entry_t *entries, int nentries);
        void setStringList(const char *key, const char **list, 
                                    int nentries);
        void setPreviewSizes(size_entry_t *entries, int n);
        void setPictureSizes(size_entry_t *entries, int n);
        void setPreviewSize(size_entry_t entry);
        void setPictureSize(size_entry_t entry);
    private:
        CameraParameters *mParams;
};


} /* ! namespace android */

#endif /* ! PARAMETER_HELPER_H */
