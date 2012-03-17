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

#include "ParameterHelper.h"

#define PARAM_SEP_CHAR ','
#define MAX_VALUE_LENGTH  1024

namespace android {

/* For the specified key, create a string from the array of resolution
 * entries and set it into the supplied CameraParameter object.
 */
void ParameterHelper::parameterSizes(const char *key, 
                                     size_entry_t *entries, int nentries)
{
    if (NULL == key || NULL == entries)
        return;

    char buffer[MAX_VALUE_LENGTH];   
    memset(&buffer, 0, MAX_VALUE_LENGTH);

    for (int i = 0; i < nentries; i++) {
        strncat(buffer, entries[i].str, MAX_VALUE_LENGTH - 1);
        if (i < nentries - 1)
            strncat(buffer, ",", MAX_VALUE_LENGTH - 1);
    }
    mParams->set(key, buffer);
}

void ParameterHelper::setStringList(const char *key, const char **list, 
                                    int nentries)
{
    if (NULL == key || NULL == list)
        return;

    char buffer[MAX_VALUE_LENGTH];   
    memset(&buffer, 0, MAX_VALUE_LENGTH);

    for (int i = 0; i < nentries; i++) {
        if (NULL == list[i])
            continue;
        strncat(buffer, list[i], MAX_VALUE_LENGTH - 1);
        if (i < nentries - 1)
            strncat(buffer, ",", MAX_VALUE_LENGTH - 1);
    }
    mParams->set(key, buffer);
}

void ParameterHelper::setPreviewSizes(size_entry_t *entries, int n)
{
    parameterSizes(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, entries, n);
}

void ParameterHelper::setPictureSizes(size_entry_t *entries, int n)
{
    parameterSizes(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES, entries, n);
}

void ParameterHelper::setPreviewSize(size_entry_t entry)
{
    mParams->setPreviewSize(entry.width, entry.height);
}

void ParameterHelper::setPictureSize(size_entry_t entry)
{
    mParams->setPictureSize(entry.width, entry.height);
}


} /* ! namespace android */

