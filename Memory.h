/*
** Copyright 2012 zathrasorama@gmail.com
**
** Licensed under the Apache License, Version 2.0 (the "License"); 
** you may not use this file except in compliance with the License. 
** You may obtain a copy of the License at 
**
**     http://www.apache.org/licenses/LICENSE-2.0 
**
** Unless required by applicable law or agreed to in writing, software 
** distributed under the License is distributed on an "AS IS" BASIS, 
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
** See the License for the specific language governing permissions and 
** limitations under the License.
*/

#ifndef HAL_CAMERA_MEMORY_H
#define HAL_CAMERA_MEMORY_H

#include <binder/MemoryBase.h>
#include <binder/MemoryHeapBase.h>
#include <binder/MemoryHeapPmem.h>

extern "C" {
    #include <linux/android_pmem.h>
}

namespace android {

class PhysicalMemoryPool {
    public:
        PhysicalMemoryPool(int n_allocs, int alloc_size);

        bool initialized();
        void *getBaseAddress();
        
        int getFd() { return mHeapId; }
        
    private:
        int mHeapId;
        int mAllocs;   /* how many allocations */
        int mAllocSize;  /* size of each allocation (bytes) */        
        int mAlignedSize;  /* total page aligned size of memory rqd */
        MemoryHeapBase *mMasterHeap;
        MemoryHeapPmem *mPmemHeap;
        struct pmem_region mSize;
        uint32_t mBaseAddress;
};

}; // namespace android

#endif /* HAL_CAMERA_MEMORY_H */

