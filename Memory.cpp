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


#include <utils/String8.h>

#include "Memory.h"

namespace android {

static unsigned clp2(unsigned x) {
    x = x - 1;
    x = x | (x >> 1);
    x = x | (x >> 2);
    x = x | (x >> 4);
    x = x | (x >> 8);
    x = x | (x >>16);
    return x + 1;
}

PhysicalMemoryPool::PhysicalMemoryPool(int n_allocs, int alloc_size)
{
    LOGD("PhysicalMemoryPool: %d allocations of %d bytes", n_allocs, alloc_size);
    mAllocs = n_allocs;
    mAllocSize = alloc_size;
    mAlignedSize = clp2(n_allocs * alloc_size);
#define MEMORY_POOL "/dev/pmem_adsp"
    mMasterHeap = new MemoryHeapBase(MEMORY_POOL, mAlignedSize, 0);
    LOGD("mMasterHeap -> %08x bytes @ %p", mMasterHeap->getSize(), mMasterHeap->getBase());
    mPmemHeap = new MemoryHeapPmem(mMasterHeap, 0);
    mHeapId = mPmemHeap->getHeapID();
    if (mHeapId >= 0) {
        mPmemHeap->slap();  /* make memory available */

        if (::ioctl(mHeapId, PMEM_GET_SIZE, &mSize)) {
            LOGE("ioctl(PMEM_GET_SIZE) error %s", ::strerror(errno));
            return;
        }
    }
    LOGD("ioctl(PMEM_GET_SIZE) is %ld", mSize.len);

    mBaseAddress = (uint32_t)mMasterHeap->getBase();
}

void *PhysicalMemoryPool::getBaseAddress()
{
    return (void *)(mBaseAddress);
}

bool PhysicalMemoryPool::initialized()
{ 
    return mPmemHeap != NULL && mPmemHeap->base() != MAP_FAILED;
}


} /* ! namespace android */
