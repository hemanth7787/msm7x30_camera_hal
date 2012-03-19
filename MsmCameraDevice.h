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

/* The MsmCameraDevice class is an MSM specific subclass of CameraDevice.
 *
 * Anything that is specific to the camera sensors should go in this class.
 */

#ifndef MSM_CAMERA_DEVICE_H
#define MSM_CAMERA_DEVICE_H

#include "CameraDevice.h"
#include "Memory.h"
#include "msm_camera.h"
#include "Converters.h"

#define EFCD_ROTATE_FRAME   0

namespace android {

class MsmCamera;

class MsmCameraDevice : public CameraDevice
{
    typedef enum {
        MSM_CONFIG,     /* configuration */
        MSM_CONTROL,    /* control */
        MSM_FB          /* framebuffer */
    } msm_endpoint_e;

    public:
        MsmCameraDevice(MsmCamera *camera);

        status_t connectDevice();
        status_t disconnectDevice();
        status_t startDevice(int width, int height, uint32_t pix_fmt);
        status_t stopDevice();

    public:
        /* API for this class */
        bool configIoctl(int cmd, void *ptr);
        bool controlIoctl(int cmd, void *ptr);
        bool configCommand(int cmd, void *ptr);
        bool controlCommand(uint16_t type, void *ptr, uint16_t ptrlen);

    private:
        int openEndpoint(msm_endpoint_e which);

        /* Functions to extract information from the sensor. */
        bool getVendorId();
        bool setSensorMode(int mode);
        bool getSensorInformation();
        /* Control memory allocations for sensor operation */
        bool setupMemory();
        void removeMemory();
        /* Issue ioctl requests with suitable checks */
        bool __ioctl(int which, int cmd, void *ptr);

bool inWorkerThread();
    private:
        MsmCamera *mCamera;
               
        /* Kernel access points */
        int mControlFd;
        int mConfigFd;
        /* Sensor information */
        struct msm_camsensor_info mSensorInfo;
        int mVendorId;
        /* Sensor memory pool */
        PhysicalMemoryPool *mSetupMemory;


private:
    /* Draws a black and white checker board in the current frame buffer. */
    void drawCheckerboard();

    /* Draws a square of the given color in the current frame buffer.
     * Param:
     *  x, y - Coordinates of the top left corner of the square in the buffer.
     *  size - Size of the square's side.
     *  color - Square's color.
     */
    void drawSquare(int x, int y, int size, const YUVPixel* color);

    inline uint8_t changeExposure(uint8_t inputY) {
        return static_cast<uint8_t>(static_cast<float>(inputY) *
                                    mExposureCompensation);
    };
    
    YUVPixel    mBlackYUV;
    YUVPixel    mWhiteYUV;
    YUVPixel    mRedYUV;
    YUVPixel    mGreenYUV;
    YUVPixel    mBlueYUV;

    /* Last time the frame has been redrawn. */
    nsecs_t     mLastRedrawn;

    /*
     * Precalculated values related to U/V panes.
     */

    /* U pane inside the framebuffer. */
    uint8_t*    mFrameU;

    /* V pane inside the framebuffer. */
    uint8_t*    mFrameV;

    /* Defines byte distance between adjacent U, and V values. */
    int         mUVStep;

    /* Defines number of Us and Vs in a row inside the U/V panes.
     * Note that if U/V panes are interleaved, this value reflects the total
     * number of both, Us and Vs in a single row in the interleaved UV pane. */
    int         mUVInRow;

    /* Total number of each, U, and V elements in the framebuffer. */
    int         mUVTotalNum;

    /*
     * Checkerboard drawing related stuff
     */

    int         mCheckX;
    int         mCheckY;
    int         mCcounter;

    /* Emulated FPS (frames per second).
     * We will emulate 50 FPS. */
    static const int        mEmulatedFPS = 50;

    /* Defines time (in nanoseconds) between redrawing the checker board.
     * We will redraw the checker board every 15 milliseconds. */
    static const nsecs_t    mRedrawAfter = 15000000LL;

#if EFCD_ROTATE_FRAME
    /* Frame rotation frequency in nanosec (currently - 3 sec) */
    static const nsecs_t    mRotateFreq = 3000000000LL;

    /* Last time the frame has rotated. */
    nsecs_t     mLastRotatedAt;

    /* Type of the frame to display in the current rotation:
     *  0 - Checkerboard.
     *  1 - White/Red/Green/Blue horisontal stripes
     *  2 - Solid color. */
    int         mCurrentFrameType;

    /* Color to use to paint the solid color frame. Colors will rotate between
     * white, red, gree, and blue each time rotation comes to the solid color
     * frame. */
    YUVPixel*   mCurrentColor;
#endif  // EFCD_ROTATE_FRAME

};

}; /* ! namespace android */

#endif /* MSM_CAMERA_H */
