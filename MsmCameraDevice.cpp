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

#define LOG_TAG "MsmCameraDevice"

#include <fcntl.h>
#include <linux/videodev2.h>

#include "MsmCamera.h"
#include "Debug.h"

static pthread_t cfg_thread;
void *config_thread(void *data);

namespace android {

MsmCameraDevice::MsmCameraDevice(MsmCamera *camera) : mCamera(camera)
{
    mVendorId = -1;
    mConfigFd = -1;
    mControlFd = -1;   
}


status_t MsmCameraDevice::connectDevice()
{
    LOG_FUNCTION_NAME

    Mutex::Autolock locker(&mObjectLock);

    if (!isInitialized()) {
        LOGE("%s(): Msm camera device is not initialized.", __FUNCTION__);
        return EINVAL;
    }
    if (isConnected()) {
        LOGW("%s(): Msm camera device is already connected.", __FUNCTION__);
        return NO_ERROR;
    }

    /* Open our endpoints to the kernel driver. */
    mControlFd = openEndpoint(MSM_CONTROL);
    if (mControlFd < 0) {
        LOGE("Unable to open the control endpoint [ERR %s]", 
             strerror(errno));
        return ALREADY_EXISTS;
    }
    mConfigFd = openEndpoint(MSM_CONFIG);
    if (mConfigFd < 0) {
        LOGE("Unable to open the config endpoint [ERR %s]",
             strerror(errno));
        close(mControlFd);
        return ALREADY_EXISTS;
    }

    if (!getVendorId()) {
        LOGE("Unable to communicate with the device.");
        close(mConfigFd);
        close(mControlFd);
        return BAD_VALUE;
    }

    /* Set quarter size... */
    setSensorMode(SENSOR_PREVIEW_MODE); /* same as QTR_SIZE */
    
    if (! getSensorInformation() || !setupMemory()) {
        close(mConfigFd);
        close(mControlFd);
        return BAD_VALUE;
    }
               
    mState = CDS_CONNECTED;
    return NO_ERROR;
}

status_t MsmCameraDevice::disconnectDevice()
{
    LOG_FUNCTION_NAME
    
    Mutex::Autolock locker(&mObjectLock);
    if (!isConnected()) {
        LOGW("%s: Fake camera device is already disconnected.", __FUNCTION__);
        return NO_ERROR;
    }
    if (isStarted()) {
        LOGE("%s: Cannot disconnect from the started device.", __FUNCTION__);
        return EINVAL;
    }

    /* remove memory */
    if (mConfigFd != -1)
        close(mConfigFd);
    if (mControlFd != -1)
        close(mControlFd);

    mState = CDS_INITIALIZED;
    return NO_ERROR;
}

/* Setup & start any/all operations required to allow the camera to
 * provide frames.
 */
status_t MsmCameraDevice::startDevice(int width, int height, uint32_t pix_fmt)
{
    /* We need to start a thread to deal with configuration issues, so
     * do this here.
     */
//    if (pthread_create(&cfg_thread, NULL, config_thread, (void *)this) != 0) {
//        LOGE("Camera open thread creation failed");
//        return INVALID_OPERATION;
//    }

    Mutex::Autolock locker(&mObjectLock);
    if (!isConnected()) {
        LOGE("%s: Fake camera device is not connected.", __FUNCTION__);
        return EINVAL;
    }
    if (isStarted()) {
        LOGE("%s: Fake camera device is already started.", __FUNCTION__);
        return EINVAL;
    }

    /* Initialize the base class. */
    const status_t res = CameraDevice::commonStartDevice(width, height, pix_fmt);
    if (res == NO_ERROR) {
        /* Calculate U/V panes inside the framebuffer. */
        switch (mPixelFormat) {
            case V4L2_PIX_FMT_YVU420:
                mFrameV = mCurrentFrame + mTotalPixels;
                mFrameU = mFrameU + mTotalPixels / 4;
                mUVStep = 1;
                mUVTotalNum = mTotalPixels / 4;
                break;

            case V4L2_PIX_FMT_YUV420:
                mFrameU = mCurrentFrame + mTotalPixels;
                mFrameV = mFrameU + mTotalPixels / 4;
                mUVStep = 1;
                mUVTotalNum = mTotalPixels / 4;
                break;

            case V4L2_PIX_FMT_NV21:
                /* Interleaved UV pane, V first. */
                mFrameV = mCurrentFrame + mTotalPixels;
                mFrameU = mFrameV + 1;
                mUVStep = 2;
                mUVTotalNum = mTotalPixels / 4;
                break;

            case V4L2_PIX_FMT_NV12:
                /* Interleaved UV pane, U first. */
                mFrameU = mCurrentFrame + mTotalPixels;
                mFrameV = mFrameU + 1;
                mUVStep = 2;
                mUVTotalNum = mTotalPixels / 4;
                break;

            default:
                LOGE("%s: Unknown pixel format %.4s", __FUNCTION__,
                     reinterpret_cast<const char*>(&mPixelFormat));
                return EINVAL;
        }
        /* Number of items in a single row inside U/V panes. */
        mUVInRow = (width / 2) * mUVStep;
        mState = CDS_STARTED;
    } else {
        LOGE("%s: commonStartDevice failed", __FUNCTION__);
    }

    return res;    
    return NO_ERROR;
}


status_t MsmCameraDevice::stopDevice()
{
    LOG_FUNCTION_NAME
    
    /* destroy the config thread */
    Mutex::Autolock locker(&mObjectLock);
    if (!isStarted()) {
        LOGW("%s: Fake camera device is not started.", __FUNCTION__);
        return NO_ERROR;
    }

    mFrameU = mFrameV = NULL;
    CameraDevice::commonStopDevice();
    mState = CDS_CONNECTED;

    return NO_ERROR;
}

/*************************************************************************
 * Public functions - specific to this class
 *************************************************************************/
#define IOCTL_CONFIG  0
#define IOCTL_CONTROL 1
bool MsmCameraDevice::configIoctl(int cmd, void *ptr)
{
    return __ioctl(IOCTL_CONFIG, cmd, ptr);
}

bool MsmCameraDevice::controlIoctl(int cmd, void *ptr)
{
    return __ioctl(IOCTL_CONTROL, cmd, ptr);
}

bool MsmCameraDevice::configCommand(int cmd, void *ptr)
{
    if (__ioctl(IOCTL_CONFIG, cmd, ptr))
        return true;       
    LOGE("configCommand failed. Error: %s", strerror(errno));
    return false;
}

bool MsmCameraDevice::controlCommand(uint16_t type, void *ptr, uint16_t ptrlen)
{
    LOG_FUNCTION_NAME
    struct msm_ctrl_cmd ctrlCmd;
    memset(&ctrlCmd, 0, sizeof(ctrlCmd));

/*
struct msm_ctrl_cmd {
	uint16_t type;
	uint16_t length;
	void *value;
	uint16_t status;
	uint32_t timeout_ms;
	int resp_fd;  FIXME: to be used by the kernel, pass-through for now
};
*/
    ctrlCmd.type       = type;
    ctrlCmd.timeout_ms = 5000;
    ctrlCmd.length     = ptrlen;
    ctrlCmd.value      = ptr;
    ctrlCmd.resp_fd    = mControlFd;

    LOGD("controlCommand: ioctl for type %d", type);
    LOGD("    type:       %d", ctrlCmd.type);
    LOGD("    timeout_ms: %d", ctrlCmd.timeout_ms);
    LOGD("    length:     %d", ctrlCmd.length);
    LOGD("    value:      %p", ctrlCmd.value);
    LOGD("    resp_fd:    %d", ctrlCmd.resp_fd);
    
    int rv = __ioctl(IOCTL_CONTROL, MSM_CAM_IOCTL_CTRL_COMMAND, &ctrlCmd);
    LOGD("ioctl:: rv = %d", rv);
    LOGD("    type:       %d", ctrlCmd.type);
    LOGD("    timeout_ms: %d", ctrlCmd.timeout_ms);
    LOGD("    length:     %d", ctrlCmd.length);
    LOGD("    value:      %p", ctrlCmd.value);
    LOGD("    resp_fd:    %d", ctrlCmd.resp_fd);
    
    LOGD("ctrlCmd.value = %d", *(int32_t *)ctrlCmd.value);
    if (rv == 0)
        return true;
    
    LOGE("configCommand failed. Error: %s", strerror(errno));
    return false;
}

/*************************************************************************
 * Private functions
 *************************************************************************/
int MsmCameraDevice::openEndpoint(msm_endpoint_e which)
{
    LOG_FUNCTION_NAME
    char endpoint[32];
    int cameraId = mCamera->getCameraId();
    
    switch(which) {
        case MSM_CONFIG:
            snprintf(endpoint, 32, "/dev/msm_camera/config%d", cameraId);
            break;
        case MSM_CONTROL:
            snprintf(endpoint, 32, "/dev/msm_camera/control%d", cameraId);
            break;
        case MSM_FB:
            snprintf(endpoint, 32, "/dev/msm_camera/frame%d", cameraId);
            break;
        default:
            return -EINVAL;
    }
    int fd = open(endpoint, O_RDWR);
    if (fd < 0)
        LOGE("Error opening %s, %s", endpoint, strerror(errno));
    return fd;
}

/* Read the vendor ID for the board from the sensor using the I2C config
 * ioctl. This allows us to ensure we have a working connection to the
 * sensor as well as determine what type of sensor we have.
 */
bool MsmCameraDevice::getVendorId()
{
    struct sensor_cfg_data cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.cfgtype = CFG_I2C_IOCTL_R_OTP;

    if (configIoctl(MSM_CAM_IOCTL_SENSOR_IO_CFG, &cfg) == false) {
        LOGE("Failed to get I2C sensor data.");
        return false;
    }

    struct fuse_id *f = &cfg.cfg.fuse;
    if (f->fuse_id_word1 != 0 || f->fuse_id_word2 != 0) {
        LOGE("Read I2C data, but data appears invalid.");
        return false;
    }   
/*
    LOGD("Got sensor IO data...");
    LOGD("    fuse #1: %08x", f->fuse_id_word1);
    LOGD("    fuse #2: %08x", f->fuse_id_word2);
    LOGD("    fuse #3: %08x", f->fuse_id_word3);
    LOGD("    fuse #4: %08x", f->fuse_id_word4);
*/
    mVendorId = f->fuse_id_word3;
    LOGD("Sensor Vendor ID: %d", mVendorId);
    return true;
}

bool MsmCameraDevice::setSensorMode(int mode)
{
    LOG_FUNCTION_NAME
    struct sensor_cfg_data cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.cfgtype = CFG_SET_MODE;
    cfg.mode = mode;
    if (configIoctl(MSM_CAM_IOCTL_SENSOR_IO_CFG, &cfg)) {
        LOGD("Set sensor mode to %d", mode);
        return true;
    }
    LOGE("Error setting sensor mode to %d", mode);
    return false;
}    

bool MsmCameraDevice::getSensorInformation()
{
    LOG_FUNCTION_NAME
    memset((void*)&mSensorInfo, 0, sizeof(mSensorInfo));

    if (configIoctl(MSM_CAM_IOCTL_GET_SENSOR_INFO, &mSensorInfo))
        LOGD("Sensor information:: name: %s, flash %d, steps %d", 
             mSensorInfo.name, mSensorInfo.flash_enabled, 
             mSensorInfo.total_steps);

    /* Do we need to store these somewhere? */
    struct sensor_cfg_data cfg;

    memset(&cfg, 0, sizeof(cfg));
    cfg.cfgtype = CFG_GET_PICT_FPS;
    if (configIoctl(MSM_CAM_IOCTL_SENSOR_IO_CFG, &cfg))
        LOGD("Picture FPS: %d", cfg.cfg.gfps.pictfps);
    
    memset(&cfg, 0, sizeof(cfg));
    cfg.cfgtype = CFG_GET_PREV_L_PF;
    if (configIoctl(MSM_CAM_IOCTL_SENSOR_IO_CFG, &cfg))
        LOGD("Preview LPF: %d", cfg.cfg.prevl_pf);

    memset(&cfg, 0, sizeof(cfg));
    cfg.cfgtype = CFG_GET_PREV_P_PL;
    if (configIoctl(MSM_CAM_IOCTL_SENSOR_IO_CFG, &cfg))
        LOGD("Preview PPL: %d", cfg.cfg.prevp_pl);
   
    LOGE("Failed to get sensor information. Error: %s", strerror(errno));
    return true;
}

bool MsmCameraDevice::setupMemory()
{
    /* This is what the existing liboemcamera does, so we replicate.
     * Essentially provide 3 x 4k buffers to AEC, AWB, AF, IHIST, CS and RS
     */
    int types[18] = { MSM_PMEM_AEC, MSM_PMEM_AEC, MSM_PMEM_AEC,
                      MSM_PMEM_AWB, MSM_PMEM_AWB, MSM_PMEM_AWB,
                      MSM_PMEM_AF, MSM_PMEM_AF, MSM_PMEM_AF,
                      MSM_PMEM_IHIST, MSM_PMEM_IHIST, MSM_PMEM_IHIST,
                      MSM_PMEM_CS, MSM_PMEM_CS, MSM_PMEM_CS,
                      MSM_PMEM_RS, MSM_PMEM_RS, MSM_PMEM_RS };

    /* We need 18 blocks of 4096 bytes of physical memory. */
    mSetupMemory = new PhysicalMemoryPool(18, 4096);
    if (! mSetupMemory->initialized()) {
        LOGE("%s: failed to get pmem block...", __FUNCTION__);
        return false;
    }

    void *base = mSetupMemory->getBaseAddress();

    struct msm_pmem_info info;
    memset(&info, 0, sizeof(info));
    info.fd = mSetupMemory->getFd();
    info.vaddr = base;
    info.len = 4096;

    bool ck = true;
    for (int i = 0; i < 18; i++) {
        info.type = types[i];
        info.offset = i * 4096;       
        ck = configIoctl(MSM_CAM_IOCTL_REGISTER_PMEM, &info);
        if (ck == false) {
            LOGE("Unable to set memory for %08x", types[i]);
            break;
        }
    }
    return ck;
}

bool MsmCameraDevice::__ioctl(int which, int cmd, void *ptr)
{
    int fd = which == IOCTL_CONFIG ? mConfigFd : mControlFd;
    if (fd == -1) {
        LOGE("%s:: %s endpoint is NOT open", __FUNCTION__, 
             (which == IOCTL_CONFIG ? "config" : "control"));
        return false;
    }
    if (ioctl(fd, cmd, ptr) == 0)
        return true;
    LOGE("%s:: failed. cmd = %d, error = %s", __FUNCTION__, _IOC_NR(cmd),
         strerror(errno));
    return false;
}


bool MsmCameraDevice::inWorkerThread()
{
    /* Wait till FPS timeout expires, or thread exit message is received. */
    WorkerThread::SelectRes res =
        getWorkerThread()->Select(-1, 1000000 / mEmulatedFPS);
    if (res == WorkerThread::EXIT_THREAD) {
        LOGV("%s: Worker thread has been terminated.", __FUNCTION__);
        return false;
    }

    /* Lets see if we need to generate a new frame. */
    if ((systemTime(SYSTEM_TIME_MONOTONIC) - mLastRedrawn) >= mRedrawAfter) {
        /*
         * Time to generate a new frame.
         */

#if EFCD_ROTATE_FRAME
        const int frame_type = rotateFrame();
        switch (frame_type) {
            case 0:
                drawCheckerboard();
                break;
            case 1:
                drawStripes();
                break;
            case 2:
                drawSolid(mCurrentColor);
                break;
        }
#else
        /* Draw the checker board. */
        drawCheckerboard();

#endif  // EFCD_ROTATE_FRAME

        mLastRedrawn = systemTime(SYSTEM_TIME_MONOTONIC);
    }

    /* Timestamp the current frame, and notify the camera HAL about new frame. */
    mCurFrameTimestamp = systemTime(SYSTEM_TIME_MONOTONIC);
    mCamera->onNextFrameAvailable(mCurrentFrame, mCurFrameTimestamp, this);

    return true;
}

/****************************************************************************
 * Fake camera device private API
 ***************************************************************************/

void MsmCameraDevice::drawCheckerboard()
{
    const int size = mFrameWidth / 10;
    bool black = true;

    if((mCheckX / size) & 1)
        black = false;
    if((mCheckY / size) & 1)
        black = !black;

    int county = mCheckY % size;
    int checkxremainder = mCheckX % size;
    uint8_t* Y = mCurrentFrame;
    uint8_t* U_pos = mFrameU;
    uint8_t* V_pos = mFrameV;
    uint8_t* U = U_pos;
    uint8_t* V = V_pos;

    for(int y = 0; y < mFrameHeight; y++) {
        int countx = checkxremainder;
        bool current = black;
        for(int x = 0; x < mFrameWidth; x += 2) {
            if (current) {
                mBlackYUV.get(Y, U, V);
            } else {
                mWhiteYUV.get(Y, U, V);
            }
            *Y = changeExposure(*Y);
            Y[1] = *Y;
            Y += 2; U += mUVStep; V += mUVStep;
            countx += 2;
            if(countx >= size) {
                countx = 0;
                current = !current;
            }
        }
        if (y & 0x1) {
            U_pos = U;
            V_pos = V;
        } else {
            U = U_pos;
            V = V_pos;
        }
        if(county++ >= size) {
            county = 0;
            black = !black;
        }
    }
    mCheckX += 3;
    mCheckY++;

    /* Run the square. */
    int sqx = ((mCcounter * 3) & 255);
    if(sqx > 128) sqx = 255 - sqx;
    int sqy = ((mCcounter * 5) & 255);
    if(sqy > 128) sqy = 255 - sqy;
    const int sqsize = mFrameWidth / 10;
    drawSquare(sqx * sqsize / 32, sqy * sqsize / 32, (sqsize * 5) >> 1,
               (mCcounter & 0x100) ? &mRedYUV : &mGreenYUV);
    mCcounter++;
}

void MsmCameraDevice::drawSquare(int x,
                                          int y,
                                          int size,
                                          const YUVPixel* color)
{
#define min(a,b) (a <= b ? a : b)

    const int square_xstop = min(mFrameWidth, x + size);
    const int square_ystop = min(mFrameHeight, y + size);
    uint8_t* Y_pos = mCurrentFrame + y * mFrameWidth + x;

    // Draw the square.
    for (; y < square_ystop; y++) {
        const int iUV = (y / 2) * mUVInRow + (x / 2) * mUVStep;
        uint8_t* sqU = mFrameU + iUV;
        uint8_t* sqV = mFrameV + iUV;
        uint8_t* sqY = Y_pos;
        for (int i = x; i < square_xstop; i += 2) {
            color->get(sqY, sqU, sqV);
            *sqY = changeExposure(*sqY);
            sqY[1] = *sqY;
            sqY += 2; sqU += mUVStep; sqV += mUVStep;
        }
        Y_pos += mFrameWidth;
    }
}

#if EFCD_ROTATE_FRAME

void MsmCameraDevice::drawSolid(YUVPixel* color)
{
    /* All Ys are the same. */
    memset(mCurrentFrame, changeExposure(color->Y), mTotalPixels);

    /* Fill U, and V panes. */
    uint8_t* U = mFrameU;
    uint8_t* V = mFrameV;
    for (int k = 0; k < mUVTotalNum; k++, U += mUVStep, V += mUVStep) {
        *U = color->U;
        *V = color->V;
    }
}

void MsmCameraDevice::drawStripes()
{
    /* Divide frame into 4 stripes. */
    const int change_color_at = mFrameHeight / 4;
    const int each_in_row = mUVInRow / mUVStep;
    uint8_t* pY = mCurrentFrame;
    for (int y = 0; y < mFrameHeight; y++, pY += mFrameWidth) {
        /* Select the color. */
        YUVPixel* color;
        const int color_index = y / change_color_at;
        if (color_index == 0) {
            /* White stripe on top. */
            color = &mWhiteYUV;
        } else if (color_index == 1) {
            /* Then the red stripe. */
            color = &mRedYUV;
        } else if (color_index == 2) {
            /* Then the green stripe. */
            color = &mGreenYUV;
        } else {
            /* And the blue stripe at the bottom. */
            color = &mBlueYUV;
        }

        /* All Ys at the row are the same. */
        memset(pY, changeExposure(color->Y), mFrameWidth);

        /* Offset of the current row inside U/V panes. */
        const int uv_off = (y / 2) * mUVInRow;
        /* Fill U, and V panes. */
        uint8_t* U = mFrameU + uv_off;
        uint8_t* V = mFrameV + uv_off;
        for (int k = 0; k < each_in_row; k++, U += mUVStep, V += mUVStep) {
            *U = color->U;
            *V = color->V;
        }
    }
}

int MsmCameraDevice::rotateFrame()
{
    if ((systemTime(SYSTEM_TIME_MONOTONIC) - mLastRotatedAt) >= mRotateFreq) {
        mLastRotatedAt = systemTime(SYSTEM_TIME_MONOTONIC);
        mCurrentFrameType++;
        if (mCurrentFrameType > 2) {
            mCurrentFrameType = 0;
        }
        if (mCurrentFrameType == 2) {
            LOGD("********** Rotated to the SOLID COLOR frame **********");
            /* Solid color: lets rotate color too. */
            if (mCurrentColor == &mWhiteYUV) {
                LOGD("----- Painting a solid RED frame -----");
                mCurrentColor = &mRedYUV;
            } else if (mCurrentColor == &mRedYUV) {
                LOGD("----- Painting a solid GREEN frame -----");
                mCurrentColor = &mGreenYUV;
            } else if (mCurrentColor == &mGreenYUV) {
                LOGD("----- Painting a solid BLUE frame -----");
                mCurrentColor = &mBlueYUV;
            } else {
                /* Back to white. */
                LOGD("----- Painting a solid WHITE frame -----");
                mCurrentColor = &mWhiteYUV;
            }
        } else if (mCurrentFrameType == 0) {
            LOGD("********** Rotated to the CHECKERBOARD frame **********");
        } else {
            LOGD("********** Rotated to the STRIPED frame **********");
        }
    }

    return mCurrentFrameType;
}

#endif  // EFCD_ROTATE_FRAME






} /* ! namespace android */

void *config_thread(void *data) {
    android::MsmCameraDevice *ca = (android::MsmCameraDevice *)data;

/*
struct sensor_cfg_data {
	int cfgtype;
	int mode;
	int rs;
	uint8_t max_steps;

	union {
		int8_t af_area;
		int8_t effect;
		uint8_t lens_shading;
		uint16_t prevl_pf;
		uint16_t prevp_pl;
		uint16_t pictl_pf;
		uint16_t pictp_pl;
		uint32_t pict_max_exp_lc;
		uint16_t p_fps;
		uint16_t flash_exp_div;
		struct sensor_pict_fps gfps;
		struct exp_gain_cfg exp_gain;
		struct focus_cfg focus;
		struct fps_cfg fps;
		struct wb_info_cfg wb_info;
		struct fuse_id fuse;
		struct lsc_cfg lsctable;
		struct otp_cfg sp3d_otp_cfg;
		struct flash_cfg flash_data;
//For 2nd CAM
		enum antibanding_mode antibanding_value;
		enum brightness_t brightness_value;
		enum frontcam_t frontcam_value;
		enum wb_mode wb_value;
		enum iso_mode iso_value;
		enum sharpness_mode sharpness_value;
		enum saturation_mode saturation_value;
		enum contrast_mode  contrast_value;
		enum qtr_size_mode qtr_size_mode_value;
		enum sensor_af_mode af_mode_value;
	} cfg;
};
*/

    for (int j = 0; j < 4; j++) {
    char buffer[200];
    struct msm_stats_event_ctrl evt;
    
    memset(&buffer, 0, 200);

    evt.timeout_ms = -1;
    evt.ctrl_cmd.value = &buffer;
    evt.ctrl_cmd.length = 200;
   
    if (ca->configCommand(MSM_CAM_IOCTL_GET_STATS, &evt)) {
        LOGD("    resptype = %d", evt.resptype);
        if (evt.resptype == MSM_CAM_RESP_CTRL) {
            /* The struct msm_ctrl_cmd that is returned is then resent
             * using the MSM_CAM_IOCTL_CTRL_CMD_DONE. The values within the
             * structure here are the ones that will be returned by the
             * ioctl.
             */
            LOGD("    ctrl_cmd:");
            LOGD("        type:      %d", evt.ctrl_cmd.type);
            LOGD("        length:    %d", evt.ctrl_cmd.length);
            LOGD("        value:     %p", evt.ctrl_cmd.value);
            for (int i = 0; i < evt.ctrl_cmd.length; i++)
                LOGD("        buffer %d:   %02x", i, buffer[i]);
            LOGD("        resp_fd:   %d", evt.ctrl_cmd.resp_fd);
            LOGD("        status:    %d", evt.ctrl_cmd.status);
            buffer[0] = 0x99;
            if (ca->controlIoctl(MSM_CAM_IOCTL_CTRL_CMD_DONE, &evt.ctrl_cmd)) {
                LOGD("CMD_DONE sent OK");
            }
            
        }

    }
    }
    
    return NULL;
}

