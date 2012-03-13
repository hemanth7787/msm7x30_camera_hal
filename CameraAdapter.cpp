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

#include <pthread.h>
#include <binder/MemoryHeapPmem.h>

#include "CameraAdapter.h"
 
#define LOG_FUNCTION_NAME       LOGD("%d: %s() ENTER", __LINE__, __FUNCTION__);
#define LOG_FUNCTION_NAME_EXIT  LOGD("%d: %s() EXIT", __LINE__, __FUNCTION__);

pthread_t cfg_thread;

void *config_thread(void *data) {
    android::CameraAdapter *ca = (android::CameraAdapter *)data;

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

namespace android {


CameraAdapter::CameraAdapter() : mConfigFd(-1), mControlFd(-1), 
                                 mSetupMemory(NULL)
{
    // Empty.
}

bool CameraAdapter::initialise(int cameraId)
{
    mCameraId = cameraId;
    
    /* We use the endpoints created by the kernel driver to access the
     * sensor, so open these here.
     */
    mControlFd = openEndpoint(MSM_CONTROL);
    if (mControlFd < 0) {
        LOGE("Unable to open the control endpoint for camera ID %d [ERR %s]", cameraId, strerror(errno));
        return false;
    }
    mConfigFd = openEndpoint(MSM_CONFIG);
    if (mConfigFd < 0) {
        LOGE("Unable to open the config endpoint for camera ID %d [ERR %s]", cameraId, strerror(errno));
        return false;
    }

    if (pthread_create(&cfg_thread, NULL, config_thread, (void *)this) != 0) {
        LOGE("Camera open thread creation failed");
        return false;
    }

    /* Do general setup for the sensor here. */
    getVendorId();
    if (! getSensorInformation())
        return false;
        
    return setupMemory();
}

CameraAdapter::~CameraAdapter()
{
    /* destroy the mSetupMemory object */
    if (mConfigFd >= 0)
        close(mConfigFd);
    if (mControlFd >= 0)
        close(mConfigFd);
}

bool CameraAdapter::setPreviewSize(int w, int h)
{
    LOGD("Set preview size to %d x %d", w, h);
    previewWidth = w;
    previewHeight = h;
    setPreviewMemory();
    return true;
}

/* private */
/* Try and read the I2C data from the sensor. This seems to only be
 * supported by a few of the sensors, so it's not critical if it fails.
 */
void CameraAdapter::getVendorId()
{
    struct sensor_cfg_data cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.cfgtype = CFG_I2C_IOCTL_R_OTP;

    if (configCommand(MSM_CAM_IOCTL_SENSOR_IO_CFG, &cfg) == false) {
        LOGE("Failed to get I2C sensor data.");
        return;
    }

    struct fuse_id *f = &cfg.cfg.fuse;
    if (f->fuse_id_word1 != 0 || f->fuse_id_word2 != 0) {
        LOGE("Read I2C data, but data appears invalid.");
        return;
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
}

bool CameraAdapter::setupMemory()
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
    LOGD("base @ %p", base);

    struct msm_pmem_info info;
    memset(&info, 0, sizeof(info));
    info.fd = mSetupMemory->getFd();
    info.vaddr = base;
    info.len = 4096;

    for (int i = 0; i < 18; i++) {
        info.type = types[i];
        info.offset = i * 4096;
        
        LOGD("%02d:: info: vaddr: %08x", i, (uint32_t)info.vaddr);
        bool ck = configIoctl(MSM_CAM_IOCTL_REGISTER_PMEM, &info);
        LOGD("ioctl to set PMEM = %d", ck);
    }
    return true;
}

/* private */
bool CameraAdapter::getSensorInformation()
{
    LOG_FUNCTION_NAME
    memset((void*)&mSensorInfo, 0, sizeof(mSensorInfo));

    if (mConfigFd == -1) {
        LOGE("configCommand:: config endpoint is NOT open");
        return false;
    }
    int rv = ioctl(mConfigFd, MSM_CAM_IOCTL_GET_SENSOR_INFO, &mSensorInfo);
    if (rv == 0)
        LOGD("Sensor information:: name: %s, flash %d, steps %d", 
             mSensorInfo.name, mSensorInfo.flash_enabled, 
             mSensorInfo.total_steps);
    else
        LOGE("Failed to get sensor information. Error: %s", strerror(errno));

    return (rv == 0);
}

bool CameraAdapter::setPreviewMemory()
{
    LOG_FUNCTION_NAME
    
    /* QualcommCameraHardware.h specifies a frame header size of 0x48. */
    size_t bufferSize = 0x48 + previewWidth * previewHeight * 2;
    int rqdBuffers = 4;
    return false;
}



int CameraAdapter::openEndpoint(msm_endpoint_e which)
{
//    LOG_FUNCTION_NAME
    char endpoint[32];
    
    switch(which) {
        case MSM_CONFIG:
            snprintf(endpoint, 32, "/dev/msm_camera/config%d", mCameraId);
            break;
        case MSM_CONTROL:
            snprintf(endpoint, 32, "/dev/msm_camera/control%d", mCameraId);
            break;
        case MSM_FB:
            snprintf(endpoint, 32, "/dev/msm_camera/frame%d", mCameraId);
            break;
        default:
            return -EINVAL;
    }
    int fd = open(endpoint, O_RDWR);
    if (fd < 0)
        LOGE("Error opening %s, %s", endpoint, strerror(errno));
//    else
//        LOGD("Opened %s as fd %d", endpoint, fd);
    return fd;
}

/* Private */
#define IOCTL_CONFIG   0
#define IOCTL_CONTROL  1
bool CameraAdapter::__ioctl(int which, int cmd, void *ptr)
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

/* Public */
bool CameraAdapter::configIoctl(int cmd, void *ptr)
{
    return __ioctl(IOCTL_CONFIG, cmd, ptr);
}

/* Public */
bool CameraAdapter::controlIoctl(int cmd, void *ptr)
{
    return __ioctl(IOCTL_CONTROL, cmd, ptr);
}

/* Public */
bool CameraAdapter::configCommand(int cmd, void *ptr)
{
    if (mConfigFd == -1) {
        LOGE("configCommand:: config endpoint is NOT open");
        return false;
    }
    int ck = ioctl(mConfigFd, cmd, ptr);
    if (ck == 0)
        return true;
    LOGD("%s: ioctl %d = %d [%s]", __FUNCTION__, _IOC_NR(cmd), ck, strerror(errno));
    LOGE("configCommand failed. Error: %s", strerror(errno));
    return false;
}

/* Public */
bool CameraAdapter::controlCommand(uint16_t type, void *ptr, uint16_t ptrlen)
{
    LOG_FUNCTION_NAME
    if (mControlFd == -1) {
        LOGE("controlCommand:: control endpoint is NOT open");
        return false;
    }

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
    
    int rv = ioctl(mControlFd, MSM_CAM_IOCTL_CTRL_COMMAND, &ctrlCmd);
    LOGD("ioctl:: rv = %d", rv);
    LOGD("    type:       %d", ctrlCmd.type);
    LOGD("    timeout_ms: %d", ctrlCmd.timeout_ms);
    LOGD("    length:     %d", ctrlCmd.length);
    LOGD("    value:      %p", ctrlCmd.value);
    LOGD("    resp_fd:    %d", ctrlCmd.resp_fd);
    
    LOGD("ctrlCmd.value = %d", *(int32_t *)ctrlCmd.value);
    if (rv == 0)
        return true;
//    memcpy(pZoom, (int32_t *)ctrlCmd.value, sizeof(int32_t));
    
    LOGE("configCommand failed. Error: %s", strerror(errno));
    return false;
}

bool CameraAdapter::getMaximumZoom()
{
    LOG_FUNCTION_NAME

    int32_t pZoom = 0;
    if (controlCommand(47/*CAMERA_GET_PARM_MAXZOOM*/, &pZoom, sizeof(pZoom))) {
        LOGD("getMaximumZoom = %d", pZoom);
        /* TODO - record max zoom */
        return true;
    }

    return false;
}

bool CameraAdapter::getMaximumAFSteps()
{
    LOG_FUNCTION_NAME

    int32_t pZoom = 0;
    if (controlCommand(CFG_GET_AF_MAX_STEPS, &pZoom, sizeof(pZoom))) {
        LOGD("Max AF Steps = %d", pZoom);
        /* TODO - record  */
        return true;
    }   
    return false;
}

bool CameraAdapter::getPictureFPS()
{
    LOG_FUNCTION_NAME

    struct sensor_pict_fps fps;
    memset(&fps, 0, sizeof(fps));
/*
#define CFG_GET_PICT_FPS		21
#define CFG_GET_PREV_L_PF		22
#define CFG_GET_PREV_P_PL		23
#define CFG_GET_PICT_L_PF		24
#define CFG_GET_PICT_P_PL		25
#define CFG_GET_AF_MAX_STEPS		26
#define CFG_GET_PICT_MAX_EXP_LC		27
*/
struct sensor_cfg_data cfg;
memset(&cfg, 0, sizeof(cfg));
cfg.cfgtype = CFG_GET_PICT_FPS;
cfg.cfg.gfps.prevfps = 7685;
/* Need to set the prevfps to get a valid result. liboemcamera sets it to
 * 7685.
 */
if (configCommand(MSM_CAM_IOCTL_SENSOR_IO_CFG, &cfg)) {
    LOGD("CFG_GET_PICT_FPS = %d", cfg.cfg.gfps.pictfps);
}

memset(&cfg, 0, sizeof(cfg));
cfg.cfgtype = CFG_GET_PICT_MAX_EXP_LC;
if (configCommand(MSM_CAM_IOCTL_SENSOR_IO_CFG, &cfg)) {
    LOGD("CFG_GET_PICT_MAX_EXP_LC = %d", cfg.cfg.pict_max_exp_lc);
}

memset(&cfg, 0, sizeof(cfg));
cfg.cfgtype = CFG_GET_PREV_L_PF;
if (configCommand(MSM_CAM_IOCTL_SENSOR_IO_CFG, &cfg)) {
    prevl_pf = cfg.cfg.prevl_pf;
    LOGD("CFG_GET_PREV_L_PF = %d", cfg.cfg.prevl_pf);
}

memset(&cfg, 0, sizeof(cfg));
cfg.cfgtype = CFG_GET_PREV_P_PL;
if (configCommand(MSM_CAM_IOCTL_SENSOR_IO_CFG, &cfg)) {
    prevp_pl = cfg.cfg.prevp_pl;
    LOGD("CFG_GET_PREV_P_PL = %d", cfg.cfg.prevp_pl);
}
LOGD("Preview image is %d lines, each of %d pixels per line", prevl_pf, prevp_pl);

memset(&cfg, 0, sizeof(cfg));
cfg.cfgtype = CFG_GET_PICT_L_PF;
if (configCommand(MSM_CAM_IOCTL_SENSOR_IO_CFG, &cfg)) {
    LOGD("CFG_GET_PICT_L_PF = %d", cfg.cfg.pictl_pf);
}
memset(&cfg, 0, sizeof(cfg));
cfg.cfgtype = CFG_GET_PICT_P_PL;
if (configCommand(MSM_CAM_IOCTL_SENSOR_IO_CFG, &cfg)) {
    LOGD("CFG_GET_PICT_P_PL = %d", cfg.cfg.pictp_pl);
}



    return false;
}

bool CameraAdapter::setMode(uint16_t mode)
{
    LOG_FUNCTION_NAME

    struct sensor_cfg_data cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = mode;
    cfg.cfgtype = CFG_SET_MODE;

    if (configCommand(MSM_CAM_IOCTL_SENSOR_IO_CFG, &cfg)) {
        LOGD("Set mode to %d", mode);
        return true;
    }
    LOGE("Unable to set mode to %d", mode);
    return false;
}

bool CameraAdapter::setAFMode(af_mode_t af_type)
{
    LOG_FUNCTION_NAME
    
    if (controlCommand(13, &af_type, sizeof(af_type))) {
        LOGD("Set AF mode to %d", af_type);
        return true;
    }
    LOGE("Unable to set AF mode to %d", af_type);
    return false;
}



} /* ! android */

