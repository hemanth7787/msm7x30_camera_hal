/*
 * Copyright (C) 2012 zathrasorama@gmail.com
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

/* The Camera class is a wrapper around the camera_device structure (as
 * declared in hardware/libhardware/include/hardware/camera.h). 
 * It contains:
 *   - the various callback routines that are needed
 *   - CameraParameter object
 *   - PreviewWindow object
 *   - CallbackNotifier object
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <utils/threads.h>
#include <camera/CameraParameters.h>
#include <hardware/camera.h>

#include "CameraDevice.h"
#include "PreviewWindow.h"
#include "CallbackNotifier.h"

namespace android {

class Camera : public camera_device {
    public:
        /** Constructor */
        Camera(int cameraId, struct hw_module_t* module);

        /** Destructor */
        virtual ~Camera();


protected:
    /* Actual handler for camera_device_ops_t::set_preview_window callback.
     * NOTE: When this method is called the object is locked.
     * Note that failures in this method are reported as negave EXXX statuses.
     */
    virtual status_t setPreviewWindow(struct preview_stream_ops *window);

    /* Actual handler for camera_device_ops_t::set_callbacks callback.
     * NOTE: When this method is called the object is locked.
     */
    virtual void setCallbacks(camera_notify_callback notify_cb,
                              camera_data_callback data_cb,
                              camera_data_timestamp_callback data_cb_timestamp,
                              camera_request_memory get_memory,
                              void* user);

    /* Actual handler for camera_device_ops_t::enable_msg_type callback.
     * NOTE: When this method is called the object is locked.
     */
    virtual void enableMsgType(int32_t msg_type);

    /* Actual handler for camera_device_ops_t::disable_msg_type callback.
     * NOTE: When this method is called the object is locked.
     */
    virtual void disableMsgType(int32_t msg_type);

    /* Actual handler for camera_device_ops_t::msg_type_enabled callback.
     * NOTE: When this method is called the object is locked.
     * Return:
     *  0 if message(s) is (are) disabled, != 0 if enabled.
     */
    virtual int isMsgTypeEnabled(int32_t msg_type);

    /* Actual handler for camera_device_ops_t::start_preview callback.
     * NOTE: When this method is called the object is locked.
     * Note that failures in this method are reported as negave EXXX statuses.
     */
    virtual status_t startPreview();

    /* Actual handler for camera_device_ops_t::stop_preview callback.
     * NOTE: When this method is called the object is locked.
     */
    virtual void stopPreview();

    /* Actual handler for camera_device_ops_t::preview_enabled callback.
     * NOTE: When this method is called the object is locked.
     * Return:
     *  0 if preview is disabled, != 0 if enabled.
     */
    virtual int isPreviewEnabled();

    /* Actual handler for camera_device_ops_t::store_meta_data_in_buffers callback.
     * NOTE: When this method is called the object is locked.
     * Note that failures in this method are reported as negave EXXX statuses.
     */
    virtual status_t storeMetaDataInBuffers(int enable);

    /* Actual handler for camera_device_ops_t::start_recording callback.
     * NOTE: When this method is called the object is locked.
     * Note that failures in this method are reported as negave EXXX statuses.
     */
    virtual status_t startRecording();

    /* Actual handler for camera_device_ops_t::stop_recording callback.
     * NOTE: When this method is called the object is locked.
     */
    virtual void stopRecording();

    /* Actual handler for camera_device_ops_t::recording_enabled callback.
     * NOTE: When this method is called the object is locked.
     * Return:
     *  0 if recording is disabled, != 0 if enabled.
     */
    virtual int isRecordingEnabled();

    /* Actual handler for camera_device_ops_t::release_recording_frame callback.
     * NOTE: When this method is called the object is locked.
     */
    virtual void releaseRecordingFrame(const void* opaque);

    /* Actual handler for camera_device_ops_t::auto_focus callback.
     * NOTE: When this method is called the object is locked.
     * Note that failures in this method are reported as negave EXXX statuses.
     */
    virtual status_t setAutoFocus();

    /* Actual handler for camera_device_ops_t::cancel_auto_focus callback.
     * NOTE: When this method is called the object is locked.
     * Note that failures in this method are reported as negave EXXX statuses.
     */
    virtual status_t cancelAutoFocus();

    /* Actual handler for camera_device_ops_t::take_picture callback.
     * NOTE: When this method is called the object is locked.
     * Note that failures in this method are reported as negave EXXX statuses.
     */
    virtual status_t takePicture();

    /* Actual handler for camera_device_ops_t::cancel_picture callback.
     * NOTE: When this method is called the object is locked.
     * Note that failures in this method are reported as negave EXXX statuses.
     */
    virtual status_t cancelPicture();

    /* Actual handler for camera_device_ops_t::set_parameters callback.
     * NOTE: When this method is called the object is locked.
     * Note that failures in this method are reported as negave EXXX statuses.
     */
    virtual status_t setParameters(const char* parms);

    /* Function to actually handle the setting of parameters. */
    virtual status_t setParameters(const CameraParameters &params);

    /* Actual handler for camera_device_ops_t::get_parameters callback.
     * NOTE: When this method is called the object is locked.
     * Return:
     *  Flattened parameters string. The caller will free the buffer allocated
     *  for the string by calling camera_device_ops_t::put_parameters callback.
     */
    virtual char* getParameters();

    /* Actual handler for camera_device_ops_t::put_parameters callback.
     * Called to free the string returned from camera_device_ops_t::get_parameters
     * callback. There is nothing more to it: the name of the callback is just
     * misleading.
     * NOTE: When this method is called the object is locked.
     */
    virtual void putParameters(char* params);

    /* Actual handler for camera_device_ops_t::send_command callback.
     * NOTE: When this method is called the object is locked.
     * Note that failures in this method are reported as negave EXXX statuses.
     */
    virtual status_t sendCommand(int32_t cmd, int32_t arg1, int32_t arg2);

    /* Actual handler for camera_device_ops_t::release callback.
     * NOTE: When this method is called the object is locked.
     */
    virtual void releaseCamera();

    /* Actual handler for camera_device_ops_t::dump callback.
     * NOTE: When this method is called the object is locked.
     * Note that failures in this method are reported as negave EXXX statuses.
     */
    virtual status_t dumpCamera(int fd);


    /*********************************************************************
     * Camera API implementation
     *********************************************************************/

    public:
        /* Creates connection to the emulated camera device.
         * This method is called in response to hw_module_methods_t::open callback.
         * NOTE: When this method is called the object is locked.
         * Note that failures in this method are reported as negave EXXX statuses.
         */
        virtual status_t connectCamera(hw_device_t** device);

        /* Closes connection to the emulated camera.
         * This method is called in response to camera_device::close callback.
         * NOTE: When this method is called the object is locked.
         * Note that failures in this method are reported as negave EXXX statuses.
         */
        virtual status_t closeCamera();

        /* Gets camera information.
         * This method is called in response to camera_module_t::get_camera_info
         * callback.
         * NOTE: When this method is called the object is locked.
         * Note that failures in this method are reported as negave EXXX statuses.
         */
    virtual status_t getCameraInfo(struct camera_info* info);


    /****************************************************************************
     * Abstract API
     ***************************************************************************/

    public:
        /* Gets camera device used by this instance of the camera. */
        virtual CameraDevice* getCameraDevice() = 0;

    /*********************************************************************
     * Public API
     *********************************************************************/

    public:
        /* Initializes EmulatedCamera instance.
         * Return:
         *  NO_ERROR on success, or an appropriate error status on failure.
         */
        virtual status_t Initialize() = 0;

        /* Next frame is available in the camera device.
         * This is a notification callback that is invoked by the camera device when
         * a new frame is available.
         * Note that most likely this method is called in context of a worker thread
         * that camera device has created for frame capturing.
         * Param:
         *  frame - Captured frame, or NULL if camera device didn't pull the frame
         *      yet. If NULL is passed in this parameter use GetCurrentFrame method
         *      of the camera device class to obtain the next frame. Also note that
         *      the size of the frame that is passed here (as well as the frame
         *      returned from the GetCurrentFrame method) is defined by the current
         *      frame settings (width + height + pixel format) for the camera device.
         * timestamp - Frame's timestamp.
         * camera_dev - Camera device instance that delivered the frame.
         */
        virtual void onNextFrameAvailable(const void* frame,
                                          nsecs_t timestamp,
                                          CameraDevice* camera_dev);

        /* Entry point for notifications that occur in camera device.
         * Param:
         *  err - CAMERA_ERROR_XXX error code.
         */
        virtual void onCameraDeviceError(int err);

    /*********************************************************************
     * Camera API callbacks as defined by camera_device_ops structure.
     * See hardware/libhardware/include/hardware/camera.h for information
     * on each of these callbacks. 
     * As implemented in this class, these callbacks simply dispatch the
     * call into an instance of Camera class defined by the 'camera_device'
     * parameter.
     *********************************************************************/

    private:
        static int set_preview_window(struct camera_device* dev,
                                      struct preview_stream_ops* window);

        static void set_callbacks(struct camera_device* dev,
                                  camera_notify_callback notify_cb,
                                  camera_data_callback data_cb,
                                  camera_data_timestamp_callback data_cb_ts,
                                  camera_request_memory get_memory,
                                  void* user);

        static void enable_msg_type(struct camera_device* dev, int32_t msg_type);

        static void disable_msg_type(struct camera_device* dev, int32_t msg_type);

        static int msg_type_enabled(struct camera_device* dev, int32_t msg_type);

        static int start_preview(struct camera_device* dev);

        static void stop_preview(struct camera_device* dev);

        static int preview_enabled(struct camera_device* dev);

        static int store_meta_data_in_buffers(struct camera_device* dev, int enable);

        static int start_recording(struct camera_device* dev);

        static void stop_recording(struct camera_device* dev);

        static int recording_enabled(struct camera_device* dev);

        static void release_recording_frame(struct camera_device* dev,
                                            const void* opaque);

        static int auto_focus(struct camera_device* dev);

        static int cancel_auto_focus(struct camera_device* dev);

        static int take_picture(struct camera_device* dev);

        static int cancel_picture(struct camera_device* dev);

        static int set_parameters(struct camera_device* dev, const char* parms);

        static char* get_parameters(struct camera_device* dev);

        static void put_parameters(struct camera_device* dev, char* params);

        static int send_command(struct camera_device* dev, int32_t cmd,
                                int32_t arg1, int32_t arg2);

        static void release(struct camera_device* dev);

        static int dump(struct camera_device* dev, int fd);

        static int closeDevice(struct hw_device_t* device);


    /****************************************************************************
     * Preview management.
     ***************************************************************************/

    protected:
        /* Starts preview.
         * Note that when this method is called mPreviewWindow may be NULL,
         * indicating that framework has an intention to start displaying video
         * frames, but didn't create the preview window yet.
         * Return:
         *  NO_ERROR on success, or an appropriate error status on failure.
         */
        virtual status_t doStartPreview();

        /* Stops preview.
         * This method reverts DoStartPreview.
         * Return:
         *  NO_ERROR on success, or an appropriate error status on failure.
         */
        virtual status_t doStopPreview();

    /****************************************************************************
     * Private API.
     ***************************************************************************/

    protected:
        /* Cleans up camera when released. */
        virtual status_t cleanupCamera();


    private:
        /* Registered callbacks implementing camera API. */
        static camera_device_ops_t      mDeviceOps;

    protected:
        /* These variables are available in inherited classes. */

        /* Locks this instance for parameters, state, etc. change. */
        Mutex                           mObjectLock;

        /* Camera parameters. */
        CameraParameters                mParameters;

        /* Preview window. */
        PreviewWindow                   mPreviewWindow;

        /* Callback notifier. */
        CallbackNotifier                mCallbackNotifier;

        /* Zero-based ID assigned to this camera. */
        int                             mCameraID;


    /*********************************************************************
     * Common string values
     *********************************************************************/

    public:
        static const char KEY_FACING[];
        static const char KEY_ORIENTATION[];
        static const char KEY_ISO[];

        /* Possible values for FACING_KEY */
        static const char FACING_BACK[];
        static const char FACING_FRONT[];

};

}

#endif /* HAL_CAMERA_H */
