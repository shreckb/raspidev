#pragma once

#include "Video.Camera.h"
#include <ueye.h>

#define IDS_BUFFERS 3 //triple buffering

namespace Video{

class IDSCamera : public Camera
{
public:
    IDSCamera();
    void Configure( int camera_index_,
                    int ring_buffer_size, int color_mode, bool enable_auto_shutter,
                     double auto_reference, double auto_speed, int wb_mode,
                     bool enable_edge_enhancement, double fps) override;

    CameraStatus OpenCamera() override;
    CameraStatus OpenCamera(std::vector<Rect>& rois) override;
    CameraStatus CloseCamera() override;

    CameraStatus WaitForFrame(std::function<void(Camera&,void* param_obj)> , void* paramObj) override;
    const std::vector<std::shared_ptr<const Image>>& GetROIS() override;


    CameraStatus    GetCameraParametersInfo();

    unsigned int GetCameraIndex() override;
    unsigned int GetSensorWidth() override;
    unsigned int GetSensorHeight() override;
    unsigned char GetSensorBytesPerPixel() override;


    CameraStatus SetEventTrigger(bool val) override;
    CameraStatus SetFPS(double fps) override;

    CameraStatus    SetAutoBrightnessAOI(Video::Rect& r) override;
    CameraStatus    SetIntegrationTime      (double& integrationTime_ms)    override;
    CameraStatus    GetIntegrationTimeMax   (double& integrationTimeMax_ms) override;
    CameraStatus    GetIntegrationTimeMin   (double& integrationTimeMin_ms) override;

    CameraStatus    SaveCameraConfigToFile() override;


private:
    CameraStatus    GetCameraListAndSelectCamera();
    CameraStatus    InitializeIDSCameraParameters();

    CameraStatus    DeInitializeMemoryBuffers();
    CameraStatus    InitializeMemoryBuffers();

    CameraStatus    SelectCamera(DWORD camIdx);
    CameraStatus    SetCameraConfiguration();

    CameraStatus    EstimateAcquisitionRect();
    CameraStatus    SetAOI(Video::Rect& r);


    HIDS hcam;

    // cmaera parameters
    unsigned int                                camera_index_;
    int                                         ring_buffer_size_;
    int                                         color_mode_ ;
    bool                                        enable_auto_shutter_ ;
    double                                      auto_reference_;
    double                                      auto_speed_;
    double                                      wb_mode_;
    bool                                        enable_edge_enhancement_;
    double                                      fps_;
    double                                      fps_wanted_;

    // camera acquisition parameters
    std::vector<Rect>                           ROIS_;
    Rect                                        sensor_rect_;
    Rect                                        acquisition_rect_;
    std::vector<std::shared_ptr<Image>>         output_images_;
    std::vector<std::shared_ptr<const Image>>   output_const_images_;
    std::vector<std::shared_ptr<Image>>         ids_ring_buffer_;


    // Camera/ sensor info
    SENSORINFO                                  sensor_info_;




    bool                    isOpen_ = false;

				   // 0 = Mode Config( RGB , Full Frame)
    int m_nBitsPerPixel;
    PUEYE_CAMERA_LIST m_pCamList;
    UEYE_CAMERA_INFO    m_CameraInfo;
    UEYE_AUTO_INFO      m_AutoInfo;

    int m_event;
    volatile bool m_bRunEventThread;

    //char*           ids_capture_memory_buffer[IDS_BUFFERS];
    //int             ids_capture_memory_buffer_ID[IDS_BUFFERS];
};

}
