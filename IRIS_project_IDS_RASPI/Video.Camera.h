#pragma once

//=============================================================================
/// @file  Video.Camera.h
/// @brief Base class for Camera access
//=============================================================================
#include "Video.Image.h"
#include <vector>
#include <functional>
#include <memory>

//Adaptation for GLobal use (not only  in §fusion)
#define LOG_ADD_TO_CALLSTACK
#define LOG_TRACE_THIS_METHOD
//#define LOG_INFO(x)  {std::cout<< x << std::endl;}
//#define LOG_ERROR(x) {std::cout<< x << std::endl;}
//#define LOG_DEBUG(x) {std::cout<< x << std::endl;}
//#define LOG_TRACE(x) {std::cout<< x << std::endl;}

#define EXECUTE_VIDEO_OPERATION_AND_RETURN_IF_KO(x) {auto ret = x; if (ret != Video::CameraStatus::OK){return ret;}}

namespace Video{

enum class CameraStatus
{
    OK,
    CameraNotFound,
    InvalidParameter,
    CommandFailed,
    UnableToAcquireFrame,
    UnknownError
};

/// @brief Convert an internal step of the "Camera state" into a string
/// @throw std::invalid_argument
inline std::string ToString(CameraStatus camera_status) {
    switch (camera_status) {
        case CameraStatus::OK                   : return "OK";
        case CameraStatus::CameraNotFound       : return "Camera not found";
        case CameraStatus::InvalidParameter     : return "Invalid parameter";
        case CameraStatus::CommandFailed        : return "Command failed";
        case CameraStatus::UnableToAcquireFrame : return "Unable to acquire frame";
        case CameraStatus::UnknownError         : return "Unknown error";
    };
    throw std::invalid_argument{"invalid camera status = "
            + std::to_string(static_cast<int>(camera_status))};
}


class Camera : public std::enable_shared_from_this<Camera>
{
public:
    virtual ~Camera();

    // define camera parameters that will be applied on open
    virtual void Configure( int camera_index_,
                    int ring_buffer_size, int color_mode, bool enable_auto_shutter,
                     double auto_reference, double auto_speed, int wb_mode,
                     bool enable_edge_enhancement, double fps) = 0;

    // open camera and apply parameters
    virtual CameraStatus OpenCamera() = 0;
    virtual CameraStatus OpenCamera(std::vector<Rect>& rois) = 0;
    virtual CameraStatus CloseCamera() = 0;
    virtual CameraStatus SaveCameraConfigToFile() = 0;

    // loop on event wait to acquire frames
    // callback is called upon frame arrival
    // paramObj allows callback to store context/state
    virtual CameraStatus WaitForFrame(std::function<void(Camera&,void* param_obj)> , void* paramObj) = 0;
    // enable or disable event trigger, disable cancels waitforframe after eventtimeout
    virtual CameraStatus SetEventTrigger(bool enable) = 0;
    // Set auto brightness AOI zone
    virtual CameraStatus    SetAutoBrightnessAOI(Video::Rect& r) = 0;
    // Set Integration Timevalue (in ms)
    virtual CameraStatus SetIntegrationTime     (double& integrationTime_ms) = 0; //{ return CameraStatus::OK;}
    virtual CameraStatus GetIntegrationTimeMax  (double& integrationTimeMax_ms) = 0;
    virtual CameraStatus GetIntegrationTimeMin  (double& integrationTimeMin_ms) = 0;


    //AEC (Integration time Only, No Gain)
    void         AEC_configure(double expo_init_ms, double expo_consigne, double expo_max_ms,double expo_min_ms,  double expo_asservissementK );
    void         AEC_compute_error(double moy_actuelle);
    double       AEC_compute_expo();
    double       AEC_update(double moy_actuelle);



    virtual const std::vector<std::shared_ptr<const Image>>& GetROIS() = 0;

    virtual unsigned int GetCameraIndex() = 0;
    virtual unsigned int GetSensorWidth() = 0;
    virtual unsigned int GetSensorHeight() = 0;
    virtual unsigned char GetSensorBytesPerPixel() = 0;

    virtual CameraStatus SetFPS(double fps) = 0;

    static std::shared_ptr<Camera>   CreateCamera();

    // AEC
    double AEC_accumulator_             = 0.;
    double AEC_Tint_init_ms_            = 15.;
    double AEC_expo_asservissementK_    = 1.;
    double AEC_consigne_                = 120.;
    double AEC_expo_max_ms_             = 33.;
    double AEC_expo_min_ms_             = 0.;
    double AEC_threshold_up_            = 0.;
    double AEC_threshold_down_          = 0.;

    double AEC_expo_step_max_           = 2.;
    int    AEC_decimation_              = 6;
    int    AEC_counter_                 = 0;
    double AEC_current_expo_            = AEC_Tint_init_ms_;
    double AEC_tolerance_              =  0.1;



};

} // namespace Video
