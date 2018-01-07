#include "Video.IDSCamera.h"
#include "Video.IDSConfigFileTemplate.h"


//#include "Log/Log.h"

#define EVENTTHREAD_WAIT_TIMEOUT    1000
#include <iostream>
#include <fstream>
#include <stdio.h>

#include <cstring>
#include <string>


// LOG_DEFINE_LOGGER(Video::IDSCamera::, "Video.IDSCamera")

namespace Video {


IDSCamera::IDSCamera()
{
    hcam = 0;
    m_event = 0;
    m_pCamList = 0;
    m_bRunEventThread = false;

    Configure(0,3,IS_CM_BGR8_PACKED, true, 128.0, 50.0, 0, false, 30.0);
}


void IDSCamera::Configure(int camera_index,int ring_buffer_size, int color_mode, bool enable_auto_shutter, double auto_reference, double auto_speed, int wb_mode, bool enable_edge_enhancement, double fps)
{
    camera_index_ = camera_index;
    ring_buffer_size_ = ring_buffer_size;
    color_mode_ = color_mode;
    enable_auto_shutter_ = enable_auto_shutter;
    auto_reference_  = auto_reference;
    auto_speed_ = auto_speed;
    wb_mode_ = wb_mode;
    enable_edge_enhancement_ = enable_edge_enhancement;
    fps_ = fps;
    fps_wanted_ = fps;
}

CameraStatus    IDSCamera::GetCameraParametersInfo()
{
    int ret;
        LOG_ADD_TO_CALLSTACK;
        LOG_TRACE_THIS_METHOD;
    ret = is_GetAutoInfo(hcam, &m_AutoInfo); // APram auto dispo
        LOG_INFO("\tAutoAbility :" << m_AutoInfo.AutoAbility << " | [ReturnCode]=" << ret);
        switch (m_AutoInfo.AutoAbility)
        {
            case 0x01 :
                LOG_INFO("\t\t-> AutoShutter possible");
                break;
            case 0x02 :
                LOG_INFO("\t\t-> AutoGain possible");
                break;
            case 0x03 :
                LOG_INFO("\t\t-> AutoGain and AutoShutter possible");
                break;
            case 0x04 :
                LOG_INFO("\t\t-> AutoWhiteBalance possible");
                break;
            default:
                break;
        }
        LOG_INFO("\tAuto Bright Status :");
        LOG_INFO("\t\tcurValue :" << m_AutoInfo.sBrightCtrlStatus.curValue);
        LOG_INFO("\t\tcurError :" << m_AutoInfo.sBrightCtrlStatus.curError);
        LOG_INFO("\t\tcurControllelr :" << m_AutoInfo.sBrightCtrlStatus.curController);
        LOG_INFO("\t\tcurValue :" << m_AutoInfo.sBrightCtrlStatus.curValue);
        LOG_INFO("\tAuto WB Status :");
        LOG_INFO("\tcurController :" << m_AutoInfo.sWBCtrlStatus.curController);

	is_GetSensorInfo(hcam, &sensor_info_);
        LOG_INFO("Sensor Info:");
        LOG_INFO("\tSensor Name :" << sensor_info_.strSensorName);
        LOG_INFO("\tColor Mode :" << +sensor_info_.nColorMode);
        LOG_INFO("\tWidth :" << sensor_info_.nMaxWidth);
        LOG_INFO("\tHeight :" << sensor_info_.nMaxHeight);
        LOG_INFO("\tGlobal shutter ( as opposed to rolling shutter) :" << sensor_info_.bGlobShutter);
        LOG_INFO("\tMaster Gain applied :" << sensor_info_.bMasterGain);
        LOG_INFO("\tRed Gain applied :" << sensor_info_.bRGain);
        LOG_INFO("\tGreen Gain applied :" << sensor_info_.bGGain);
        LOG_INFO("\tBlue Gain applied :" << sensor_info_.bBGain);

        sensor_rect_.Left = 0;
        sensor_rect_.Top = 0;
        sensor_rect_.Width = sensor_info_.nMaxWidth;
        sensor_rect_.Height = sensor_info_.nMaxHeight;
        LOG_RECT(INFO, "SENSOR RECT ", sensor_rect_);
    return CameraStatus::OK;
}


CameraStatus    IDSCamera::SelectCamera(DWORD camIdx)
{
    // m_pCamList should be initialized
    if ((m_pCamList == NULL) || (m_pCamList->dwCount < (camIdx+1)))
    {
        LOG_ERROR("Tried to select a non-existent camera : " << camIdx );
        return CameraStatus::CameraNotFound;
    }
    // copy current camera info
    memcpy (&m_CameraInfo, &m_pCamList->uci[camIdx], sizeof(UEYE_CAMERA_INFO));
    hcam = reinterpret_cast<HIDS>(m_CameraInfo.dwCameraID);
        LOG_INFO("CameraInfo at index : " << camIdx );
        LOG_INFO( "\tSerial Number : " << m_CameraInfo.SerNo);
        LOG_INFO( "\tModel : " << m_CameraInfo.Model);
        LOG_INFO( "\tCamera ID : "<<  m_CameraInfo.dwCameraID);
        LOG_INFO( "\tDevice ID : " << m_CameraInfo.dwDeviceID);
        LOG_INFO( "\tSensor ID : " << m_CameraInfo.dwSensorID);
    int retStatus =  is_InitCamera(&hcam,NULL);
        LOG_DEBUG("Initilized camera and got return code : "<< retStatus);
    int busSpeed = is_GetBusSpeed (hcam);
        LOG_DEBUG("Bus Speed "<< busSpeed);
    GetCameraParametersInfo();
    return CameraStatus::OK;
}

CameraStatus    IDSCamera::GetCameraListAndSelectCamera()
{
    CameraStatus ret = CameraStatus::OK;
    m_pCamList = new UEYE_CAMERA_LIST;
    m_pCamList->dwCount = 0;
    if (is_GetCameraList (m_pCamList) == IS_SUCCESS)
    {
        DWORD dw = m_pCamList->dwCount;
        delete m_pCamList;
        // Reallocate the required camera list size
        m_pCamList = reinterpret_cast<PUEYE_CAMERA_LIST>(new char[sizeof(DWORD) + dw * sizeof(UEYE_CAMERA_INFO)]);
        m_pCamList->dwCount = dw;
        // Get CameraList and store it ...
        if (is_GetCameraList (m_pCamList) == IS_SUCCESS)
        {
            LOG_DEBUG("Found " <<  m_pCamList->dwCount << " cameras" );
            if (m_pCamList->dwCount <= camera_index_)
            {
                LOG_ERROR("Unable to select camera # " <<  camera_index_ << " ,  " << m_pCamList->dwCount<< " cameras detected." );
                return CameraStatus::CameraNotFound;
            }
            ret = SelectCamera(camera_index_);
        }
    }
    return ret;
}

CameraStatus    IDSCamera::InitializeIDSCameraParameters()
{
    EXECUTE_VIDEO_OPERATION_AND_RETURN_IF_KO(GetCameraListAndSelectCamera())
    EXECUTE_VIDEO_OPERATION_AND_RETURN_IF_KO(SetCameraConfiguration())
    EXECUTE_VIDEO_OPERATION_AND_RETURN_IF_KO(EstimateAcquisitionRect())
    EXECUTE_VIDEO_OPERATION_AND_RETURN_IF_KO(InitializeMemoryBuffers())
    return CameraStatus::OK;
}

CameraStatus IDSCamera::OpenCamera()
{
    std::vector<Rect>   roi;
    return OpenCamera(roi);
}

CameraStatus IDSCamera::OpenCamera(std::vector<Rect>& rois)
{
     //assign rois to internal list
     ROIS_.clear();
     ROIS_ = rois;
     // apply camera parameters and isntanciate buffers
     return InitializeIDSCameraParameters();
}

CameraStatus IDSCamera::CloseCamera()
{
    DeInitializeMemoryBuffers();
    int ret = is_ExitCamera(hcam);
        LOG_DEBUG("is_ExitCamera returned " << ret);
    return CameraStatus::OK;
}

CameraStatus IDSCamera::SetCameraConfiguration(){
	LOG_DEBUG("Set Camera configuration parameters ");
	int nRet;

    //Configure file
    /*
    //enable_auto_shutter
    size_t f = ids_config_template_.find("#ENABLE_AUTO_SHUTTER#");
    ids_config_template_.replace(f, std::string("#ENABLE_AUTO_SHUTTER#").length(), enable_auto_shutter_?"1":"0");

    //auto_reference
    f = ids_config_template_.find("#AUTO_REFERENCE#");
    ids_config_template_.replace(f, std::string("#AUTO_REFERENCE#").length(), std::to_string(auto_reference_));

    //auto_speed
    f = ids_config_template_.find("#AUTO_SPEED#");
    ids_config_template_.replace(f, std::string("#AUTO_SPEED#").length(), std::to_string(auto_speed_));

    //color mode

    f = ids_config_template_.find("#COLOR_MODE#");
    ids_config_template_.replace(f, std::string("#COLOR_MODE#").length(), std::to_string(color_mode_));
    */
    // write updated parameter file
    std::string camera_config_file_to_upload_ ="/tmp/camera_config_uploaded.ini";
    std::ofstream cfg_out_file;
    cfg_out_file.open(camera_config_file_to_upload_);
    cfg_out_file << ids_config_template_;
    cfg_out_file.close();

    auto path = L"/tmp/camera_config_uploaded.ini";
    nRet = is_ParameterSet(hcam, IS_PARAMETERSET_CMD_LOAD_FILE,  const_cast<void*>(static_cast<const void*>(path)),0);
    LOG_DEBUG("Uploaded parmeter set from " << path << " . Return status " << nRet);
    /*
    // DsiableIPO threads
    UINT nAllowIpo = IS_CONFIG_IPO_NOT_ALLOWED;
    nRet = is_Configuration(IS_CONFIG_IPO_CMD_SET_ALLOWED, (void*)&nAllowIpo , sizeof(nAllowIpo));
    LOG_DEBUG("Tried to dsiable IPO. command tried to set value for IPO_CMD_GET_ALLOWED =  "<< nAllowIpo);
    int is_ipo_allowed;
    nRet = is_Configuration(IS_CONFIG_IPO_CMD_GET_ALLOWED, (void*)&is_ipo_allowed , sizeof(is_ipo_allowed ));
    LOG_DEBUG("Tried to dsiable IPO. After command, IPO_CMD_GET_ALLOWED returned "<< is_ipo_allowed);
    */

    switch (color_mode_)
    {
	   case IS_CM_SENSOR_RAW8:
		  m_nBitsPerPixel = 8;
          break;
        case  IS_CM_BGR8_PACKED:
		  m_nBitsPerPixel = 24;
          break;
        default:
          m_nBitsPerPixel = 24;
          break;
	}

	/*
    // set color mlode in sequence ;: first raw 12 to enable hardware debyaer , then wanted color mode
    for (auto cm : {IS_CM_SENSOR_RAW12, color_mode_})
    {
        nRet = is_SetColorConverter (hcam, cm, IS_CONV_MODE_HARDWARE_3X3);
        LOG_DEBUG("Status SetColorConverter hardware DeBayer " << IS_CONV_MODE_HARDWARE_3X3 << " for colorMode " << +cm <<". Return status " << nRet);

        nRet = is_SetColorMode(hcam, cm);
        LOG_DEBUG("Status SetColorMode " << cm << "  .BitsPerPixel= " << m_nBitsPerPixel <<   ".Return status" << nRet);

        nRet = is_SetColorConverter (hcam, cm, IS_CONV_MODE_HARDWARE_3X3);
        LOG_DEBUG("Status SetColorConverter hardware DeBayer " << IS_CONV_MODE_HARDWARE_3X3 << " for colorMode " << +cm << ". Return status " << nRet);

    }

    //set AOI
	//Passage en mode BITMAPxx
	//INT displayMode = IS_SET_DM_DIB;
	//nRet = is_SetDisplayMode(hcam, displayMode);
	//LOG_DEBUG("Status displayMode "<< displayMode << ".Return status" << nRet);
	SetFPS(fps_wanted_);
	// Automatic Exposure Control
	double max_auto_gain = 100.0;
    nRet = is_SetAutoParameter(hcam, IS_SET_AUTO_GAIN_MAX, &max_auto_gain, 0);
    LOG_DEBUG("Set AEC max gain " << max_auto_gain << ".Return status " << nRet);
    double max_auto_exp = 165.0;
    nRet = is_SetAutoParameter(hcam, IS_SET_AUTO_SHUTTER_MAX, &max_auto_exp, 0);
    LOG_DEBUG("Set AEC max exp " << max_auto_exp << ".Return status " << nRet);
    double auto_histeresis = 2.0;
    nRet = is_SetAutoParameter(hcam, IS_SET_AUTO_HYSTERESIS, &auto_histeresis, 0);
    LOG_DEBUG("Set AEC hysteresis " << auto_histeresis << ".Return status " << nRet);



    double dEnable = enable_auto_shutter_ ? 1 : 0;
	nRet = is_SetAutoParameter(hcam, IS_SET_ENABLE_AUTO_SHUTTER, &dEnable, 0);
	LOG_DEBUG("Set AEC COnfig Auto " << dEnable << ".Return status " << nRet);
    nRet = is_SetAutoParameter(hcam, IS_SET_ENABLE_AUTO_SENSOR_GAIN_SHUTTER, &dEnable, 0);
    LOG_DEBUG("Set AEC COnfig Auto sensor gain shutter " << dEnable << ".Return status " << nRet);
	//Set brightness setpoint to 60:
	nRet = is_SetAutoParameter(hcam, IS_SET_AUTO_REFERENCE, &auto_reference_, 0);
	LOG_DEBUG("Status AEC Reference " << auto_reference_ << ".Return status " << nRet);

    nRet = is_SetAutoParameter(hcam, IS_SET_AUTO_SPEED, &auto_speed_, 0);
    LOG_DEBUG("Status AEC Speed " << auto_speed_ << ".Return status " << nRet);


    CHAR *pBuffer = new char[sizeof(AES_CONFIGURATION) - sizeof(CHAR) + sizeof(AES_PEAK_WHITE_CONFIGURATION)];
    AES_CONFIGURATION *pAesConfiguration = (AES_CONFIGURATION*)pBuffer;
    pAesConfiguration->nMode = IS_AES_MODE_PEAK;
    AES_PEAK_WHITE_CONFIGURATION *pPeakWhiteConfiguration = (AES_PEAK_WHITE_CONFIGURATION*)pAesConfiguration->pConfiguration;
    pPeakWhiteConfiguration->nReference = 65;
    pPeakWhiteConfiguration->nHysteresis = 2;
    nRet = is_AutoParameter(hcam, IS_AES_CMD_SET_CONFIGURATION, pAesConfiguration , sizeof(AES_CONFIGURATION) - sizeof(CHAR) + sizeof(AES_PEAK_WHITE_CONFIGURATION));
    LOG_DEBUG("Send AES configuration. Return status " << nRet);

    INT nEnable = IS_AUTOPARAMETER_ENABLE;
    nRet = is_AutoParameter(hcam, IS_AES_CMD_SET_ENABLE, &nEnable, sizeof(nEnable));
    LOG_DEBUG("Enable AES .Return status " << nRet);



    //nRet = is_SetAutoParameter(hcam, IS_SET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &wb_mode_, 0);
    //LOG_DEBUG("Set WB AutoConfig to  "<<  wb_mode_ << ".Return status " << nRet);

// disable edge enhancement
    UINT nEdgeEnhancement = enable_edge_enhancement_ ? 1 :0;
    nRet = is_EdgeEnhancement(hcam,IS_EDGE_ENHANCEMENT_CMD_SET, (void*)&nEdgeEnhancement, sizeof(nEdgeEnhancement) );
    LOG_DEBUG("Enable/Disable Edge Enhancement " << nEdgeEnhancement << " . Return status " << nRet);

    //GetCameraParametersInfo();

    // @toDo : add as parameters
    nRet = is_SetColorCorrection (hcam, IS_CCOR_ENABLE_NORMAL,NULL);
    LOG_DEBUG("Szet color correction " << IS_CCOR_ENABLE_NORMAL << ".Return status" << nRet);
    */


    //INIT MORPHO CUSTOM AEC
    if(!enable_auto_shutter_){

     double expo_min_ms, expo_max_ms, expo_current_ms;
     GetIntegrationTimeMax(expo_max_ms);
     GetIntegrationTimeMin(expo_min_ms);
     expo_current_ms = expo_min_ms +(expo_max_ms - expo_min_ms)/3. ;
     AEC_configure(expo_current_ms, auto_reference_, expo_max_ms, expo_min_ms,  auto_speed_ );
     LOG_DEBUG("Send MORPHO AES configuration : \n\t min Exposure Time : "<< expo_min_ms << " ms \n\t max Exposure Time :  " << expo_max_ms <<" ms \n\t ==> Exposure Init :" << expo_current_ms <<" ms");
    }
     // SET First Integration Time
     //SetIntegrationTime(expo_current_ms);
     //LOG_DEBUG("Set manual exposure to  : " << expo_current_ms <<" ms , return " << nRet );

     SaveCameraConfigToFile();
    // parameters might not be all ok but camera can still be operated
    return CameraStatus::OK;
}

CameraStatus IDSCamera::SaveCameraConfigToFile()
{

    // save parameters to /tmp/s-trafficlight
    auto out_path = L"/tmp/camera_config_downloaded.ini";
    int nRet = is_ParameterSet(hcam, IS_PARAMETERSET_CMD_SAVE_FILE,  const_cast<void*>(static_cast<const void*>(out_path)),0);
    LOG_DEBUG("Saved parmeter set to /tmp/s-trafficlight/camera_config_dowloaded.ini . Return status " << nRet);
    return CameraStatus::OK;

}

CameraStatus IDSCamera::SetFPS(double fps)
{
    double min_frame_time = 0.0;
    double max_frame_time = 0.0;
    double intervall  = 0.0;

    int ret = is_GetFrameTimeRange (hcam, &min_frame_time, &max_frame_time, &intervall);
    double minfps = floor(100.0/max_frame_time)/100.0;
    double maxfps = floor(100.0/min_frame_time)/100.0;
    LOG_INFO("Get Camera fps range :  " << minfps << " - "<<maxfps<< ")  . return = "<< ret);
	ret = is_SetFrameRate(hcam, fps, &fps_);
    LOG_INFO("Set Camera fps to " << fps_ << " (wanted="<<fps<< ")  . return = "<< ret);
    return CameraStatus::OK;
}
const std::vector<std::shared_ptr<const Image>>& IDSCamera::GetROIS()
{
    return output_const_images_;
}

CameraStatus IDSCamera::WaitForFrame( std::function<void(Camera&,void* param_obj)> callback, void* param_obj)
{
    LOG_ADD_TO_CALLSTACK;
    LOG_TRACE_THIS_METHOD;
    int ret = 0;

    m_event = IS_SET_EVENT_FRAME;
    ret = is_EnableEvent (hcam, m_event);
    LOG_DEBUG("\tEnabled Event " << ret);
    if (ret == IS_SUCCESS)
    {
        ret = is_CaptureVideo(hcam, IS_DONT_WAIT);
            LOG_DEBUG("Start Video Capture " << ret);
        m_bRunEventThread = true;
        while (m_bRunEventThread && (ret == IS_SUCCESS))
        {
            if (is_WaitEvent (hcam, m_event, EVENTTHREAD_WAIT_TIMEOUT) == IS_SUCCESS)
            {
                     //LOG_DEBUG("\tGot frame from async wait" );
                TimeStamp acquisition_timestamp = TimeStamp::Now();
                // get active ids buffer
                char* foo = NULL;
                char* foo_last = NULL;
                int cur_active_frame=0;
                ret = is_GetActSeqBuf(hcam, &cur_active_frame, &foo, &foo_last);
                    // LOG_DEBUG("\tGot frame, active buffer is " << cur_active_frame << " GetActiveSeqBuf returned "<< ret);
                if (ret == IS_SUCCESS)
                {
                    ret = is_LockSeqBuf (hcam, cur_active_frame, foo);
                        // LOG_DEBUG("\t Lock active buffer returned "<< ret);
                    if (ret == IS_SUCCESS)
                    {
                        // fill the rois that will be sent to algorithm
                        for (auto it = output_images_.begin(); it != output_images_.end();++it)
                        {
                            (*it)->CopyFrom(*ids_ring_buffer_[cur_active_frame-1]);
                            (*it)->update_timestamp(acquisition_timestamp);
                        }
                        // release the ids buffer
                        ret = is_UnlockSeqBuf (hcam, cur_active_frame, foo);
                          // LOG_DEBUG("\t UnLocked active buffer "<< ret);

                        // call the user code that will use the callback
                             //LOG_DEBUG("\tGotFrame Callback");
                        callback(*this,param_obj);
                    }
                }
            }
            else
            {
                LOG_DEBUG("\tDidn't get Frame before TIMEOUT");
            }
        }
        is_StopLiveVideo (hcam, IS_WAIT);
    }
    is_DisableEvent(hcam,m_event);
    return CameraStatus::OK;
}

CameraStatus    IDSCamera::SetAutoBrightnessAOI(Video::Rect& r)
{
    return CameraStatus::OK;
    int ret;
    IS_RECT rectAOI;
    rectAOI.s32X     = r.Left;
    rectAOI.s32Y     = r.Top;
    rectAOI.s32Width =  r.Width;
    rectAOI.s32Height = r.Height;

    ret = is_AOI( hcam, IS_AOI_AUTO_BRIGHTNESS_SET_AOI, static_cast<void*>(&rectAOI), sizeof(rectAOI));
        LOG_RECT(INFO,"\tSet Auto Brightness camera ROI to ", r);
        LOG_TRACE("\tis_AOI returned " << ret);
    // ensure camera parameters afeter ROI set
    SetFPS(fps_wanted_);
    //is_Exposure()
    return ret==IS_SUCCESS?CameraStatus::OK:CameraStatus::CommandFailed;
}

CameraStatus    IDSCamera::SetAOI(Video::Rect& r)
{
    int ret;
    IS_RECT rectAOI;
    rectAOI.s32X     = r.Left;
    rectAOI.s32Y     = r.Top;
    rectAOI.s32Width =  r.Width;
    rectAOI.s32Height = r.Height;

    ret = is_AOI( hcam, IS_AOI_IMAGE_SET_AOI, static_cast<void*>(&rectAOI), sizeof(rectAOI));
        LOG_RECT(INFO,"\tSet camera ROI to ", r);
        LOG_TRACE("\tis_AOI returned " << ret);

    // ensure camera parameters afeter ROI set
    SetFPS(fps_wanted_);
    //is_Exposure()
    return ret==IS_SUCCESS?CameraStatus::OK:CameraStatus::CommandFailed;

}


CameraStatus    IDSCamera::SetIntegrationTime(double& integrationTime_ms){
    int ret;

    ret = is_Exposure (hcam, IS_EXPOSURE_CMD_SET_EXPOSURE, &integrationTime_ms, sizeof(integrationTime_ms));
    //LOG_TRACE("\tis_Exposure ( set ) returned " << ret << "\tTint(ms)= " << integrationTime_ms );
    //ret = is_SetHardwareGain (hcam, 0 , 0, 0, 0);
    //LOG_TRACE("\tis_SetHardwareGain " << ret);
    return ret==IS_SUCCESS?CameraStatus::OK:CameraStatus::CommandFailed;

}
CameraStatus    IDSCamera::GetIntegrationTimeMax(double& integrationTimeMax_ms){
    int ret;
    ret = is_Exposure (hcam, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX, &integrationTimeMax_ms, sizeof(integrationTimeMax_ms));

    LOG_TRACE("\tis_Exposure (read max) returned " << ret);
    return ret==IS_SUCCESS?CameraStatus::OK:CameraStatus::CommandFailed;
}
CameraStatus    IDSCamera::GetIntegrationTimeMin(double& integrationTimeMin_ms){
    int ret;


    ret = is_Exposure (hcam, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN, &integrationTimeMin_ms, sizeof(integrationTimeMin_ms));
    LOG_TRACE("\tis_Exposure (read min) returned " << ret);


    return ret==IS_SUCCESS?CameraStatus::OK:CameraStatus::CommandFailed;
}





CameraStatus    IDSCamera::EstimateAcquisitionRect()
{
    LOG_ADD_TO_CALLSTACK;
    LOG_TRACE_THIS_METHOD;

    // ROI constraints
    //@ Todo : read cmaera constraints directly from camera
    int width_padding = 8;
    int height_padding = 2;
    unsigned short min_roi_width = 88;
    unsigned short min_roi_height = 28;

    if (ROIS_.size() == 0)
    {
        acquisition_rect_ = sensor_rect_;
        ROIS_.push_back(sensor_rect_);
    }
    else
    {
        unsigned short int left = sensor_rect_.Width;
        int right = 0;
        unsigned short int top = sensor_rect_.Height;
        int bottom = 0;
        for (auto it = ROIS_.begin(); it != ROIS_.end(); ++it)
        {
            LOG_RECT(DEBUG, "Debug ROI rect ",*it);
            left = std::min(left, it->Left);
            right = std::max(right, it->Left+it->Width);
            top = std::min(top, it->Top);
            bottom = std::max(bottom, it->Top + it->Height);
        }
        acquisition_rect_.Left = left;
        acquisition_rect_.Width = right-left;
        acquisition_rect_.Top = top;
        acquisition_rect_.Height = bottom-top;
    }
    // Adjust acquisition rect with padding cosntraints
    // position of aoi should be padded
    int left_adjust = acquisition_rect_.Left % 4;
    int top_adjust = acquisition_rect_.Top % 2;
    acquisition_rect_.Left -= left_adjust;  // align on allowed pixel
    acquisition_rect_.Width += left_adjust; // increase width to keep requestd roi
    acquisition_rect_.Top -= top_adjust; // align on allowed pixel
    acquisition_rect_.Height += top_adjust; // increase height to keep requestd roi

    // width and hieght should be padded
    int width_adjust = acquisition_rect_.Width % width_padding;
    int height_adjust = acquisition_rect_.Height % height_padding;
    if (width_adjust > 0)
    {
        acquisition_rect_.Width += (width_padding - width_adjust);
    }
    if (height_adjust > 0)
    {
        acquisition_rect_.Height += (height_padding - height_adjust);
    }
    // adjust min of with and height
    acquisition_rect_.Width = std::max(acquisition_rect_.Width, min_roi_width);
    acquisition_rect_.Height = std::max(acquisition_rect_.Height, min_roi_height);

    LOG_RECT(INFO, "Estimated Acquisition rect to ",acquisition_rect_);
    CameraStatus ret_aoi = SetAOI(acquisition_rect_);

    //SET exposure controle AOI
    LOG_RECT(INFO, "Brightness  set to the Acquisition rect",acquisition_rect_);
    CameraStatus ret_Baoi= SetAutoBrightnessAOI(acquisition_rect_);
    if(ret_Baoi !=CameraStatus::OK){
        LOG_RECT(ERROR, "Brightness  set to the Acquisition rect FAILED",acquisition_rect_);
    }
     return ret_aoi;
}

CameraStatus    IDSCamera::DeInitializeMemoryBuffers()
{
        LOG_ADD_TO_CALLSTACK;
        LOG_TRACE_THIS_METHOD;
    int ret;
    ret = is_ClearSequence(hcam);
        LOG_DEBUG("\tClear Sequence returned "<< ret);
    for (auto it=ids_ring_buffer_.begin(); it != ids_ring_buffer_.end(); ++it)
    {
        ret = is_FreeImageMem(hcam,reinterpret_cast<char*>((*it)->GetBufferUnsafe()), (*it)->GetImageID());
            LOG_DEBUG("\tFree Image " << (*it)->GetImageID() << " returned "<< ret);
        (*it).reset();
    }
    ids_ring_buffer_.clear();
    /*
    for (auto it=output_images_.begin(); it != output_images_.end(); ++it)
    {
        (*it).reset();
    }*/
    output_images_.clear();
    output_const_images_.clear();

    return CameraStatus::OK;
}

CameraStatus IDSCamera::InitializeMemoryBuffers()
{
    LOG_ADD_TO_CALLSTACK;
    LOG_TRACE_THIS_METHOD;
    int ret;
    //Ensure buffers are cleared
    DeInitializeMemoryBuffers();
    //Allocate
    for (int idx = 0; idx <ring_buffer_size_; ++idx )
    {
        char* mem;
        int   memID;
        ret = is_AllocImageMem(hcam, acquisition_rect_.Width,	acquisition_rect_.Height, m_nBitsPerPixel, &mem, &memID);
        auto img = std::make_shared<Image>(mem,acquisition_rect_,GetSensorBytesPerPixel(),memID);
        ids_ring_buffer_.push_back(img);
    	    LOG_DEBUG("\tStatus AllocImage "<< ret << "  BitsPerPixel="<<m_nBitsPerPixel);
        ret = is_AddToSequence(hcam,mem,memID);
            LOG_DEBUG("\tStatus AddToSequence "<< ret);
    }
    for (auto it = ROIS_.begin(); it != ROIS_.end(); ++it )
    {
        auto img = std::make_shared<Image>(*it, GetSensorBytesPerPixel());
        output_images_.push_back(img);
        output_const_images_.push_back(std::shared_ptr<const Image>(img));
    }
	return CameraStatus::OK;
}

unsigned int IDSCamera::GetCameraIndex()
{
    return camera_index_;
}

unsigned int IDSCamera::GetSensorWidth()
{
    return sensor_info_.nMaxWidth;
}

unsigned int IDSCamera::GetSensorHeight()
{
    return sensor_info_.nMaxHeight;
}

unsigned char IDSCamera::GetSensorBytesPerPixel()
{
    return (m_nBitsPerPixel+7)/8;
}

CameraStatus IDSCamera::SetEventTrigger(bool val)
{
    m_bRunEventThread = val;
    return CameraStatus::OK;
}

}
