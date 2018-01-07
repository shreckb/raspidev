#pragma once

#include <iostream>
#include "Video.IDSCamera.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

static void GotVideoFrame(Video::Camera& cam, void* paramObj){

    auto roi = cam.GetROIS()[0];
    const unsigned char* pimg = roi->GetConstBuffer();
    Video::Rect rect = roi->GetRect();

    Mat frame( rect.Height, rect.Width, CV_8UC3, const_cast<unsigned char*>(pimg));

    // FLIP
    cv::flip(frame, frame, -1);

    //cv::imwrite("visage.jpg", frame);
    //cout << "frame VISAGE received("<<rect.Height<<"x"<<rect.Width<< ")" <<  " FPS = "<< GLOBAL_DATA.facedetection_computeFPS() << "sec" << endl;
    int nb_visage = 0;

    double elapseTime = ((double)getTickCount() - GLOBAL_DATA.last_detection_time )/getTickFrequency();
    if(GLOBAL_DATA.facedetection && elapseTime > GLOBAL_DATA.face_detection_freq){
        double time_start = (double)getTickCount();
        cout << "\t\t Face detection ... ";
        nb_visage = GLOBAL_DATA.findFaces(frame);
        double elapsed_time  = 1000.*((double)getTickCount() -time_start )/getTickFrequency();
        cout << ": "<< elapsed_time << " ms - " << nb_visage << " visgae detected" <<endl;
        GLOBAL_DATA.last_detection_time = (double)getTickCount();
        //cout << "\t\t"<<  nb_visage << " visage  detected  in..." << endl;
        //On tri par taille  si plusieurs visage

        if(nb_visage)
            GLOBAL_DATA.followVisage_Camshift_init(frame);

    }

    // Suivi du visage 
    GLOBAL_DATA. followVisage_camShift(frame);

    //Affichage 

    GLOBAL_DATA.drawFaces(frame);

    /*
    if(nb_visage){
      cout << "\t\t"<<  nb_visage << " visage  detected" << endl;
      imwrite("face_detected.jpg", frame);
    }
    */


    

    //GLOBAL_DATA.m_visage = cv::imread("visage.jpeg");
    double zoom_factor = double(GLOBAL_DATA.m_windowViSA_width)/frame.cols;
    resize(frame, GLOBAL_DATA.m_visage_resized, Size(), zoom_factor,zoom_factor,INTER_NEAREST);

    //affichage
    imshow(GLOBAL_DATA.m_windowViSA_name, GLOBAL_DATA.m_visage_resized);
    waitKey(1);



    //STOP the THREAD if needed
    if(GLOBAL_DATA.cameraVisage_running == false){
      cout << "closing camera visage thread" << endl;
      cam.SetEventTrigger(false);
      cam.CloseCamera();

    }

}

void runFaceDetection(){

    // Initialize specific IDS Camera
    Video::IDSCamera ids_camera = Video::IDSCamera();
    // Dereference specific type to simplify later Hardware Camera Change if needed
    Video::Camera& camera = ids_camera;

    //Configure
    int camera_index_       = 0;
    int ring_buffer_size    = 3;
    int color_mode          = 1;
    bool enable_auto_shutter= 1;
    double auto_reference   = 100;
    double auto_speed       = 50;
    int wb_mode             =  0;
    bool enable_edge_enhancement = false;
    double fps                     = 15;
    camera.Configure(camera_index_,ring_buffer_size, color_mode, enable_auto_shutter, auto_reference, auto_speed, wb_mode, enable_edge_enhancement, fps);

    //parametre dans fichier + binning => RECT = "Width=848 - Height=640\n"
    std::vector<Video::Rect> rois;
    rois.resize(1);
    Video::Rect sensorROI;
    rois[0].Left = 0;
    rois[0].Top = 0;
    rois[0].Width = 848;
    rois[0].Height = 640;
    Video::CameraStatus error = camera.OpenCamera(rois);
    if (error != Video::CameraStatus::OK)
        return ;

    cout << "************ Start of Face Detection ************ " << endl;
    auto got_frame_callback = std::function<void(Video::Camera&, void*)> (GotVideoFrame);
    camera.WaitForFrame(got_frame_callback,NULL);

    camera.CloseCamera();
    cout << "************  End of Face Detection ************" << endl;

}
