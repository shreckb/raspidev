#pragma once
#include <raspicam/raspicam_cv.h>
#include <iostream>
using namespace std;


void runIRISDetection(){

  raspicam::RaspiCam_Cv Camera;
  //set camera params
  Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );

  //Open camera
  cout<<"Opening Camera..."<<endl;
  if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return;}


  //SET FPS
  Camera.set(CV_CAP_PROP_FPS, GLOBAL_DATA.m_iris_fps);   //set grabbing fps
  double fps = Camera.get(CV_CAP_PROP_FPS);  //read fps from cam
  cout << "*set IRIS FPS to  " <<fps << endl;
  cout << "************ Start of IRIS Detection ************ " << endl;
  while(GLOBAL_DATA.cameraIRIS_running){
    Camera.grab();
    Camera.retrieve ( GLOBAL_DATA.m_iris);
    //cout << "frame IRIS received("<<GLOBAL_DATA.m_iris.rows<<"x"<<GLOBAL_DATA.m_iris.cols<< ")" <<  " FPS = "<< GLOBAL_DATA.irisdetection_computeFPS() << "sec" << endl;


    double zoom_factor = double(GLOBAL_DATA.m_windowIRIS_width )/GLOBAL_DATA.m_iris.cols;
    resize(GLOBAL_DATA.m_iris, GLOBAL_DATA.m_iris_resized, Size(), zoom_factor,zoom_factor,INTER_NEAREST);

    //affichage
    imshow(GLOBAL_DATA.m_windowIRIS_name, GLOBAL_DATA.m_iris_resized);
    waitKey(1);

  }
  cout << "************  End of IRIS Detection ************" << endl;

}
