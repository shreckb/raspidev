#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>

using namespace cv;

bool compareRectangleBySize(const cv::Rect &R1, const cv::Rect &R2) {
    return (R1.height * R1.width) > (R2.height * R2.width);
}



class myApp
{
public:

    myApp();


    //thread
    bool cameraVisage_running = true;
    bool cameraIRIS_running   = true;

    // Face Detection
    bool facedetection = true; // On  lance ou pas la face detection => gerer dasn le main
    bool init_faceDetection(std::string xml_find);
    int findFaces(cv::Mat  frame_bgr);
    void drawFaces(cv::Mat &frame_bgr);
    double facedetection_computeFPS(){
        double time = (double)getTickCount();
        double fps = (time - facedetection_fps)/getTickFrequency();
        facedetection_fps = time;
        return fps;
    }
    cv::Mat m_visage_gray;
    cv::Mat m_visage_resized;
    //gestion  detection toute les N secondes
    double last_detection_time =0;
    double face_detection_freq = 10.; // time en second



    //Affichage (5" HMI : 800x480)
    std::string m_windowViSA_name = "VISAGE";
    std::string m_windowIRIS_name = "IRIS";
    int m_windowViSA_width = 800; //800/2;
    int m_windowIRIS_width = 800/2;

    // IRIS DETECTION
    double m_iris_fps = 25.0;
    double irisdetection_computeFPS(){
        double time = (double)getTickCount();
        double fps = (time - irisdetection_fps)/getTickFrequency();
        irisdetection_fps = time;
        return fps;
    }
    cv::Mat m_iris;
    cv::Mat m_iris_resized;

    //FACE FOLOWING : CAM SHIFT
    bool camshift_just_init = false;
    cv::Rect camshift_startingWindows;
    cv::RotatedRect camshift_trackBox;
    cv::Mat camshitf_hsv, camshift_hue, camshift_mask, camshift_hist ,camshift_backproj;
    int camshift_hsize = 16;
    float camshift_hranges[2] = {0,180};
    const float* camshift_phranges = camshift_hranges;
    cv::Point origin;
    int camshitf_vmin = 10, camshitf_vmax = 256, camshitf_smin = 30;
    void followVisage_Camshift_init(cv::Mat frame_bgr);
    void followVisage_camShift(cv::Mat frame_bgr);
    

private:
    //face detection
    cv::CascadeClassifier face_cascade;
    std::vector<Rect> faces;
    bool is_FaceDetection_initialized = false;

    //
    double facedetection_fps;
    double irisdetection_fps;



};

myApp::myApp(){
  is_FaceDetection_initialized = init_faceDetection("haarcascade_frontalface_default.xml");
  if(!is_FaceDetection_initialized && facedetection){
        //On stop les thread 
        cameraVisage_running = false;
        cameraIRIS_running   = false;
  }

  //positionne les fenetre
  cv::namedWindow(m_windowViSA_name);
  cv::moveWindow(m_windowViSA_name, 0,10);
  cv::namedWindow(m_windowIRIS_name);
  cv::moveWindow(m_windowIRIS_name, m_windowViSA_width,10);

}


bool myApp::init_faceDetection(std::string xml_file){

    //face_cascade.load("/Users/greg/anaconda/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml");
    if(!face_cascade.load(xml_file)){
        std::cout << " Impossible de charger le CNN : " << xml_file << std::endl;
        return false;

    }else{
        return true;
    }
 
}

int myApp::findFaces(cv::Mat  frame_rgb){
    if(!is_FaceDetection_initialized)
      return 0;
    cvtColor( frame_rgb, m_visage_gray, COLOR_BGR2GRAY );
    equalizeHist( m_visage_gray, m_visage_gray );
    face_cascade.detectMultiScale( m_visage_gray, faces, 1.1, 3, 0|CASCADE_SCALE_IMAGE );

    if(faces.size() > 1)
            sort( faces.begin(), faces.end(), compareRectangleBySize);

    return faces.size();
}


void myApp::drawFaces(cv::Mat &frame_bgr){
    if(faces.size() ==0 )
        return;

    //Face DETECTION
    // FIrst Face in Green , other in black
    Point center( faces[0].x + faces[0].width/2, faces[0].y + faces[0].height/2 );
    ellipse( frame_bgr, center, Size( faces[0].width/2, faces[0].height/2), 0, 0, 360, Scalar( 255, 0, 255 ), 10, 8, 0 );
    
      
    for( auto i = 1; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        ellipse( frame_bgr, center, Size( faces[i].width/2, faces[i].height/2), 0, 0, 360, Scalar( 10,10,10), 10, 8, 0 );
    }

    // FACE Folowed
    
    rectangle(frame_bgr, camshift_startingWindows,Scalar(0,0,255), 3, CV_AA);
    //ellipse( frame_bgr, camshift_trackBox, Scalar(0,0,255), 3, CV_AA );

}

void myApp::followVisage_Camshift_init(cv::Mat frame_bgr){
    if(faces.size() ==0 )
        return;


    camshift_startingWindows = faces[0];
    
    //Passage HSV
    cvtColor(frame_bgr, camshitf_hsv, COLOR_BGR2HSV);
    //saturation  S & V 
    inRange(camshitf_hsv, Scalar(0  , camshitf_smin, camshitf_vmin),
                         Scalar(180, 256          , camshitf_vmax),
            camshift_mask            );
    //Extract Hue
    int ch[] = {0, 0};
    if(camshift_hue.cols *camshift_hue.rows  !=  frame_bgr.cols *frame_bgr.rows)
        camshift_hue.create(camshitf_hsv.size(), camshitf_hsv.depth());
    mixChannels(&camshitf_hsv, 1, &camshift_hue, 1, ch, 1);
    
    //  Histo 2D
    Mat roi(camshift_hue, camshift_startingWindows), maskroi(camshift_mask, camshift_startingWindows);
    calcHist(&roi, 1, 0, maskroi, camshift_hist, 1, &camshift_hsize, &camshift_phranges);
    normalize(camshift_hist, camshift_hist, 0, 255, NORM_MINMAX);


    
    // Permet le byPass de recherche  quand on vient d'initialisé
    camshift_just_init = true;

}

void myApp::followVisage_camShift(cv::Mat frame_bgr){
if(faces.size() ==0 )
        return;

    if(camshift_just_init){
        camshift_just_init = false;
        return;
    }


//Passage HSV
cvtColor(frame_bgr, camshitf_hsv, COLOR_BGR2HSV);
//saturation  S & V 
inRange(camshitf_hsv, Scalar(0  , camshitf_smin, camshitf_vmin),
                    Scalar(180, 256          , camshitf_vmax),
        camshift_mask            );


// Perform CAMShift
std::cout << " cmashift camshift_startingWindows IN  :( "<< camshift_startingWindows.x << " , "<< camshift_startingWindows.y << ")"<< std::endl;
calcBackProject(&camshift_hue, 1, 0, camshift_hist, camshift_backproj, &camshift_phranges);
camshift_backproj &= camshift_mask;
int  nb_steps = meanShift(camshift_backproj, camshift_startingWindows, TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));
std::cout << " cmashift camshift_startingWindows OUT :( "<< camshift_startingWindows.x << " , "<< camshift_startingWindows.y << ")"<< std::endl;
std::cout << " cmashift took : "<< nb_steps << "to find face"<< std::endl;
//camshift_trackBox = CamShift(camshift_backproj, camshift_startingWindows, TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));
//camshift_startingWindows = camshift_trackBox.boundingRect();

}

