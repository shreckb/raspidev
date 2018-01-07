#include <iostream>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include "my_app.h"

// Global
myApp GLOBAL_DATA;


#include "main.visage.h"
#include "main.iris.h"

using namespace std;




int main()
{
    int k;
    cout << "Hello world!" << endl;
    GLOBAL_DATA.facedetection = true;

    cout << "Lancement des threads VISAGE et IRIS" << endl;
    boost::thread thread_face_detect{runFaceDetection};
    //boost::thread thread_iris_detect{runIRISDetection};

    cout << "entrez une commande : " << endl;
    cin >> k;
    cout << "touche frappee" << endl;

    GLOBAL_DATA.cameraVisage_running = false;
    GLOBAL_DATA.cameraIRIS_running   = false;
    //Attente de fermeture des threads
    //thread_face_detect.join();

    cout << "end of the app" << endl;
    return 0;
}
