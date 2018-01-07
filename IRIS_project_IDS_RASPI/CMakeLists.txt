cmake_minimum_required (VERSION 3.1)
project (iris_project)
set (CMAKE_CXX_STANDARD 14)



FIND_PACKAGE( Boost 1.50 COMPONENTS system filesystem program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )


find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(raspicam_DIR "/usr/local/lib/cmake/")
find_package(raspicam REQUIRED)

set (MES_SOURCES
   TimeStamp.cc
   Video.Camera.cc
   Video.IDSCamera.cc
   Video.Image.cc)

set (MES_LIBS -lueye_api -lboost_thread -lboost_system -lboost_filesystem -lboost_program_options -lwiringPiPca9685 -lwiringPi)


add_executable (iris_project ${MES_SOURCES} main.cpp)
target_link_libraries (iris_project ${MES_LIBS} ${OpenCV_LIBS} ${raspicam_CV_LIBS})

#target_link_libraries(iris_project ${Boost_LIBRARIES} -lboost_thread -lboost_system -lboost_filesystem -lboost_program_options)
