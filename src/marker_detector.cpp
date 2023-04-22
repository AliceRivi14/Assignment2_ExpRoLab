/**
* \file marker_detector.cpp
* \brief Nodo per l'ndividuazione dei marker ArUco attraverso il movimento della camera del robot
* \author Alice Rivi
* \version 1.0
* \date 05/04/2023

* \details
*
* Subscribes to: 
*    RGB/RGB/image_raw Topic to retrieve the robot vision through the camera
*
* Publishes to: 
*   /A/jointC_position_controller/command Topic to pub the RGB camera joint angle for the robot arm
*
*   /MarkerList Topic in which it is published the markers detected by the robot
*
* Clients:
*   /room_info 
*
* Description :
*
* Through this node, the initial room in which the robot is located is inspected in order 
* to identify all markers and to be able to derive information about the locations in which 
* the robot will later move.
*
**/


#include <ros/ros.h>
#include <iostream>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <opencv2/highgui/highgui.hpp>

#include <assignment2/RoomInformation.h>
#include <assignment2/RoomConnection.h>
#include "marker_detector.h"
#include <cmath>
#include <list>
#include <unistd.h>



#include <typeinfo>

using namespace std;

#define _USE_MATH_DEFINES

list<int> List;

MarkerDetectorClass::MarkerDetectorClass(ros::NodeHandle* nodehandle):nh(*nodehandle){

/**
 * \brief Contructor of the MarkerDetectorClass
 * \param nodehandle
 * 
 * \return
 * 
 * This is the constructor of the MarkerDetectClass. It simply initializes publishers and subscribers that will be used in the code.
**/

  // Inizialization publisher and subscriber
  Pub_PoseCamera = nh.advertise<std_msgs::Float64>("/A/jointC_position_controller/command",1000);
  Pub_MList = nh.advertise<std_msgs::Int32>("/MarkerList",1000);
  Sub_Camera = nh.subscribe("/RGB/RGB/image_raw", 1000, &MarkerDetectorClass::CallbackCamera, this);
  // Inizialization client
  IDClient = nh.serviceClient<assignment2::RoomInformation>("/room_info");

  TotMarket = 0;
}

void MarkerDetectorClass::CallbackCamera(const sensor_msgs::Image::ConstPtr& ImFeed){

 /**
 * \brief Callback for the camera
 * \param
 * 
 * \return
 * 
 * This method is the callback of the camera, used by the respective subscriber.
 * It reads the information from the camera and thanks to the ArUco libraries processes it and detects the respective ArUco marker.
 * It prints on the terminal the detected marker ID.
 * 
  * Once all the markers are found their ID are published on the topic.
 * 
**/

 
  CvPtr = cv_bridge::toCvCopy(ImFeed,sensor_msgs::image_encodings::BGR8);

  cv::Mat inImage = CvPtr->image;
  // Clear out previous detection results
  Markers.clear();
  // Detect new marker
  MDetector.detect(inImage, Markers, CamParam, MSize, false);

  // Calibration
  CamParam = aruco::CameraParameters();

  if (!IDClient.waitForExistence(ros::Duration(5))) {
        	ROS_ERROR("Service not found");
        	return;
    	} 

  // Print the ID of the detected marker
  for (size_t i = 0; i<Markers.size(); i++){
    cout << "ID marker detected: ";
    int Id = Markers.at(i).id;
    cout << Id << " \n";
    if(find(List.begin(), List.end(), Id) == List.end()){
      assignment2::RoomInformation Room_srv;
      Room_srv.request.id = Id;

      if (IDClient.call(Room_srv) && Room_srv.response.room != "No room associated with this marker ID"){
        List.push_back(Id);
        TotMarket++;
      }
    }
    std_msgs::Int32 MId;
    MId.data = Id;
    Pub_MList.publish(MId);
  }

}

void MarkerDetectorClass::DetectMarker(){

  /**
 * \brief Method to find the marker
 * \param
 * 
 * \return
 * 
 * This method is used to move the robot joint and to detect the markers.
 * In particular the robot arm is set in the 'HomePose' configuration and the RGB joint is move looking at the ground from 0 to 3.14.
 * In this way it looks at all markers placed on the ground.
 * 
 * All the angles are set in radians.
**/

  // MoveIt
  robot_model_loader::RobotModelLoader RobotModel("robot_description");
  moveit::planning_interface::MoveGroupInterface Group("arm");

  std_msgs::Float64 Omega;
  Omega.data = 0.0;

  Group.setNamedTarget("HomePose");
  Group.move();
  sleep(1);
  cout << "HomePose" << endl;
  // RGB camera rotation around the z-axis
  while(Omega.data <= 2*M_PI){
    Omega.data += M_PI/6;
    cout << Omega.data << endl;
    Pub_PoseCamera.publish(Omega);
    sleep(2);
  }

  cout << "Marker found: " << TotMarket << endl;
}

int main(int argc, char **argv){

  /**
 * \brief Main
* \param
 * 	  argc The number of command line arguments
 * 	  argv The command line arguments
 *
 * \return 
 * 	  0 on success, non-zero on failure
 * 
 * This is the main method. It is responsible for making the routing work correctly.
 * The detection process continues until all 7 markers have been detected.
**/

  ros::init(argc, argv, "marker_detector");
  ros::NodeHandle nh_;

  MarkerDetectorClass MarkDetectClass(&nh_);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  cout << "MARKER DETECTOR" << endl;
  while(ros::ok()){

    while(MarkDetectClass.TotMarket != 7){
      MarkDetectClass.DetectMarker();
    }
    

    
    ros::waitForShutdown();
    
  }

  return 0;
}
