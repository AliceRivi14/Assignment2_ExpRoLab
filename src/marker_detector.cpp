/**
* \file marker_detector.cpp
* \brief Nodo per l'ndividuazione dei marker ArUco attraverso il movimento della camera del robot
* \author Alice Rivi
* \version 0.1
* \date 05/04/2023

* \details
*
* Subscribes to: 
*    RGB/RGB/image_raw
*
* Publishes to: 
*   /A/joint0_position_controller/command
*
*   /MarkerList
*
* Clients:
*   /room_info 
*
* Description :
*
* Attraverso questo nodo viene ispezionata la stanza iniziale in cui si trova il robot 
* per individuare tutti i marker e poter ricavare le informazione riguardo alle location 
* in cui successoivamente il robot dovrà muoversi.

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

using namespace std;

#define _USE_MATH_DEFINES

list<int> List;

MarkerDetectorClass::MarkerDetectorClass(ros::NodeHandle* nodehandle):nh(*nodehandle){

  // Inizialization publisher and subscriber
  Pub_PoseCamera = nh.advertise<std_msgs::Float64>("/A/jointC_position_controller/command",1000);
  Pub_MList = nh.advertise<std_msgs::Int32MultiArray>("/MarkerList",1000);
  Sub_Camera = nh.subscribe("/RGB/RGB/image_raw", 1000, &MarkerDetectorClass::CallbackCamera, this);
  // Inizialization client
  IDClient = nh.serviceClient<assignment2::RoomInformation>("/room_info");

  TotMarket = 0;
}

void MarkerDetectorClass::CallbackCamera(const sensor_msgs::Image::ConstPtr& ImFeed){

  // It reads the information from the camera and thanks to the ArUco libraries processes it and detects the respective ArUco marker.
  // It prints on the terminal the detected marker ID.

 
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
    //BUG: request.id sbagliato
    if(find(List.begin(), List.end(), Id) == List.end()){
      assignment2::RoomInformation Room_srv;
      Room_srv.request.id = Id;

      if (IDClient.call(Room_srv) && Room_srv.response.room != "No room associated with this marker ID"){
        List.push_back(Id);
        TotMarket++;
      }
    }
  }

}

void MarkerDetectorClass::DetectMarker(){

  // MoveIt
  robot_model_loader::RobotModelLoader RobotModel("robot_description");
  moveit::planning_interface::MoveGroupInterface Group("arm");

  std_msgs::Float64 Omega;
  Omega.data = 0.0;

  Group.setNamedTarget("HomePose");
  Group.move();
  sleep(2);
  cout << "HomePose" << endl;
  // RGB camera rotation around the z-axis
  while(Omega.data <= 2*M_PI){
    Omega.data += M_PI/4;
    cout << Omega.data << endl;
    Pub_PoseCamera.publish(Omega);
    sleep(1.5);
  }

  cout << "Marker found: " << TotMarket << endl;

  Omega.data = 0.0;
  Pub_PoseCamera.publish(Omega);

  Group.setNamedTarget("LowPose");
  Group.move();
  sleep(2);
  cout << "LowPose" << endl;
  // RGB camera rotation around the z-axis
  while(Omega.data < 2*M_PI){
    Omega.data += M_PI/4;
    cout << Omega.data << endl;
    Pub_PoseCamera.publish(Omega);
    sleep(1.5);
  }
  
  cout << "Marker found: " << TotMarket << endl;

  Group.setNamedTarget("HomePose");
  Group.move();
  sleep(1);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "marker_detector");
  ros::NodeHandle nh_;

  MarkerDetectorClass MarkDetectClass(&nh_);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok()){

    while(MarkDetectClass.TotMarket != 7){
      MarkDetectClass.DetectMarker();
    }
    std_msgs::Int32MultiArray ArrList;
    for(int const &i: List){
      ArrList.data.push_back(i);
    }
    MarkDetectClass.Pub_MList.publish(ArrList);

    
    ros::waitForShutdown();
    
  }

  return 0;
}
