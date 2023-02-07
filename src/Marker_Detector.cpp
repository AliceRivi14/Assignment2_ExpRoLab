#include <ros/ros.h>
#include <iostream>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <opencv2/highgui/highgui.hpp>

#include "Marker_Detector.h"
#include <cmath>
#include <list>
#include <unistd.h>

using namespace std;

#define _USE_MATH_DEFINES

list<int> List;

MarkerDetectorClass::MarkerDetectorClass(ros::NodeHandle* nodehandle):nh(*nodehandle){

  Pub_PoseJoint0 = nh.advertise<std_msgs::Float64>("/A/joint0_position_controller/command",1000);
  Pub_PoseJoint1 = nh.advertise<std_msgs::Float64>("/A/joint1_position_controller/command",1000);
  Pub_PoseJoint2 = nh.advertise<std_msgs::Float64>("/A/joint2_position_controller/command",1000);
  Pub_PoseCamera = nh.advertise<std_msgs::Float64>("/A/jointC_position_controller/command",1000);

  Pub_MList = nh.advertise<std_msgs::Int32MultiArray>("/MarkerList",1000);

  Sub_Camera = nh.subscribe("RGB/RGB/image_raw", 1000, &MarkerDetectorClass::CallbackCamera, this);

  TotMarket = 0;
}

void MarkerDetectorClass::CallbackCamera(const sensor_msgs::Image::ConstPtr& msg){

  // It reads the information from the camera and thanks to the ArUco libraries processes it and detects the respective ArUco marker.
  // It prints on the terminal the detected marker ID.

  // ArUco stuff
  aruco::MarkerDetector MDetector;
  std::vector<aruco::Marker> Markers;
  aruco::CameraParameters CamParam;
  double MSize = 0.05;

  // CvImagePtr to convert the ROS image message in a CvImage suitable for working with OpenCV
  cv_bridge::CvImagePtr CvPtr;
  CvPtr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

  cv::Mat inImage = CvPtr->image;
  // Clear out previous detection results
  Markers.clear();
  // Detect new marker
  MDetector.detect(inImage, Markers, CamParam, MSize, false);

  cout << "ID marker detected: ";
  for (size_t i = 0; i<Markers.size(); i++){
    int Id = Markers.at(i).id;
    cout << Id << " ";
    if(!(std::find(List.begin(), List.end(), Id) != List.end()))
      List.push_back(Id);
      TotMarket++;
  }
  cout << endl;

}

void MarkerDetectorClass::DetectMarker(){

  // MoveIt
  moveit::planning_interface::MoveGroupInterface Group("arm");
  Group.setEndEffectorLink("RGB");
  Group.setPoseReferenceFrame("base_link");
  Group.setPlannerId("RRTstar");
  Group.setNumPlanningAttempts(10);
  Group.setPlanningTime(10.0);
  Group.allowReplanning(true);
  Group.setGoalJointTolerance(0.0001);
  Group.setGoalPositionTolerance(0.0001);
  Group.setGoalOrientationTolerance(0.001);

  std_msgs::Float64 Omega;

  cout << "SONO QUA" << endl;

  Group.setNamedTarget("HomePose");
  Group.move();
  cout << "HomePose" << endl;
  // RGB camera rotation around the z-axis
  for(Omega.data = 0.0; Omega.data <= 2*M_PI; Omega.data += M_PI/12){
    cout << "OMEGA: " << Omega.data << endl;
    Pub_PoseCamera.publish(Omega);
  }
  cout << "Marker found: " << TotMarket << endl;

  Group.setNamedTarget("LowPose");  // PROBLEMA COLLISIONE??
  Group.move();
  sleep(0.5);
  cout << "LowPose" << endl;
  // RGB camera rotation around the z-axis
  for(Omega.data = 0.0; Omega.data <= 2*M_PI; Omega.data += M_PI/12){
    Pub_PoseCamera.publish(Omega);
  }
  cout << "Marker found: " << TotMarket << endl;

  Group.setNamedTarget("GroundPose");
  Group.move();
  sleep(0.5);
  cout << "GroundPose" << endl;
  // Arm Chassis rotation around the z-axis
  for(Omega.data = 0.0; Omega.data <= 2*M_PI; Omega.data += M_PI/12){
    Pub_PoseJoint0.publish(Omega);
  }
  cout << "Marker found: " << TotMarket << endl;

  Group.setNamedTarget("HomePose");
  Group.move();
  sleep(0.5);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "Marker_Detector");
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
