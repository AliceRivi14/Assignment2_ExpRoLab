#ifndef MARKER_DETECTCTOR_CLASS_H_
#define MARKER_DETECTCTOR_CLASS_H_

#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <algorithm>
#include "sensor_msgs/Image.h"
#include <std_msgs/Float64.h>
#include "std_msgs/Int32.h"

class MarkerDetectorClass
{
public:
  MarkerDetectorClass(ros::NodeHandle* nodehandle);

  ros::NodeHandle nh;

  ros::Publisher Pub_Joint0;
  ros::Publisher Pub_PoseCamera;
  ros::Publisher Pub_MList;
  ros::Subscriber Sub_Camera;

  ros::ServiceClient IDClient;

  void DetectMarker();
  void CallbackCamera(const sensor_msgs::Image::ConstPtr& ImFeed);

  int TotMarket;

   // ArUco stuff
  aruco::MarkerDetector MDetector;
  std::vector<aruco::Marker> Markers;
  aruco::CameraParameters CamParam;
  double MSize = 0.05;

  // CvImagePtr to convert the ROS image message in a CvImage suitable for working with OpenCV
  cv_bridge::CvImagePtr CvPtr;

};
#endif
