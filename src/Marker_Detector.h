#ifndef MARKER_DETECT_CLASS_H_
#define MARKER_DETECT_CLASS_H_

#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include "sensor_msgs/Image.h"
#include <std_msgs/Float64.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

class MarkerDetectorClass
{
public:
  MarkerDetectorClass(ros::NodeHandle* nodehandle);

  ros::NodeHandle nh;

  ros::Publisher Pub_PoseCamera;
  ros::Publisher Pub_MList;
  ros::Subscriber Sub_Camera;

  ros::ServiceClient IDClient;

  void DetectMarker();
  void CallbackCamera(const sensor_msgs::Image::ConstPtr& msg);

  int TotMarket;

};
#endif
