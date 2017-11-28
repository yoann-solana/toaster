#ifndef PDG_TF_UTILITY_H
#define PDG_TF_UTILITY_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <utility/string_utility.h>

const std::vector<std::string> components = {"x", "y", "z", "rx", "ry", "rz"};

const std::string tf_pref("tf_");
const std::string tf_sep = "_2_";

//template <class T>
//bool getParamOnROSServer(ros::NodeHandle& nh, const std::string param_name, T& param_val)
//{
//    if( !nh.hasParam(param_name) )
//    {
//        ROS_ERROR_STREAM("Param "<<param_name<<" unknown from ROS server");
//        return false;
//    }

//    nh.getParam(param_name, param_val);

//    return true;
//}

bool loadTfFromROSServer(ros::NodeHandle& nh, const std::string prefix, std::string tf_name, geometry_msgs::TransformStamped& tf);

#endif
