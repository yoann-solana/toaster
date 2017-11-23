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

template <class T>
bool getParamOnROSServer(ros::NodeHandle& nh, const std::string param_name, T& param_val)
{
    if( !nh.hasParam(param_name) )
    {
        ROS_ERROR_STREAM("Param "<<param_name<<" unknown from ROS server");
        return false;
    }

    nh.getParam(param_name, param_val);

    return true;
}

bool loadTfFromROSServer(ros::NodeHandle& nh, const std::string prefix, std::string tf_name, geometry_msgs::TransformStamped& tf)
{
    std::string param_name;
    std::vector<double> params_val(components.size());
    double val;
    for(size_t i=0; i<components.size(); i++)
    {
        param_name = prefix + "/" + tf_name + "/" + components[i];

        if( !getParamOnROSServer(nh, param_name, val))
        {
            ROS_ERROR_STREAM("Tf "<<tf_name<<" cannot be set with param name = "<<param_name);
            return false;
        }

        params_val[i] = val;
    }

    tf.transform.translation.x = params_val[0];
    tf.transform.translation.y = params_val[1];
    tf.transform.translation.z = params_val[2];

    tf2::Quaternion q;
    q.setRPY(params_val[3], params_val[4], params_val[5]);
    tf.transform.rotation = tf2::toMsg(q);

    // Set the reference and child frames
    std::string ref_frame, child_frame;

    splitStringInTwo(tf_name, ref_frame, child_frame, tf_sep, tf_pref);

    tf.header.frame_id = ref_frame;
    tf.child_frame_id = child_frame;
    tf.header.seq = 0;

    return true;
}

#endif
