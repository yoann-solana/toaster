#include <utility/tf_utility.h>

bool loadTfFromROSServer(ros::NodeHandle& nh, const std::string prefix, std::string tf_name, geometry_msgs::TransformStamped& tf)
{
    std::string param_name;
    std::vector<double> params_val(components.size());
    double val;
    for(size_t i=0; i<components.size(); i++)
    {
        param_name = prefix + "/" + tf_name + "/" + components[i];

        if( !nh.getParam(param_name, val))
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
