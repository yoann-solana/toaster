/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright © 2017 CNRS-LAAS
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Yoann Solana */

// YOYO:TODO Wiki

/***** Description
 * This locate_mocap package is used to compute the transform
  between the origin of the MOCAP system and the map frame of the adream apartment.
 */

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
//#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <tinyxml.h>


const std::string NODE_NAME = "locate_mocap";

const std::vector<std::string> components = {"x", "y", "z", "rx", "ry", "rz"};

typedef std::map<std::string, geometry_msgs::TransformStamped> map_tfs_t;
map_tfs_t map_tfs = {
    {"tf_map_2_aecm", geometry_msgs::TransformStamped()}
   ,{"tf_aecm_2_aecma", geometry_msgs::TransformStamped()}
   ,{"tf_aecma_2_aec", geometry_msgs::TransformStamped()}
   ,{"tf_aec_2_mocap", geometry_msgs::TransformStamped()}
};
const std::string tf_to_adapt_name = "tf_aecm_2_aecma";

const std::string tf_pref("tf_");
const std::string tf_sep = "_2_";
const std::string cmd_sep = "=";



bool splitStringInTwo(const std::string str, std::string& str1, std::string& str2, const std::string sep, const std::string pref);

template <class T> bool getParamOnROSServer(ros::NodeHandle& nh, const std::string param_name, T& param_val);

bool loadTfFromROSServer(ros::NodeHandle& nh, map_tfs_t::iterator& it);

bool loadTfFs(ros::NodeHandle& nh);

void pubTfs(tf2_ros::TransformBroadcaster& br);

void updateTfFromKeyInput(const std::string s);

bool openXmlFile(std::string fileName, TiXmlDocument& doc);

bool createEnvMarker(visualization_msgs::Marker& mk, const geometry_msgs::PoseStamped p, const int id,
                     std_msgs::ColorRGBA c, const std::string obj_name, TiXmlDocument& listObj, const double duration);

void setPoseStamped(geometry_msgs::PoseStamped& p, const std::string frame, const double x = 0., const double y = 0., const double z = 0.,
                    const double roll = 0., const double pitch = 0., const double yaw = 0.);

void setColor(std_msgs::ColorRGBA& c, const double r, const double g, const double b, const double a = 1.);

double getBoundedTo01(const double v);

void updatePoseMk(visualization_msgs::Marker& mk, const geometry_msgs::PoseStamped p);

void displayTf(const geometry_msgs::TransformStamped tf);

int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster br;
    ros::Publisher pub_mk = nh.advertise<visualization_msgs::Marker>("/locate_mocap/marker", 100);
    TiXmlDocument listObj;
    if( !openXmlFile("/src/list_obj.xml", listObj))
    {
        ROS_ERROR_STREAM("Cannot open xml file 'list_obj");
        return -1;
    }


    // Load the tfs
    loadTfFs(nh);

    // Print description for the user
    std::cout<<"This program helps to measure the tranform between the origin frame of the MOCAP system and the origin frame "
               "of the environment 'map'. The alignement is done by updating the value of the transform "<<tf_to_adapt_name<<"."
               "To update a component of the transform you must enter 'name=value', where compo_value must be a double"
               "compo_name is one of these pattern:";
    for(size_t i=0; i<components.size(); i++)
        std::cout<<" "<<components[i];
    std::cout<<std::endl;

    // Create markers representing the environment
    // - mk_env1: holds the model of the apartment in the map frame (corner of the ardream room),
    //   this one will move when the user will update the tf_to_adapt_name transform
    // - mk_env2: holds the model of the apartment which is defined in the mocap frame
    visualization_msgs::Marker mk_env1, mk_env2;
    geometry_msgs::PoseStamped p1;
    setPoseStamped(p1, "map");
    std_msgs::ColorRGBA c1;
    setColor(c1, 0.2, 0.2, 1., 0.9);
    createEnvMarker(mk_env1, p1, 1, c1, "env_adream_empty_updated", listObj, 100.);
    geometry_msgs::PoseStamped p2;
    setPoseStamped(p2, "aec");
    std_msgs::ColorRGBA c2;
    setColor(c2, 0.2, 1., 0.2, 0.9);
    createEnvMarker(mk_env2, p2, 2, c2, "env_apartment_empty_updated", listObj, 10000.);

    pub_mk.publish(mk_env1);
    pub_mk.publish(mk_env2);

    // Start the loop
    std::string msg;
    ros::Rate r(1); // In Hz
    while (ros::ok())
    {
        pubTfs(br);
        pub_mk.publish(mk_env1);
        pub_mk.publish(mk_env2);

        std::cout<<"To update "<<tf_to_adapt_name<<" frame, enter: 'name=value' <=> ";
        std::cin >> msg ;
        updateTfFromKeyInput(msg);
        msg.clear();



        r.sleep();
    }

    return 0;
}

/**********************************************************************/

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

bool loadTfFromROSServer(ros::NodeHandle& nh, map_tfs_t::iterator& it)
{
    std::string param_name;
    std::vector<double> params_val(components.size());
    double val;
    for(size_t i=0; i<components.size(); i++)
    {
        param_name = NODE_NAME + "/" + it->first + "/" + components[i];

        if( !getParamOnROSServer(nh, param_name, val))
        {
            ROS_ERROR_STREAM("Tf "<<it->first<<" cannot be set");
            return false;
        }

        params_val[i] = val;
    }

    it->second.transform.translation.x = params_val[0];
    it->second.transform.translation.y = params_val[1];
    it->second.transform.translation.z = params_val[2];

    tf2::Quaternion q;
    q.setRPY(params_val[3], params_val[4], params_val[5]);
    it->second.transform.rotation = tf2::toMsg(q);

    // Set the reference and child frames
    std::string ref_frame, child_frame;

    splitStringInTwo(it->first, ref_frame, child_frame, tf_sep, tf_pref);

    it->second.header.frame_id = ref_frame;
    it->second.child_frame_id = child_frame;
    it->second.header.seq = 0;

    return true;
}

bool loadTfFs(ros::NodeHandle& nh)
{
    map_tfs_t::iterator it = map_tfs.begin();
    for(; it!=map_tfs.end(); ++it)
    {
        if( !loadTfFromROSServer(nh, it) )
        {
            return false;
        }

//        std::stringstream ss; ss<<"\n-----\n"<<it->second;
//        std::cout<<ss.str()<<std::endl;
    }

    return true;
}

void pubTfs(tf2_ros::TransformBroadcaster& br)
{
    map_tfs_t::iterator it = map_tfs.begin();
    std::vector<geometry_msgs::TransformStamped> tfs(map_tfs.size());
    ros::Time t = ros::Time::now();
    int i = 0;
    std::cout<<"-----------------------\n";
    for(; it!=map_tfs.end(); ++it, i++)
    {
        geometry_msgs::TransformStamped& tf = it->second;
        tf.header.stamp = t;
        tf.header.seq ++;

        tfs[i] = it->second;

//        displayTf(tfs[i]);
    }

    br.sendTransform(tfs);
}

void updateTfFromKeyInput(const std::string s)
{
    std::string comp, val_str;
    double val;

    if( !splitStringInTwo(s, comp, val_str, cmd_sep, "") )
        return;

    try {
        val = std::stod(val_str);
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM(e.what());
        return;
    }

    map_tfs_t::iterator it = map_tfs.find(tf_to_adapt_name);
    if(it == map_tfs.end())
    {
        ROS_ERROR_STREAM("Frame "<<tf_to_adapt_name<<" unknown");
        return;
    }

    geometry_msgs::TransformStamped& tf = it->second;
    double roll, pitch, yaw;
    if(comp == components[0]) // x
    {
        tf.transform.translation.x = val;
    }
    else if(comp == components[1]) // y
    {
        tf.transform.translation.y = val;
    }
    else if(comp == components[2]) // z
    {
        tf.transform.translation.z = val;
    }
    else if(comp == components[3]  ||  comp == components[4]  ||  comp == components[5]) // rx || ry || rz
    {
        tf2::Quaternion q;
        tf2::fromMsg(tf.transform.rotation, q);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        if(comp == components[3]) // roll
        {
            roll = val;
        }
        else if(comp == components[4]) // pitch
        {
            pitch = val;
        }
        else if(comp == components[5]) // yaw
        {
            yaw = val;
        }

        q.setRPY(roll, pitch, yaw);
        tf.transform.rotation = tf2::toMsg(q);
    }

    std::cout<<"Frame "<<tf_to_adapt_name<<": x y z; roll pitch yaw = "<<tf.transform.translation.x<<" "<<
               tf.transform.translation.y<<" "<<tf.transform.translation.z<<"; "<<roll<<" "<<pitch<<" "<<yaw<<"\n";
}

void displayTf(const geometry_msgs::TransformStamped tf)
{
    geometry_msgs::Transform t = tf.transform;
    std::cout<<"tf "<<tf.header.frame_id<<"->"<<tf.child_frame_id<<": "<<t.translation.x<<" "<<t.translation.y<<" "<<
               t.translation.z<<" "<<"; "<<t.rotation.x<<" "<<t.rotation.y<<" "<<t.rotation.z<<" "<<t.rotation.x<<"\n";
}

bool splitStringInTwo(const std::string str, std::string& str1, std::string& str2, const std::string sep, const std::string pref)
{
    std::string s = str;
    // Check and remove tf prefix if valid
    if ( s != pref &&
         s.size() > pref.size() &&
         s.substr(0, pref.size()) == pref )
    {
        // s start with tf_prefix
        s = s.substr(pref.size(), s.size());
    }
    else
    {
        return false;
    }

    // Split the remaining string
    str1 = s.substr(0, s.find(sep));
    str2 = s.substr(s.find(sep) + sep.size(), s.size());

    // Check the validity of the strings
    if(str1.empty()  ||  str2.empty())
    {
        ROS_ERROR_STREAM("str1 |& str2 frames cannot be set from "<<s<<" string");
        return false;
    }

    return true;
}

bool createEnvMarker(visualization_msgs::Marker& mk, const geometry_msgs::PoseStamped p, const int id,
                     std_msgs::ColorRGBA c, const std::string obj_name, TiXmlDocument& listObj, const double duration)
{
    //frame id
    mk.header.frame_id = p.header.frame_id; //"map";

    //namespace
    mk.ns = obj_name;
    mk.id = id;

    //action
    mk.action = visualization_msgs::Marker::ADD;

    //pose
    mk.pose = p.pose;

    //color
    mk.color = c;

    //scale
    double scale = 1.;
    mk.scale.x = scale;
    mk.scale.y = scale;
    mk.scale.z = scale;

    mk.type = visualization_msgs::Marker::MESH_RESOURCE;

    mk.lifetime = ros::Duration(duration);

    TiXmlHandle hdl(&listObj);
    TiXmlElement *elem = hdl.FirstChildElement().FirstChildElement().Element();

    std::string obj_name_xml;
    std::string mesh_r;

    bool found = false;
    while (elem) //for each element of the xml file
    {
        obj_name_xml = elem->Attribute("name");
        mesh_r = elem->Attribute("mesh_resource");
        elem = elem->NextSiblingElement();

        if (obj_name_xml.compare(obj_name) == 0) //if there is a 3d model relativ to this object
        {
            mk.mesh_resource = mesh_r;
            mk.mesh_use_embedded_materials = true;

            elem = NULL;
            found = true;
        }
    }

    if( !found)
    {
        ROS_ERROR_STREAM("Object "<<obj_name<<" not found in list_obj.xml");
    }

    if( mesh_r.empty() )
    {
        ROS_ERROR_STREAM("Mesh ressource not found for "<<obj_name);
        return false;
    }

    return true;
}

bool openXmlFile(std::string fileName, TiXmlDocument& doc)
{
    std::stringstream path;
    path << ros::package::getPath("toaster_visualizer") << fileName;
    doc = TiXmlDocument(path.str());

    if (!doc.LoadFile()) {
        ROS_ERROR_ONCE("Error while loading xml file");
        ROS_ERROR_ONCE("error # %d", doc.ErrorId());
        ROS_ERROR_ONCE("%s", doc.ErrorDesc());

        return false;
    }

    return true;
}

void setPoseStamped(geometry_msgs::PoseStamped& p, const std::string frame, const double x, const double y, const double z,
                    const double roll, const double pitch, const double yaw)
{
    p.header.frame_id = frame;

    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    p.pose.orientation = tf2::toMsg(q);
}

void setColor(std_msgs::ColorRGBA& c, const double r, const double g, const double b, const double a)
{
    c.r = getBoundedTo01(r);
    c.g = getBoundedTo01(g);
    c.b = getBoundedTo01(b);
    c.a = getBoundedTo01(a);
}

double getBoundedTo01(const double v)
{
    double val;
    val = (v < 0.)? (0.): (v);
    val = (v > 1.)? (1.): (v);

    return val;
}

void updatePoseMk(visualization_msgs::Marker& mk, const geometry_msgs::PoseStamped p)
{
    mk.header.frame_id = p.header.frame_id;
    mk.pose = p.pose;
}
