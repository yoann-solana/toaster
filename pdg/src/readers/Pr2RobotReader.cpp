#include "pdg/readers/Pr2RobotReader.h"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

Pr2RobotReader::Pr2RobotReader(bool fullRobot)
    : RobotReader()
{
    fullRobot_ = fullRobot;
}

void Pr2RobotReader::init(ros::NodeHandle* node, std::string param) {
  std::cout << "[PDG] Initializing Pr2RobotReader" << std::endl;
  Reader<Robot>::init(node, param);

  RobotReader::init();

  if (fullRobot_)
      sub_ = node_->subscribe("joint_states", 1, &Pr2RobotReader::pr2JointStateCallBack, this);

  initJointsName_ = false;
  Robot* curRobot = new Robot("pr2");
  //TODO: setname with id
  curRobot->setName("PR2_ROBOT");

  lastConfig_["pr2"] = curRobot;
}

void Pr2RobotReader::mocapRobCb(const optitrack::or_pose_estimator_state::ConstPtr & msg)
{
    if(activated_)
    {
      Robot* curRobot = lastConfig_["pr2"];

      try {
          if (curRobot  &&  msg->pos.size() != 0) {
              //set position
              bg::model::point<double, 3, bg::cs::cartesian> rob_position;
              //set the orientation
              std::vector<double> rob_orientation;

              // Convert the pose in the map frame if use_tf_map_mocap_ is true
              Eigen::Affine3d e_map_2_mocap = tf2::transformToEigen(tf_map_2_mocap_);
              Eigen::Affine3d e_mocap_2_base = Eigen::Affine3d(Eigen::Translation3d(msg->pos[0].x, msg->pos[0].y, msg->pos[0].z)
                                                          * Eigen::Quaterniond(msg->pos[0].qw, msg->pos[0].qx, msg->pos[0].qy, msg->pos[0].qz));
              Eigen::Affine3d e_map_2_base = e_map_2_mocap*e_mocap_2_base;

              //// Set the value
              geometry_msgs::TransformStamped tf_st_map_2_base = tf2::eigenToTransform(e_map_2_base);
              // Position
              rob_position.set<0>(tf_st_map_2_base.transform.translation.x);
              rob_position.set<1>(tf_st_map_2_base.transform.translation.y);
              rob_position.set<2>(tf_st_map_2_base.transform.translation.z);

              // Orientation
              tf2::Quaternion q_map_2_base;
              tf2::fromMsg(tf_st_map_2_base.transform.rotation, q_map_2_base);
              tf2::Matrix3x3 m_map_2_base(q_map_2_base);
              double r, p, y;
              m_map_2_base.getRPY(r, p, y);
              rob_orientation.push_back(r);
              rob_orientation.push_back(p);
              rob_orientation.push_back(y);

              // Set the position and orientation
              curRobot->setPosition(rob_position);
              curRobot->setOrientation(rob_orientation);
              curRobot->setTime(ros::Time::now().toNSec()/1000.);
          }

      } catch (tf::TransformException ex) {
        ROS_ERROR_STREAM("Cannot update pose of " + robot_name_ + " " + robot_pseudo_ + " from mocap\n" + std::string(ex.what()));
      }
    }
}


// Maybe get this from a config file?

/*void Pr2RobotReader::initJointsName() {
    pr2JointsName_.push_back("base_link");
    pr2JointsName_.push_back("torso_lift_joint");
    pr2JointsName_.push_back("head_pan_joint");
    pr2JointsName_.push_back("head_tilt_joint");
    pr2JointsName_.push_back("laser_tilt_mount_joint");
    pr2JointsName_.push_back("r_shoulder_pan_joint");
    pr2JointsName_.push_back("r_shoulder_lift_joint");
    pr2JointsName_.push_back("r_upper_arm_roll_joint");
    pr2JointsName_.push_back("r_elbow_flex_joint");
    pr2JointsName_.push_back("r_forearm_roll_joint");
    pr2JointsName_.push_back("r_wrist_flex_joint");
    pr2JointsName_.push_back("r_wrist_roll_joint");
    pr2JointsName_.push_back("r_gripper_joint");
    pr2JointsName_.push_back("l_shoulder_pan_joint");
    pr2JointsName_.push_back("l_shoulder_lift_joint");
    pr2JointsName_.push_back("l_upper_arm_roll_joint");
    pr2JointsName_.push_back("l_elbow_flex_joint");
    pr2JointsName_.push_back("l_forearm_roll_joint");
    pr2JointsName_.push_back("l_wrist_flex_joint");
    pr2JointsName_.push_back("l_wrist_roll_joint");
    pr2JointsName_.push_back("l_gripper_joint");
}*/

void Pr2RobotReader::updateRobot(tf::TransformListener &listener)
{
  if(fullRobot_)
  {
    Robot* curRobot = lastConfig_["pr2"];
    Joint* curJoint = new Joint("pr2_base_link", "pr2");

    //// We start with base:
    if(use_mocap_loc_) // When the base is localed with the MOCAP sytem
    {
        //// Convert the pose of the robot from map->base_footprint to map->odom_combined
        // Get the pose map->base_footprint
        double r = curRobot->getOrientation()[0];
        double p = curRobot->getOrientation()[1];
        double y = curRobot->getOrientation()[2];
        tf::Quaternion q = tf::createQuaternionFromRPY(r, p, y);
        Eigen::Affine3d e_map_2_base_footprint = Eigen::Affine3d(
                Eigen::Translation3d(curRobot->getPosition().get<0>(), curRobot->getPosition().get<1>(), curRobot->getPosition().get<2>())
                * Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));

        // Get the pose odom_combined->base_footprint
        bool tf_ok = false;
        tf::StampedTransform t;
        try {
            listener.waitForTransform("odom_combined", robot_footprint_, ros::Time(0), ros::Duration(1.0) ); // dest frame, origin frame
            listener.lookupTransform("odom_combined", robot_footprint_, ros::Time(0), t);
            tf_ok = true;
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }


        if(tf_ok)
        {
            Eigen::Affine3d e_odom_combined_2_base_footprint = Eigen::Affine3d(
                    Eigen::Translation3d(t.getOrigin()[0], t.getOrigin()[1], t.getOrigin()[2])
                    * Eigen::Quaterniond(t.getRotation().getW(), t.getRotation().getX(), t.getRotation().getY(), t.getRotation().getZ()));

            Eigen::Affine3d e_map_2_odom_combined;
            e_map_2_odom_combined = e_map_2_base_footprint * e_odom_combined_2_base_footprint.inverse();
            tf::transformEigenToMsg(e_map_2_odom_combined, tf_map_2_odom_combined_.transform);
        }
    }
    else
    {
        curJoint->setName("base_link");

        setRobotJointLocation(listener, curJoint);
        curRobot->setOrientation(curJoint->getOrientation());
        curRobot->setPosition(curJoint->getPosition());
        curRobot->setTime(curJoint->getTime());
        delete curJoint;
    }

    //Then other joints if needed
    if (fullRobot_ && initJointsName_) {
        for (unsigned int i = 0; i < pr2JointsName_.size(); i++) {
            curJoint = curRobot->skeleton_[pr2JointsName_[i]];
            curJoint->setName(pr2JointsName_[i]);
            setRobotJointLocation(listener, curJoint);
        }
    }
  }
}

void Pr2RobotReader::setRobotJointLocation(tf::TransformListener &listener, Joint* joint) {
    tf::StampedTransform transform;
    std::string jointId = "/";
    std::vector<double> jointOrientation;
    bg::model::point<double, 3, bg::cs::cartesian> jointPosition;
    jointId.append(joint->getName());

    ROS_DEBUG("current joint %s \n", jointId.c_str());

    try {
        ros::Time now = ros::Time::now();
        ros::Time last = ros::Time(0);
        listener.waitForTransform("/map", jointId,
                last, ros::Duration(0.0));
        listener.lookupTransform("/map", jointId,
                last, transform);

        //Joint position
        jointPosition.set<0>(transform.getOrigin().x());
        jointPosition.set<1>(transform.getOrigin().y());
        jointPosition.set<2>(transform.getOrigin().z());

        //Joint orientation
        //curRobot->orientation.push_back(tf::getRoll(transform.getRotation()));
        //curRobot->orientation.push_back(tf::getPitch(transform.getRotation()));
        jointOrientation.push_back(0.0);
        jointOrientation.push_back(0.0);
        jointOrientation.push_back(tf::getYaw(transform.getRotation()));

        joint->setTime(now.toNSec());
        joint->setPosition(jointPosition);
        joint->setOrientation(jointOrientation);

    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
}

void Pr2RobotReader::pr2JointStateCallBack(const sensor_msgs::JointState::ConstPtr & msg) {
    if (!initJointsName_) {
        for (unsigned int i = 0; i < msg->name.size(); i++) {
            std::string jointName = msg->name[i];
            jointName = jointName.substr(0, msg->name[i].size() - 5);
            if (jointName.compare("r_gripper_") == 0 || jointName.compare("l_gripper_") == 0)
                jointName.append("palm_");
            jointName.append("link");
            pr2JointsName_.push_back(jointName);
            std::stringstream jointId;
            jointId << "pr2";
            jointId << msg->name[i];
            lastConfig_["pr2"]->skeleton_[pr2JointsName_[i]] = new Joint(jointId.str(), "pr2");

        }
        initJointsName_ = true;
    }

    if (pr2JointsName_.size() == msg->position.size()) {
        for (unsigned int i = 0; i < pr2JointsName_.size(); i++) {
            lastConfig_["pr2"]->skeleton_[pr2JointsName_[i]]->position = msg->position[i];
        }
    }
}


//Destructor

Pr2RobotReader::~Pr2RobotReader() {
    for (std::map<std::string, Robot*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it) {
        delete it->second;
    }
}
