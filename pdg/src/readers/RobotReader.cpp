#include "pdg/readers/RobotReader.h"
#include <utility/tf_utility.h>
#include <utility/params.h>


RobotReader::RobotReader()
    : Reader<Robot>()
    , fullRobot_(false)
{}

RobotReader::~RobotReader()
{
  for(std::map<std::string, Robot*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it)
    delete it->second;
}

void RobotReader::init()
{
    // Check and get param form ROS server about mocap location if they exist
    std::string key_pref = "/pdg/";
    node_->getParam(key_pref+ "use_mocap_loc", use_mocap_loc_);
    std::cout<<"use_mocap_loc_ = "<<use_mocap_loc_<<std::endl;

    if(use_mocap_loc_)
    {
        // Get the name and pseudo of the robot
        node_->getParam(key_pref + "robot_name", robot_name_);
        node_->getParam(key_pref + "robot_pseudo", robot_pseudo_);
        node_->getParam(key_pref + "robot_footprint", robot_footprint_);

        if(robot_name_.empty()  ||  robot_pseudo_.empty()  ||  robot_footprint_.empty())
        {
            ROS_ERROR_STREAM("robot_name or robot pseudo or robot_footprint must be defined in robots.yaml file. Location with MOCAP will be not used");
            use_mocap_loc_ = false;
            return;
        }
        std::cout<<"robot_name_ = "<<robot_name_<<", robot_pseudo_ = "<<robot_pseudo_<<", robot_footprint_ = "<<robot_footprint_<<std::endl;


        // Get the topic where the pose of pr2 will be provided in the MOCAP frame
        if(!node_->getParam(key_pref + mocap_robot_topic_prefix, mocap_robot_topic_))
        {
            ROS_ERROR_STREAM("Cannot get value of '"<<mocap_robot_topic_prefix<<"' from ROS server");
            use_mocap_loc_ = false;
            return;
        }

        mocap_robot_topic_ += robot_name_ + "_" + robot_pseudo_ + "_" + robot_footprint_;
        std::cout<<"mocap_robot_topic_ = "<<mocap_robot_topic_<<std::endl;

        // Init the subscriber
        sub_mocap_rob_pose_ = node_->subscribe(mocap_robot_topic_, 10, &RobotReader::mocapRobCb, this);
        while(sub_mocap_rob_pose_.getNumPublishers() < 1)
        {
            ROS_WARN_STREAM("Waiting for publisher on topic "<<mocap_robot_topic_);
            sleep(1);
        }

        // Load the transform from map to mocap
        if (node_->hasParam(locate_mocap_prefix + "/" + tf_map_2_mocap_name))
        {
            if(!loadTfFromROSServer(*node_, locate_mocap_prefix, tf_map_2_mocap_name, tf_map_2_mocap_))
            {
                ROS_WARN_STREAM(tf_map_2_mocap_name<<" frame not defined on the the ROS server");
                use_mocap_loc_ = false;
                return;
            }
        }
    }
}

void RobotReader::mocapRobCb(const optitrack::or_pose_estimator_state::ConstPtr & msg)
{
    ROS_WARN_STREAM_ONCE("Your RobotReadder class must define the 'mocapRobCb()' Callback function");
}

void RobotReader::publishTfBase(tf::TransformBroadcaster& tf_br)
{
    if(!use_mocap_loc_)
        return;

    Robot* curRobot = lastConfig_["pr2"];

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(curRobot->getPosition().get<0>(), curRobot->getPosition().get<1>(), curRobot->getPosition().get<2>()));
    double r = curRobot->getOrientation()[0];
    double p = curRobot->getOrientation()[1];
    double y = curRobot->getOrientation()[2];
    tf::Quaternion q = tf::createQuaternionFromRPY(r, p, y);
    transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

    tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", robot_footprint_));

    std::cout<<"Published tf_map_2_"<<robot_footprint_<<": "<<curRobot->getPosition().get<0>()<<" "<<curRobot->getPosition().get<1>()<<" "<<curRobot->getPosition().get<2>()<<
               "; "<<r<<" "<<p<<" "<<y<<std::endl;
}

void RobotReader::Publish(struct toasterList_t& list_msg)
{
  if(activated_)
  {
    for (std::map<std::string, Robot *>::iterator it = lastConfig_.begin();
         it != lastConfig_.end(); ++it) {

        toaster_msgs::Fact fact_msg = DefaultFactMsg(it->first, it->second->getTime());
        list_msg.fact_msg.factList.push_back(fact_msg);

        //Robot
        toaster_msgs::Robot robot_msg;
        robot_msg.meAgent.mobility = 0;

        fillEntity(lastConfig_[it->first], robot_msg.meAgent.meEntity);

        if (fullRobot_)
        {
            for (std::map<std::string, Joint *>::iterator itJoint = lastConfig_[it->first]->skeleton_.begin();
                 itJoint != lastConfig_[it->first]->skeleton_.end(); ++itJoint)
            {
                toaster_msgs::Joint joint_msg;
                robot_msg.meAgent.skeletonNames.push_back(itJoint->first);
                fillEntity((itJoint->second), joint_msg.meEntity);

                joint_msg.jointOwner = it->first;
                joint_msg.position = itJoint->second->position;

                robot_msg.meAgent.skeletonJoint.push_back(joint_msg);
            }
        }
        list_msg.robot_msg.robotList.push_back(robot_msg);
    }
  }
}

bool RobotReader::isPresent(std::string id){
  timeval curTime;
  gettimeofday(&curTime, NULL);
  unsigned long now = curTime.tv_sec * pow(10,9) + curTime.tv_usec;
  unsigned long timeThreshold = pow(10,9);

  long timeDif = lastConfig_[id]->getTime() - now;

  if ( fabs(timeDif) < timeThreshold)
      return true;
  else
      return false;
}

toaster_msgs::Fact RobotReader::DefaultFactMsg(std::string subjectId, uint64_t factTime)
{
  toaster_msgs::Fact fact_msg;

  //Fact
  fact_msg.property = "isPresent";
  fact_msg.subjectId = subjectId;
  fact_msg.stringValue = "true";
  fact_msg.confidence = 0.90;
  fact_msg.factObservability = 1.0;
  fact_msg.time = factTime;
  fact_msg.valueType = 0;

  return fact_msg;
}
