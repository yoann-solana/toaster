/*
 * File:   RobotReader.h
 * Author: Grégoire Milliez
 * mail: gregoire.milliez@laas.fr
 *  Copyright 2014 LAAS/CNRS. All rights reserved.
 *
 * Created on December 3, 2014, 6:19 PM
 */

// A robot reader is a class that will read data from a middleware message
// and fill a Robot class from toaster-lib accordingly to publish on a ros topic.

#ifndef ROBOTREADER_H
#define	ROBOTREADER_H

#include <ros/ros.h>
#include "toaster-lib/Robot.h"
#include "pdg/readers/Reader.h"
#include "pdg/types.h"
#include <map>
#include <string>
#include "optitrack/or_pose_estimator_state.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

const std::string mocap_robot_topic_prefix = "mocapRobotTopicPrefix";

class RobotReader : public Reader<Robot>{

    public:
        RobotReader();
        virtual ~RobotReader();

        void init();

        virtual void Publish(struct toasterList_t& list_msg);

        void publishTfBase(tf::TransformBroadcaster &tf_br);

    protected:
        bool fullRobot_;
        std::string robot_pseudo_;
        std::string robot_name_;
        std::string robot_footprint_;

        bool use_mocap_loc_;
        std::string mocap_robot_topic_;
        ros::Subscriber sub_mocap_rob_pose_;
        geometry_msgs::TransformStamped tf_map_2_mocap_;

        bool isPresent(std::string id);

        toaster_msgs::Fact DefaultFactMsg(std::string subjectId, uint64_t factTime);

        virtual void mocapRobCb(const optitrack::or_pose_estimator_state::ConstPtr & msg);
};

#endif /* ROBOTREADER_H */
