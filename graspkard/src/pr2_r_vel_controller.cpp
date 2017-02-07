/*
* Copyright (C) 2015, 2016 Jannik Buckelo <jannikbu@cs.uni-bremen.de>,
* Georg Bartels <georg.bartels@cs.uni-bremen.de>
*
*
* This file is part of giskard_examples.
*
* giskard_examples is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2 
* of the License, or (at your option) any later version.  
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <yaml-cpp/yaml.h>
#include <giskard/giskard.hpp>
//#include <kdl_conversions/kdl_msg.h>
#include <boost/lexical_cast.hpp>

#include <eigen3/Eigen/Eigen>
#include <r_libs/VisualizationManager.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf_conversions/tf_eigen.h>

int nWSR_;
giskard::QPController controller_;
std::vector<std::string> joint_names_;
std::vector<ros::Publisher> vel_controllers_;
ros::Publisher visPub;
ros::Subscriber js_sub_;
Eigen::VectorXd state_;
bool controller_started_;
std::string frame_id_;
VisualizationManager visMan;

void js_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // TODO: turn this into a map!
  // is there a more efficient way?
  for (unsigned int i=0; i < joint_names_.size(); i++)
  {
    for (unsigned int j=0; j < msg->name.size(); j++)
    {
      if (msg->name[j].compare(joint_names_[i]) == 0)
      {
        state_[i] = msg->position[j];
      }
    }
  }

  if (!controller_started_)
    return;

  if (controller_.update(state_, nWSR_))
  {
    Eigen::VectorXd commands = controller_.get_command();
    for (unsigned int i=0; i < vel_controllers_.size(); i++)
    {
      std_msgs::Float64 command;
      command.data = commands[i];
      vel_controllers_[i].publish(command);
    }
    //std::cout << "published new joint states" << std::endl;
  }
  else
  {
    ROS_WARN("Update failed.");
    // TODO: remove or change to ros_debug
    std::cout << "State " << state_ << std::endl;
  }
}

//void printGoal(const geometry_msgs::PoseStamped& goal)
//{
//  ROS_INFO("New goal: frame_id=%s\nposition=(%f, %f, %f), orientation=(%f, %f, %f, %f)", 
//      goal.header.frame_id.c_str(), 
//      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
//      goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z,
//      goal.pose.orientation.w);
//}
//

void print_eigen(const Eigen::VectorXd& command)
{
  std::string cmd_str = " ";
  for(size_t i=0; i<command.rows(); ++i)
    cmd_str += boost::lexical_cast<std::string>(command[i]) + " ";
  ROS_INFO("Command: (%s)", cmd_str.c_str());
}

void goal_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
//  printGoal(*msg);

  //std::cout << "received new goal" << std::endl;

  /*if(msg->header.frame_id.compare(frame_id_) != 0)
  {
    ROS_WARN("frame_id of right EE goal did not match expected '%s'. Ignoring goal", 
        frame_id_.c_str());
    return;
  }*/

  visMan.beginNewDrawCycle();
  double cWidth = 0.06;
  double cLength = 0.2;
  double cHeight = 0.06;

  Affine3d cakepos;

  geometry_msgs::Pose asd = *msg;

  tf::Pose temp;
  tf::poseMsgToTF(asd, temp);
  tf::transformTFToEigen(temp, cakepos);

  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(visMan.shapeMarker(0, 
                                  cakepos, 
                                  visualization_msgs::Marker::CUBE, 
                                  Vector3d(cLength, cHeight , cWidth ),
                                  0.f, 
                                  1.f, 
                                  0.f, 
                                  1.f, 
                                  "base_link"));

  visMan.endDrawCycle(markers.markers);
  visPub.publish(markers);

  // copying position goal
  state_[joint_names_.size() + 0] = msg->position.x;
  state_[joint_names_.size() + 1] = msg->position.y;
  state_[joint_names_.size() + 2] = msg->position.z;

  state_[joint_names_.size() + 3] = msg->orientation.x;
  state_[joint_names_.size() + 4] = msg->orientation.y;
  state_[joint_names_.size() + 5] = msg->orientation.z;
  state_[joint_names_.size() + 6] = msg->orientation.w;

  state_[joint_names_.size() + 7] = cLength;
  state_[joint_names_.size() + 8] = cWidth;
  state_[joint_names_.size() + 9] = cHeight;


  if (!controller_started_)
  {
    if (controller_.start(state_, nWSR_))
    {
      ROS_INFO("Controller started.");
      controller_started_ = true;
    }
    else
    {
      ROS_ERROR("Couldn't start controller.");
      print_eigen(state_);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_controller");
  ros::NodeHandle nh("~");

  nh.param("nWSR", nWSR_, 10);

  std::string controller_description;
  if (!nh.getParam("controller_description", controller_description))
  {
    ROS_ERROR("Parameter 'controller_description' not found in namespace '%s'.", nh.getNamespace().c_str());
    return 0;
  }

  if (!nh.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("Parameter 'joint_names' not found in namespace '%s'.", nh.getNamespace().c_str());
    return 0;
  }

  if (!nh.getParam("frame_id", frame_id_))
  {
    ROS_ERROR("Parameter 'frame_id' not found in namespace '%s'.", nh.getNamespace().c_str());
    return 0;
  }

  YAML::Node node = YAML::Load(controller_description);
  giskard::QPControllerSpec spec = node.as< giskard::QPControllerSpec >();
  controller_ = giskard::generate(spec);
  state_ = Eigen::VectorXd::Zero(joint_names_.size() + 10);
  controller_started_ = false;

  for (std::vector<std::string>::iterator it = joint_names_.begin(); it != joint_names_.end(); ++it)
    vel_controllers_.push_back(nh.advertise<std_msgs::Float64>("/" + it->substr(0, it->size() - 6) + "_velocity_controller/command", 1));

  ROS_INFO("Waiting for goal.");
  ros::Subscriber goal_sub = nh.subscribe("/goal", 0, goal_callback);
  js_sub_ = nh.subscribe("joint_states", 0, js_callback);

  // Initialize visualization publisher
  visPub  = nh.advertise<visualization_msgs::MarkerArray>("/graspkard_visualization", 1);
  visMan.addNamespace(0, "cylinder");

  ros::spin();

  return 0;
}
