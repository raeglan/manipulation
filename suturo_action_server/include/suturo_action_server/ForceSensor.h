#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <unordered_map>

namespace urdf {
  class Model;
  class Joint;
}

using namespace std;

struct SJointState {
	SJointState(const sensor_msgs::JointState& js);

	ros::Time stamp;
	unordered_map<string, double> positions;
	unordered_map<string, double> velocities;
	unordered_map<string, double> efforts;
};

struct SJointCommand {
	ros::Time stamp;
	double velocity;
};

class ForceSensor {
public:

	void addCommand(const string& joint, double velocity, ros::Time stamp = ros::Time::now());
	void updateExpectations(const sensor_msgs::JointState& js);
	void setURDF(const urdf::Model& urdf);

private:
	double delay;
	unordered_map<string, SJointCommand> lastCommand;
	unordered_map<string, const std::shared_ptr<urdf::Joint>> joints;
};
