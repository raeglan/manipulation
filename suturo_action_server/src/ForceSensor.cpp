#include "suturo_action_server/ForceSensor.h"

#include <urdf/model.h>

SJointState::SJointState(const sensor_msgs::JointState& js) {
	stamp = js.header.stamp;
	for (size_t i = 0; i < js.name.size(); i++) {
		positions[js.name[i]]  = js.position[i];
		velocities[js.name[i]] = js.velocity[i];
		efforts[js.name[i]]    = js.effort[i];
	}
}

void ForceSensor::addCommand(const std::string& joint, double velocity, ros::Time stamp) {
	lastCommand[joint] = {stamp, velocity};
}

void ForceSensor::updateExpectations(const sensor_msgs::JointState& js) {

}

void ForceSensor::setURDF(const urdf::Model& urdf) {
    lastCommand.clear();
    //errors.clear();
    // for (auto it = urdf.joints_.begin(); it != urdf.joints_.end(); it++) {
    // 	if (it->second->type == urdf::Joint::REVOLUTE || it->second->type == urdf::Joint::PRISMATIC) {
    // 		joints[it->first] = it->second;
    // 	}
    // }
}