#include "GiskardActionServer.h"

using namespace YAML;

GiskardActionServer::GiskardActionServer(string _name) 
: controllerInitialized(false)
, name(_name)
, nWSR(100)
, nh("~")
, server(nh, name, boost::bind(&GiskardActionServer::setGoal, this, _1), false)
{
	server.start();
}

void GiskardActionServer::setGoal(const suturo_manipulation_msgs::MoveRobotGoalConstPtr& goal) {
	//Node ctrlJoints = YAML::Load(goal.controlled_joints);

	velControllers.clear();
	jointIndexMap.clear();
	for (size_t i = 0; i < goal.controlled_joints.size(); i++) {
		string jointName = goal.controlled_joints[i];
		jointIndexMap[goal.controlled_joints[i]] = i;
		velControllers.push_back(nh.advertise<std_msgs::Float64>("/" + jointName.substr(0, jointName.size() - 6) + "_velocity_controller/command", 1));
	}

	Node yamlController = YAML::Load(goal.controller_yaml);
	giskard::QPControllerSpec spec = yamlController.as<giskard::QPControllerSpec>();
	controller = giskard::generate(spec);
	controllerInitialized = false;

	for (size_t i = 0; i < goal.params.size(); i++) {

	}

	state = Eigen::VectorXd::Zero(jointIndexMap.size() + goal.params.size());

	if (controller.start(state, nWSR)) {
		controllerInitialized = true;
		ROS_INFO("Controller started");
		jsSub = nh.subscribe("/joint_state", 1, GiskardActionServer::jointStateCallback, this);
	} else {
		ROS_ERROR("Starting of controller failed!");
	}
}	

void GiskardActionServer::jointStateCallback(const sensor_msgs::JointState::ConstPtr& jointStateMsg) {
	if (!controllerInitialized)
		return;

	for (size_t i = 0; i < jointStateMsg->name.size(); i++) {
		auto it = jointIndexMap.find(jointStateMsg->name[i]);
		if (it != jointIndexMap.end()) {
			state[it->second] = jointStateMsg->position[i];
		}
	}

	updateLoop();

	if (controller.update(state, nWSR)) {
		Eigen::VectorXd commands = controller.get_command();
		for (unsigned int i=0; i < velControllers.size(); i++) {
			std_msgs::Float64 command;
			command.data = commands[i];
			velControllers[i].publish(command);
		}
	} else {
		ROS_WARN("Update failed!");
		cerr << "State: " << state << endl;
	}
}