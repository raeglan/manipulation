#include "suturo_action_server/GiskardActionServer.h"
#include <std_msgs/Float64.h>
#include <suturo_manipulation_msgs/TypedParam.h>

using namespace YAML;
using namespace suturo_manipulation_msgs;

struct SBacklogParam {
	suturo_manipulation_msgs::TypedParam param;
	size_t idx;
};

GiskardActionServer::GiskardActionServer(string _name) 
: controllerInitialized(false)
, lastFeedback(0)
, name(_name)
, nWSR(100)
, nh("~")
, server(nh, name, boost::bind(&GiskardActionServer::setGoal, this, _1), false)
{
	server.start();
}

void GiskardActionServer::setGoal(const MoveRobotGoalConstPtr& goal) {
	//Node ctrlJoints = YAML::Load(goal->controlled_joints);
	lastFeedback = 0;

	velControllers.clear();
	jointIndexMap.clear();
	queries.clear();
	for (size_t i = 0; i < goal->controlled_joints.size(); i++) {
		string jointName = goal->controlled_joints[i];
		jointIndexMap[goal->controlled_joints[i]] = i;
		velControllers.push_back(nh.advertise<std_msgs::Float64>("/" + jointName.substr(0, jointName.size() - 6) + "_velocity_controller/command", 1));
	}

	Node yamlController = YAML::Load(goal->controller_yaml);
	giskard::QPControllerSpec spec = yamlController.as<giskard::QPControllerSpec>();
	controller = giskard::generate(spec);
	controllerInitialized = false;

	size_t paramLength = 0;
	size_t jntOffset = jointIndexMap.size();
	vector<SBacklogParam> backlog;

	for (size_t i = 0; i < goal->params.size(); i++) {
		TypedParam p = goal->params[i];
		SBacklogParam bp;
		bp.param = p;
		bp.idx = paramLength;

		if (p.isConst)
			backlog.push_back(bp);

		switch (p.type) {
			case TypedParam::DOUBLE:
				paramLength++;
				break;
			case TypedParam::TRANSFORM:
				if (!p.isConst) {
					queries.push_back(boost::shared_ptr<AQuery>(
						new TFQuery(this, paramLength + jntOffset, p.value, "base_link", &tfListener)
						));
				}
				paramLength += 7;
				break;
			default:
				ROS_ERROR("STFU, wat is %d?!", p.type);
		}
	}

	state = Eigen::VectorXd::Zero(jointIndexMap.size() + paramLength);

	for (SBacklogParam bp: backlog) {
		switch (bp.param.type) {
			case TypedParam::DOUBLE:
				decodeDouble(bp.idx, bp.param.value);				
				break;
			case TypedParam::TRANSFORM:
				decodeTransform(bp.idx, bp.param.value);
				break;
			default:
				ROS_ERROR("Param type %d is unknown. Param: %s", bp.param.type, bp.param.name.c_str());
				break;
		}
	}

	if (controller.start(state, nWSR)) {
		controllerInitialized = true;
		ROS_INFO("Controller started");
		jsSub = nh.subscribe("/joint_state", 1, &GiskardActionServer::jointStateCallback, this);
		const giskard::Scope& scope = controller.get_scope();
	    feedbackExpr = scope.find_double_expression(goal->feedbackValue);
	
	    ros::spinOnce();
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

	bool ok = true;
	for (size_t i = 0; i < queries.size() && ok; i++) {
		ok &= queries[i]->eval();
	}

	if (!ok) {
		ROS_ERROR("Query evaluation failed! Skipping this update step.");
	}

	updateLoop();

	if (controller.update(state, nWSR)) {
		Eigen::VectorXd commands = controller.get_command();
		for (unsigned int i=0; i < velControllers.size(); i++) {
			std_msgs::Float64 command;
			command.data = commands[i];
			velControllers[i].publish(command);
		}

		feedback.current_value = feedbackExpr->value();
		feedback.alteration_rate = lastFeedback - feedback.current_value;
		lastFeedback = feedback.current_value;
	} else {
		ROS_WARN("Update failed!");
		cerr << "State: " << state << endl;
	}
}

void GiskardActionServer::decodeDouble(size_t startIdx, string value) {
	state[startIdx] = ::atof(value.c_str());
}

void GiskardActionServer::decodeTransform(size_t startIdx, string transform) {
	for (size_t i = 0; i < 7; i++)
		state[startIdx + i] = ::atof(transform.c_str());
}

void GiskardActionServer::decodeTransform(size_t startIdx, tf::Transform transform) {
	tf::Vector3 pos = transform.getOrigin();
    tf::Vector3 rot = transform.getRotation().getAxis();
    double angle = transform.getRotation().getAngle();

	state[startIdx + 0] = pos.x();
    state[startIdx + 1] = pos.y();
    state[startIdx + 2] = pos.z();

    state[startIdx + 3] = rot.x();
    state[startIdx + 4] = rot.y();
    state[startIdx + 5] = rot.z();
    state[startIdx + 6] = angle;
}