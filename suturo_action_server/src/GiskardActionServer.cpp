#include "suturo_action_server/GiskardActionServer.h"
#include <suturo_manipulation_msgs/TypedParam.h>
#include <control_msgs/GripperCommand.h>
#include <giskard/GiskardLangParser.h>

using namespace YAML;
using namespace suturo_manipulation_msgs;

void print_eigen(const Eigen::VectorXd& command)
{
  std::string cmd_str = " ";
  for(size_t i=0; i<command.rows(); ++i)
    cmd_str += boost::lexical_cast<std::string>(command[i]) + " ";
  ROS_INFO("Command: (%s)", cmd_str.c_str());
}

struct SBacklogParam {
	suturo_manipulation_msgs::TypedParam param;
	size_t idx;
};

GiskardActionServer::GiskardActionServer(string _name) 
: controllerInitialized(false)
, terminateExecution(false)
, lastFeedback(0)
, dT(0)
, rGripperIdx(-1)
, lGripperIdx(-1)
, rGripperEffort(30)
, lGripperEffort(30)
, name(_name)
, nWSR(1000)
, nh("~")
, server(nh, name, boost::bind(&GiskardActionServer::setGoal, this, _1), false)
, collisionScene(collQueryMap)
{
	rGripperPub = nh.advertise<control_msgs::GripperCommand>("r_pr2_gripper_command", 1);
	lGripperPub = nh.advertise<control_msgs::GripperCommand>("l_pr2_gripper_command", 1);

	string urdf;
	rosparam::get("robot_description", urdf);
	collisionScene.setRobotDescription(urdf);

	server.start();
}

void GiskardActionServer::setGoal(const MoveRobotGoalConstPtr& goal) {
	//Node ctrlJoints = YAML::Load(goal->controlled_joints);
	terminateExecution = false;
	lastFeedback = 0;
	dT = 0;
	lastUpdate = ros::Time::now();
	rGripperIdx = -1;
	lGripperIdx = -1;

	velControllers.clear();
	jointIndexMap.clear();
	queries.clear();
	for (size_t i = 0; i < goal->controlled_joints.size(); i++) {
		string jointName = goal->controlled_joints[i];
		jointIndexMap[goal->controlled_joints[i]] = i;
		velControllers.push_back(nh.advertise<std_msgs::Float64>("/" + jointName.substr(0, jointName.size() - 6) + "_velocity_controller/command", 1));
		if (jointName.compare("r_gripper_joint") == 0)
			rGripperIdx = i;
		else if (jointName.compare("l_gripper_joint") == 0)
			lGripperIdx = i;
	}

	controllerInitialized = false;
	collisionScene.clearQueryLinks();
	collQueryMap.clear();
	
	try {
		Node yamlController = YAML::Load(goal->controller_yaml);
		giskard::QPControllerSpec spec = yamlController.as<giskard::QPControllerSpec>();
		controller = giskard::generate(spec);
	} catch (YAML::Exception e) {
		ROS_ERROR("%s", e.what());
		MoveRobotResult res;
		res.reason_for_termination = MoveRobotResult::DEFECT_YAML;
		try {
			giskard::GiskardLangParser glParser;
			giskard::QPControllerSpec spec = glParser.parseQPController(goal->controller_yaml);
			
			YAML::Node yamlSpec = YAML::Load("");
			yamlSpec["spec"] = spec;

			std::ofstream fout("last-gk-controller.yaml");
			fout << yamlSpec;
			fout.close();

			controller = giskard::generate(spec);

		} catch (giskard::GiskardLangParser::EOSException e) {
			ROS_ERROR("%s", e.what());
			server.setAborted(res);
			return;
		} catch (giskard::GiskardLangParser::ParseException e) {
			ROS_ERROR("%s", e.what());
			server.setAborted(res);
			return;
		} catch (std::invalid_argument e) {
			ROS_ERROR("%s", e.what());
			res.reason_for_termination = MoveRobotResult::DEFECT_CONTROLLER;
			server.setAborted(res);
			return;
		}

	} catch (std::invalid_argument e) {
		ROS_ERROR("%s", e.what());
		MoveRobotResult res;
		res.reason_for_termination = MoveRobotResult::DEFECT_CONTROLLER;
		server.setAborted(res);
		return;
	}

	size_t paramLength = 0;
	size_t jntOffset = jointIndexMap.size();
	vector<SBacklogParam> backlog;

	for (size_t i = 0; i < goal->params.size(); i++) {
		TypedParam p = goal->params[i];
		SBacklogParam bp;
		bp.param = p;
		bp.idx = jntOffset + paramLength;

		if (p.name.compare("r_gripper_effort") == 0) {
			rGripperEffort = ::atof(p.value.c_str());
			continue;
		} else if (p.name.compare("l_gripper_effort") == 0) {
			lGripperEffort = ::atof(p.value.c_str());
			continue;
		}

		if (p.isConst)
			backlog.push_back(bp);

		switch (p.type) {
			case TypedParam::DOUBLE:
				paramLength++;
				break;
			case TypedParam::TRANSFORM:
				if (!p.isConst) {
					istringstream ss(p.value);
					string childframe, refFrame;
					ss >> childframe;
					ss >> refFrame;
					queries.push_back(boost::shared_ptr<AQuery>(
						new TFQuery(this, paramLength + jntOffset, childframe, refFrame, &tfListener)
						));
				}
				paramLength += 7;
				break;
			case TypedParam::ELAPSEDTIME:
				queries.push_back(boost::shared_ptr<AQuery>(
						new ElapsedTimeQuery(this, paramLength + jntOffset)
						));
				paramLength++;
				break;
			case TypedParam::VECTOR:
				paramLength += 3;
				break;
			case TypedParam::COLLISIONQUERY:
				if (!p.isConst) {
					queries.push_back(boost::shared_ptr<AQuery>(
						new CollisionQuery(this, paramLength + jntOffset, p.value, collQueryMap)
						));	

					collisionScene.addQueryLink(p.value);
				}
			default:
				ROS_ERROR("Datatype of index %d is unknown! Aborting goal!", p.type);
				MoveRobotResult res;
				res.reason_for_termination = MoveRobotResult::INVALID_DATATYPE;
				server.setAborted(res);
				return;
		}
	}

	state = Eigen::VectorXd::Zero(jointIndexMap.size() + paramLength);

	for (SBacklogParam bp: backlog) {
		switch (bp.param.type) {
			case TypedParam::DOUBLE:
				decodeDouble(bp.idx, bp.param.value);				
				break;
			case TypedParam::VECTOR:
				decodeVector(bp.idx, bp.param.value);
				break;
			case TypedParam::TRANSFORM:
				decodeTransform(bp.idx, bp.param.value);
				break;
			default:
				ROS_ERROR("Param type %d is unknown. Param: %s", bp.param.type, bp.param.name.c_str());
				break;
		}
	}

	ROS_INFO("Controller started");
	//jsSub = nh.subscribe("/joint_state", 1, &GiskardActionServer::jointStateCallback, this);
	const giskard::Scope& scope = controller.get_scope();
	try {
    	feedbackExpr = scope.find_double_expression(goal->feedbackValue);
	} catch (std::invalid_argument e) {
		ROS_ERROR("%s", e.what());
		MoveRobotResult res;
		res.reason_for_termination = MoveRobotResult::INVALID_FEEDBACK;
		server.setAborted(res);	
		return;	
	}
    ros::spinOnce();

	while (!server.isPreemptRequested() && ros::ok() && !terminateExecution) {
		boost::shared_ptr<const sensor_msgs::JointState> js = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
		jointStateCallback(js);
	}

	if (ros::ok()) {
		if (terminateExecution) {
			ROS_INFO("Server aborted due to infeasible goal!");
			MoveRobotResult res;
			res.reason_for_termination = MoveRobotResult::INFEASIBLE_GOAL;
			server.setAborted(res);
		} else {
			ROS_INFO("Server was preempted!");
			MoveRobotResult res;
			res.reason_for_termination = MoveRobotResult::PREEMPTED;
			server.setPreempted(res);
		}
	} else {
		ROS_INFO("Server was terminated!");
		MoveRobotResult res;
		res.reason_for_termination = MoveRobotResult::TERMINATED;
		server.setAborted(res);
	}

	ROS_INFO("Setting velocity commands to zero!");
	for (unsigned int i=0; i < velControllers.size(); i++) {
		std_msgs::Float64 command;
		command.data = 0.0;
		velControllers[i].publish(command);
	}

	ros::spinOnce();
}	

void GiskardActionServer::jointStateCallback(const sensor_msgs::JointState::ConstPtr& jointStateMsg) {
	ros::Time now = ros::Time::now();
	dT = (now - lastUpdate).toSec();
	lastUpdate = now;

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
		return;
	}

	updateLoop();

	if (!controllerInitialized) {
		if (controller.start(state, nWSR)) {
			controllerInitialized = true;
		} else {
			terminateExecution = true;
			ROS_ERROR("Starting of controller failed!");
			return;
		}
	}

	if (controller.update(state, nWSR)) {
		Eigen::VectorXd commands = controller.get_command();
		for (unsigned int i=0; i < velControllers.size(); i++) {
			std_msgs::Float64 command;
			command.data = commands[i];
			velControllers[i].publish(command);
		}

		if (rGripperIdx > -1) {
			control_msgs::GripperCommand cmd;
			cmd.position = state[rGripperIdx] + commands[rGripperIdx] * dT;
			cmd.max_effort = rGripperEffort;

			rGripperPub.publish(cmd);
		}

		if (lGripperIdx > -1) {
			control_msgs::GripperCommand cmd;
			cmd.position = state[lGripperIdx] + commands[lGripperIdx] * dT;
			cmd.max_effort = lGripperEffort;

			lGripperPub.publish(cmd);
		}

		feedback.current_value = feedbackExpr->value();
		feedback.alteration_rate = lastFeedback - feedback.current_value;
		lastFeedback = feedback.current_value;
		server.publishFeedback(feedback);
	} else {
		ROS_WARN("Update failed!");
		cerr << "State: " << state << endl;
	}
}

void GiskardActionServer::decodeDouble(size_t startIdx, string value) {
	state[startIdx] = ::atof(value.c_str());
}

void GiskardActionServer::decodeDouble(size_t startIdx, double value) {
	state[startIdx] = value;
}

void GiskardActionServer::decodeVector(size_t startIdx, string vector) {
	for (size_t i = 0; i < 3; i++)
		state[startIdx + i] = ::atof(vector.c_str());
}

void GiskardActionServer::decodeVector(size_t startIdx, Vector3d vector) {
	state[startIdx + 0] = vector[0];
	state[startIdx + 1] = vector[1];
	state[startIdx + 2] = vector[2];
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