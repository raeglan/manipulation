#include "suturo_action_server/GiskardActionServer.h"
#include <suturo_manipulation_msgs/TypedParam.h>
#include <suturo_manipulation_msgs/Float64Map.h>
#include <control_msgs/GripperCommand.h>
#include <giskard/GiskardLangParser.h>
#include <eigen_conversions/eigen_kdl.h>

#include <visualization_msgs/MarkerArray.h>

using namespace YAML;
using namespace suturo_manipulation_msgs;

void print_eigen(const Eigen::VectorXd& command)
{
  std::string cmd_str = " ";
  for(size_t i=0; i<command.rows(); ++i)
    cmd_str += boost::lexical_cast<std::string>(command[i]) + " ";
  ROS_INFO("Command: (%s)", cmd_str.c_str());
}

GiskardActionServer::GiskardActionServer(string _name) 
: controllerInitialized(false)
, terminateExecution(false)
, lastFeedback(0)
, dT(0)
, rGripperIdx(-1)
, lGripperIdx(-1)
, rGripperEffort(30)
, lGripperEffort(30)
, newJS(false)
, name(_name)
, nWSR(1000)
, nh("~")
, server(nh, name, boost::bind(&GiskardActionServer::setGoal, this, _1), false)
{
	rGripperPub = nh.advertise<control_msgs::GripperCommand>("r_pr2_gripper_command", 1);
	lGripperPub = nh.advertise<control_msgs::GripperCommand>("l_pr2_gripper_command", 1);

	visPub = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1);
	visScalarPub = nh.advertise<suturo_manipulation_msgs::Float64Map>("debug_scalar", 1);

	jsSub = nh.subscribe("/joint_states", 1, &GiskardActionServer::updatejointState, this);

	visManager.addNamespace(vVector, "Vectors");
	visManager.addNamespace(vFrame, "Frames");

	posControllers["head_tilt_joint"] = {
		nh.advertise<std_msgs::Float64>("/head_tilt_position_controller/command", 1),
		-1
	};
	posControllers["head_pan_joint"] = {
		nh.advertise<std_msgs::Float64>("/head_pan_position_controller/command", 1),
		-1
	};

	server.start();
}

void GiskardActionServer::updatejointState(const sensor_msgs::JointState::ConstPtr& jointState) {
	currentJS = *jointState;
	newJS = true;
}

void GiskardActionServer::setGoal(const MoveRobotGoalConstPtr& goal) {
	//Node ctrlJoints = YAML::Load(goal->controlled_joints);
	terminateExecution = false;
	lastFeedback = 0;
	dT = 0;
	lastUpdate = ros::Time::now();
	rGripperIdx = -1;
	lGripperIdx = -1;

	visScalars.clear();
	visVectors.clear();
	visFrames.clear();

	velControllers.clear();
	jointSet.clear();
	queries.clear();
	for (auto it = posControllers.begin(); it != posControllers.end(); it++)
		it->second.idx = -1;


	controllerInitialized = false;
	
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
	
	const std::vector<std::string>& controlled_joints = controller.get_controllable_names();
	for (size_t i = 0; i < controlled_joints.size(); i++) {
		string jointName = controlled_joints[i];
		jointSet.insert(jointName);
		velControllers.push_back(nh.advertise<std_msgs::Float64>("/" + jointName.substr(0, jointName.size() - 6) + "_velocity_controller/command", 1));
		if (jointName.compare("r_gripper_joint") == 0)
			rGripperIdx = i;
		else if (jointName.compare("l_gripper_joint") == 0)
			lGripperIdx = i;

		if (posControllers.find(jointName) != posControllers.end())
			posControllers[jointName].idx = i;
	}

	vector<suturo_manipulation_msgs::TypedParam> backlog;

	const giskard::Scope& scope = controller.get_scope();

	for (size_t i = 0; i < goal->params.size(); i++) {
		TypedParam p = goal->params[i];

		if (p.name.compare("r_gripper_effort") == 0) {
			rGripperEffort = ::atof(p.value.c_str());
			continue;
		} else if (p.name.compare("l_gripper_effort") == 0) {
			lGripperEffort = ::atof(p.value.c_str());
			continue;
		}

		if (p.isConst)
			backlog.push_back(p);

		switch (p.type) {
			case TypedParam::DOUBLE:
				break;
			case TypedParam::TRANSFORM:
				if (!p.isConst) {
					istringstream ss(p.value);
					string childframe, refFrame;
					ss >> childframe;
					ss >> refFrame;
					queries.push_back(boost::shared_ptr<AQuery>(
						new TFQuery(this, p.name, childframe, refFrame, &tfListener)
						));
				}
				break;
			case TypedParam::ELAPSEDTIME:
				queries.push_back(boost::shared_ptr<AQuery>(
						new ElapsedTimeQuery(this, p.name)
						));
				break;
			case TypedParam::VISUALIZE: {
				istringstream ss(p.value);
				string primaryValue, refPoint;
				ss >> primaryValue;
				ss >> refPoint;

				if (refPoint == "") {
					try {
						KDL::Expression<double>::Ptr dblExpr = scope.find_double_expression(primaryValue);
						visScalars[primaryValue] = dblExpr;
						break;
					} catch (const std::invalid_argument& e) { }

					try {
						KDL::Expression<KDL::Frame>::Ptr frameExpr = scope.find_frame_expression(primaryValue);
						visFrames[primaryValue] = frameExpr;
						break;
					} catch (const std::invalid_argument& e) { }

					ROS_ERROR("Failed to find frame or scalar value with name '%s'", primaryValue.c_str());
				} else {
					KDL::Expression<KDL::Vector>::Ptr a, b;
					try {
						a = scope.find_vector_expression(primaryValue);
					} catch (const std::invalid_argument& e) { 
						ROS_ERROR("Failed to find vector value with name '%s'", primaryValue.c_str());
						break;
					}

					try {
						b = scope.find_vector_expression(refPoint);
					} catch (const std::invalid_argument& e) { 
						ROS_ERROR("Failed to find vector value with name '%s'", refPoint.c_str());
						break;
					}

					visVectors[primaryValue] = TVecPair(a,b);
				}
			}
			break;
			default:
				ROS_ERROR("Datatype of index %d is unknown! Aborting goal!", p.type);
				MoveRobotResult res;
				res.reason_for_termination = MoveRobotResult::INVALID_DATATYPE;
				server.setAborted(res);
				return;
		}
	}

	state = Eigen::VectorXd::Zero(scope.get_input_size());

	for (auto p: backlog) {
		switch (p.type) {
			case TypedParam::DOUBLE:
				decodeDouble(p.name, p.value);				
				break;
			case TypedParam::VECTOR:
				decodeVector(p.name, p.value);
				break;
			case TypedParam::TRANSFORM:
				decodeTransform(p.name, p.value);
				break;
			default:
				ROS_ERROR("Param type %d is unknown. Param: %s", p.type, p.name.c_str());
				break;
		}
	}

	ROS_INFO("Controller started");
	//jsSub = nh.subscribe("/joint_state", 1, &GiskardActionServer::jointStateCallback, this);
	try {
    	feedbackExpr = scope.find_double_expression(goal->feedbackValue);
	} catch (const std::invalid_argument& e) {
		ROS_ERROR("%s", e.what());
		MoveRobotResult res;
		res.reason_for_termination = MoveRobotResult::INVALID_FEEDBACK;
		server.setAborted(res);	
		return;	
	}
    ros::spinOnce();

	while (!server.isPreemptRequested() && ros::ok() && !terminateExecution) {
		if (newJS) {
			jointStateCallback(currentJS);
			newJS = false;
		}
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

	visManager.clearAllMarkers(&visPub);

	ros::spinOnce();
}	


void GiskardActionServer::jointStateCallback(const sensor_msgs::JointState jointStateMsg) {
	ros::Time now = ros::Time::now();
	dT = (now - lastUpdate).toSec();
	lastUpdate = now;

	for (size_t i = 0; i < jointStateMsg.name.size(); i++) {
		auto it = jointSet.find(jointStateMsg.name[i]);
		if (it != jointSet.end()) {
			controller.get_scope().set_input(jointStateMsg.name[i], jointStateMsg.position[i], state);
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

		for (auto it = posControllers.begin(); it != posControllers.end(); it++) {
			if (it->second.idx > -1) {
				std_msgs::Float64 command;
				command.data = state[it->second.idx] + commands[it->second.idx] * dT;
				it->second.pub.publish(command);		
			}
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

		/// ---------------- VISUALIZATION --------------------

		suturo_manipulation_msgs::Float64Map scalars;

		for (auto it = visScalars.begin(); it != visScalars.end(); it++) {
			scalars.names.push_back(it->first);
			scalars.values.push_back(it->second->value());
		}

		visScalarPub.publish(scalars);

		visManager.beginNewDrawCycle();
		visualization_msgs::MarkerArray ma;

		for (auto it = visVectors.begin(); it != visVectors.end(); it++) {
			KDL::Vector vaKDL = it->second.first->value();
			KDL::Vector vbKDL = it->second.second->value(); 
			Eigen::Vector3d va, vb;
			tf::vectorKDLToEigen(vaKDL, va);
			tf::vectorKDLToEigen(vbKDL, vb);
			visManager.annotatedVector(ma.markers, vVector, vb, va, it->first, 0, 0.8f, 0, 1, "base_link");
		}

		for (auto it = visFrames.begin(); it != visFrames.end(); it++) {
			KDL::Frame fKDL = it->second->value();
			Eigen::Affine3d f = Affine3d::Identity();
			tf::transformKDLToEigen(fKDL,f);
			visManager.poseMarker(ma.markers, vFrame, f, 0.1, 1.f, "base_link");
		}

		visManager.endDrawCycle(ma.markers);
		visPub.publish(ma);

		/// ---------------------------------------------------

		feedback.current_value = feedbackExpr->value();
		feedback.alteration_rate = lastFeedback - feedback.current_value;
		lastFeedback = feedback.current_value;
		server.publishFeedback(feedback);
	} else {
		ROS_WARN("Update failed!");
		cerr << "State: " << state << endl;
	}
}

bool GiskardActionServer::decodeDouble(const std::string& name, string value) {
	return controller.get_scope().set_input(name, ::atof(value.c_str()), state);
}

bool GiskardActionServer::decodeDouble(const std::string& name, double value) {
	return controller.get_scope().set_input(name, value, state);
}

bool GiskardActionServer::decodeVector(const std::string& name, string vector) {
	const char* c_v = vector.c_str();
	return controller.get_scope().set_input(name, 
		Eigen::Vector3d(::atof(c_v), ::atof(c_v), ::atof(c_v)),
		state);
}

bool GiskardActionServer::decodeVector(const std::string& name, Eigen::Vector3d vector) {
	return controller.get_scope().set_input(name, vector, state);
}

bool GiskardActionServer::decodeTransform(const std::string& name, string transform) {
	const char* c_t = transform.c_str();
	return controller.get_scope().set_input(name, 
		Eigen::Vector3d(::atof(c_t), ::atof(c_t), ::atof(c_t)), ::atof(c_t),
		Eigen::Vector3d(::atof(c_t), ::atof(c_t), ::atof(c_t)), 
		state);
}

bool GiskardActionServer::decodeTransform(const std::string& name, tf::Transform transform) {
	tf::Vector3 pos = transform.getOrigin();
    tf::Vector3 rot = transform.getRotation().getAxis();
    double angle = transform.getRotation().getAngle();
	return controller.get_scope().set_input(name, 
			Eigen::Vector3d(rot.x(), rot.y(), rot.z()), angle,
			Eigen::Vector3d(pos.x(), pos.y(), pos.z()), state);
}