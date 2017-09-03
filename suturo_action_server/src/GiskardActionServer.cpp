#include "suturo_action_server/GiskardActionServer.h"
#include <suturo_manipulation_msgs/TypedParam.h>
#include <suturo_manipulation_msgs/Float64Map.h>
#include <control_msgs/GripperCommand.h>
#include <giskard_suturo_parser/giskard_parser.hpp>
#include <giskard_suturo_parser/parser.h>
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
, rGripper_dT(0)
, lGripper_dT(0)
, rGripperIdx(-1)
, lGripperIdx(-1)
, rGripperEffort(30)
, lGripperEffort(30)
, newJS(false)
, name(_name)
, nWSR(1000)
, nh("~")
, server(nh, name, boost::bind(&GiskardActionServer::setGoal, this, _1), false)
, collisionScene(collQueryMap)
, visRefFrameName("base_link")
{
	
	posErrorPub = nh.advertise<suturo_manipulation_msgs::Float64Map>("pos_errors", 1);
	velErrorPub = nh.advertise<suturo_manipulation_msgs::Float64Map>("vel_errors", 1);

	visPub = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1);
	visScalarPub = nh.advertise<suturo_manipulation_msgs::Float64Map>("debug_scalar", 1);

	jsCmdPub = nh.advertise<sensor_msgs::JointState>("/simulator/commands", 1);

	jsSub = nh.subscribe("/joint_states", 1, &GiskardActionServer::updatejointState, this);

	visManager.addNamespace(vPoint, "Points");
	visManager.addNamespace(vVector, "Vectors");
	visManager.addNamespace(vFrame, "Frames");

	string urdf;
	ros::param::get("/robot_description", urdf);
	collisionScene.setRobotDescription(urdf);

	string config;
	if (nh.getParam("config", config)) {
		try {
			YAML::Node configNode = YAML::Load(config);
			loadConfig(configNode);
		} catch (const YAML::Exception& e) {
			ROS_ERROR("Parsing of given configuration failed!");
		}
	} else {
		ROS_INFO("No further configuration information given. Using default configuration.");
	}

	server.start();
}

void GiskardActionServer::loadConfig(YAML::Node configNode) {
	if (configNode["position_controllers"]) {
		map<string, string> ctrls = configNode["position_controllers"].as<map<string, string>>();
		for (auto it = ctrls.begin(); it != ctrls.end(); it++) {
			posControllers[it->first] = { nh.advertise<std_msgs::Float64>(it->second, 1), 0.0 };
		}
	}
	if (configNode["gripper_controllers"]) {
		map<string, string> ctrls = configNode["gripper_controllers"].as<map<string, string>>();
		for (auto it = ctrls.begin(); it != ctrls.end(); it++) {
			gripperControllers[it->first] = { nh.advertise<control_msgs::GripperCommand>(it->second, 1), 0.0, 0.0 };
		}	
	}
	if (configNode["joint_state_command_topic"]) {
		string topicName = configNode["joint_state_command_topic"].as<string>();
		jsCmdPub = nh.advertise<sensor_msgs::JointState>(topicName, 1);
	}
	if (configNode["visualization_target_frame"]) {
		visRefFrameName = configNode["visualization_target_frame"].as<string>();
	}
}

void GiskardActionServer::updatejointState(const sensor_msgs::JointState::ConstPtr& jointState) {
	if(jsMutex.try_lock()) {
		currentJS = *jointState;
		newJS = true;
		jsMutex.unlock();
	}
}

void GiskardActionServer::setGoal(const MoveRobotGoalConstPtr& goal) {
	//Node ctrlJoints = YAML::Load(goal->controlled_joints);
	terminateExecution = false;
	lastFeedback = 0;
	dT = 0;
	lastUpdate = ros::Time::now();

	visScalars.clear();
	visVectors.clear();
	visFrames.clear();

	velControllers.clear();
	lastControllablePos.clear();
	jointSet.clear();
	queries.clear();
	for (auto it = posControllers.begin(); it != posControllers.end(); it++) {
		it->second.dT = 0;
	}

	for (auto it = gripperControllers.begin(); it != gripperControllers.end(); it++) {
		it->second.dT = 0;
	}

	controllerInitialized = false;
	collisionScene.clearQueryLinks();
	collQueryMap.clear();
	
	try {
		Node yamlController = YAML::Load(goal->controller_yaml);
		giskard_core::QPControllerSpec spec = yamlController.as<giskard_core::QPControllerSpec>();
		controller = giskard_core::generate(spec);
	} catch (YAML::Exception e) {
		ROS_ERROR("%s", e.what());
		MoveRobotResult res;
		res.reason_for_termination = MoveRobotResult::DEFECT_YAML;
		try {
			giskard_suturo::GiskardLangParser glParser;
			giskard_core::QPControllerSpec spec = glParser.parseQPController(goal->controller_yaml);
			
			YAML::Node yamlSpec = YAML::Load("");
			yamlSpec["spec"] = spec;

			std::ofstream fout("last-gk-controller.yaml");
			fout << yamlSpec;
			fout.close();

			controller = giskard_core::generate(spec);

		} catch (const std::exception& e) {
			ROS_ERROR("%s", e.what());

			try {
				giskard_suturo::GiskardPPParser gppparser;
				giskard_core::QPControllerSpec spec = gppparser.parseFromString(goal->controller_yaml);
				controller = giskard_core::generate(spec);			
			} catch (const std::exception& e) {
				ROS_ERROR("%s", e.what());				
				server.setAborted(res);
				return;
			}
		}

	} catch (std::invalid_argument e) {
		ROS_ERROR("%s", e.what());
		MoveRobotResult res;
		res.reason_for_termination = MoveRobotResult::DEFECT_CONTROLLER;
		server.setAborted(res);
		return;
	}
	
	const giskard_core::Scope& scope = controller.get_scope();

	vector<string> jointInputs = controller.get_input_names(giskard_core::tJoint);
	unordered_set<string> controlledSet(controller.get_controllable_names().begin(), controller.get_controllable_names().end());
	
	const std::vector<std::string>& controllableNames = controller.get_controllable_names();

	lastCommand = controller.get_command_map();

	bool bControlledJointsEnded = false;

	for (size_t i = 0; i < jointInputs.size(); i++) {
		string jointName = jointInputs[i];
		jointSet.insert(jointName);
		
		if (controlledSet.find(jointName) != controlledSet.end()) {
			velControllers[jointName] = nh.advertise<std_msgs::Float64>("/" + jointName.substr(0, jointName.size() - 6) + "_velocity_controller/command", 1);
		}
	}

	vector<suturo_manipulation_msgs::TypedParam> backlog;

	auto inputMap = scope.get_input_map();
	for (auto it = jointInputs.begin(); it != jointInputs.end(); it++) {
		inputMap.erase(*it);
	}

	for (size_t i = 0; i < goal->params.size(); i++) {
		TypedParam p = goal->params[i];

		size_t effortSubstrIdx = p.name.find("_effort");
		if (effortSubstrIdx != string::npos) {
			string gripperName = p.name.substr(0, effortSubstrIdx);
			auto gIt = gripperControllers.find(gripperName);
			if (gIt != gripperControllers.end()) {
				gIt->second.effort = ::atof(p.value.c_str());
				continue;
			} else {
				ROS_WARN("Received effort parameter '%s', but can't find gripper '%s'! Interpreting it as normal parameter...", p.name.c_str(), gripperName.c_str());
			}
		}

		inputMap.erase(p.name);

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
				queries.push_back(boost::shared_ptr<AQuery>(new ElapsedTimeQuery(this, p.name)));
				break;
			case TypedParam::VECTOR:
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
						KDL::Expression<KDL::Vector>::Ptr vectorExpr = scope.find_vector_expression(primaryValue);
						visPoints[primaryValue] = vectorExpr;
						break;
					} catch (const std::invalid_argument& e) { }


					try {
						KDL::Expression<KDL::Frame>::Ptr frameExpr = scope.find_frame_expression(primaryValue);
						visFrames[primaryValue] = frameExpr;
						break;
					} catch (const std::invalid_argument& e) { }

					ROS_ERROR("Failed to find frame, point or scalar value with name '%s'", primaryValue.c_str());
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
				ROS_ERROR("Datatype of index %d is unknown! Parameter number %zu. Aborting goal!", p.type, i+1);
				MoveRobotResult res;
				res.reason_for_termination = MoveRobotResult::INVALID_DATATYPE;
				server.setAborted(res);
				return;
		}
	}

	unordered_map<string, pair<bool, bool>> collisionMap;
	auto inputIt = inputMap.begin();
	while(inputIt != inputMap.end()){
		std::string name = inputIt->first;
		
		if(name.find("COLL:") == 0 && name.size() > 7 && name[6] == ':'){
			string linkName = name.substr(7);
			auto lit = collisionMap.find(linkName);
			if (lit == collisionMap.end())
				collisionMap[linkName] = pair<bool, bool>(false, false);

			switch(name[5]) {
				case 'L':
					collisionMap[linkName].first = true;
				break;
				case 'W':
					collisionMap[linkName].second = true;
				break;
				default:
					ROS_ERROR("Malformed collision input '%s': Expected 'L' or 'W' as identification.", name.c_str());				
				break;
			}
			inputIt = inputMap.erase(inputIt);
			continue;
		}
		inputIt++;
	}

	for (auto it = collisionMap.begin(); it != collisionMap.end(); it++) {
		if (it->second.first && it->second.second) {
			queries.push_back(boost::shared_ptr<AQuery>(new CollisionQuery(this, it->first, collQueryMap)));	
			collisionScene.addQueryLink(it->first);
		} else {
			if (!it->second.first)
				ROS_ERROR("Collision input for link '%s' is missing on link point!", it->first.c_str());
			
			if (!it->second.second)
				ROS_ERROR("Collision input for link '%s' is missing in world point!", it->first.c_str());
			
			MoveRobotResult res;
			res.reason_for_termination = MoveRobotResult::DEFECT_CONTROLLER;
			server.setAborted(res);
			return;
		}
	}

	if (inputMap.size() > 0) {
		stringstream ss;
		for (auto it = inputMap.begin(); it != inputMap.end(); it++) {
			ss << endl << "   " << it->first;
		}
		ROS_ERROR("Not all inputs were assigned. Inputs missing assignment: %s", ss.str().c_str());
		MoveRobotResult res;
		res.reason_for_termination = MoveRobotResult::DEFECT_CONTROLLER;
		server.setAborted(res);
		return;
	}

	generateVisualsFromScope(scope);

	state = Eigen::VectorXd::Zero(controller.get_input_size());

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
		if (jsMutex.try_lock()) {
			if (newJS) {
				jointStateCallback(currentJS);
				newJS = false;
			}
			jsMutex.unlock();
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
	for (auto it = velControllers.begin(); it != velControllers.end(); it++) {
		std_msgs::Float64 command;
		command.data = 0.0;
		it->second.publish(command);
	}

	visManager.clearAllMarkers(&visPub);

	ros::spinOnce();
}	


void GiskardActionServer::jointStateCallback(const sensor_msgs::JointState jointStateMsg) {
	ros::Time now = ros::Time::now();
	dT = (now - lastUpdate).toSec();
	lastUpdate = now;

	map<string, double> vels;
	map<string, double> pos;

	for (size_t i = 0; i < jointStateMsg.name.size(); i++) {
		auto it = jointSet.find(jointStateMsg.name[i]);
		if (it != jointSet.end()) {
			controller.set_input(state, jointStateMsg.name[i], jointStateMsg.position[i]);
			if (lastCommand.find(jointStateMsg.name[i]) != lastCommand.end()) {
				vels[jointStateMsg.name[i]] = jointStateMsg.velocity[i];
				pos[jointStateMsg.name[i]] = jointStateMsg.position[i];
			}
		}
	}

	//Eigen::VectorXd posErrors = Eigen::VectorXd::Zero(controlledIndices.size());
	//Eigen::VectorXd velErrors = Eigen::VectorXd::Zero(controlledIndices.size());
	double dTjs = (jointStateMsg.header.stamp - lastUpdate).toSec();

	if (lastControllablePos.size() == vels.size()) {
		suturo_manipulation_msgs::Float64Map posErrorMsg;
		suturo_manipulation_msgs::Float64Map velErrorMsg;
		for (auto it = vels.begin(); it != vels.end(); it++) {
			posErrorMsg.names.push_back(it->first);
			posErrorMsg.values.push_back(pos[it->first] - (lastControllablePos[it->first] + lastCommand[it->first] * dTjs));
			velErrorMsg.names.push_back(it->first);
			velErrorMsg.values.push_back(it->second - lastCommand[it->first]);
			lastControllablePos[it->first] = pos[it->first];
		}
		posErrorPub.publish(posErrorMsg);
		velErrorPub.publish(velErrorMsg);
	} else {
		lastControllablePos = pos;
	}


	collisionScene.updateQuery();

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

		map<string, double> commands = controller.get_command_map();
		lastCommand = commands;

		sensor_msgs::JointState jsCmd;
        jsCmd.name.reserve(commands.size());
        jsCmd.position.reserve(commands.size());
        jsCmd.velocity.reserve(commands.size());
        jsCmd.effort.reserve(commands.size());

		lastUpdate = ros::Time::now();
		for (auto it = commands.begin(); it != commands.end(); it++) {
			double position = pos[it->first];
			
			jsCmd.name.push_back(it->first);
			jsCmd.velocity.push_back(it->second);
            jsCmd.position.push_back(0);
            jsCmd.effort.push_back(0);

			auto vIt = velControllers.find(it->first);
			if (vIt != velControllers.end()) {
				std_msgs::Float64 command;
				command.data = it->second;
				vIt->second.publish(command);
			}
			
			auto pIt = posControllers.find(it->first);
			if (pIt != posControllers.end()) {
				pIt->second.dT += dT;
				if (pIt->second.dT > 0.2) {
					std_msgs::Float64 command;
					command.data = position + it->second * pIt->second.dT;
					pIt->second.pub.publish(command);
					pIt->second.dT = 0;
				}	
			}

			auto gIt = gripperControllers.find(it->first);
			if (gIt != gripperControllers.end()) {
				gIt->second.dT += dT;	
				if (gIt->second.dT > 0.2) {
					control_msgs::GripperCommand cmd;
					cmd.position = position + it->second * gIt->second.dT;
					cmd.max_effort = gIt->second.effort;

					gIt->second.pub.publish(cmd);
					gIt->second.dT = 0;
				}
			}
		}

		jsCmdPub.publish(jsCmd);

		/// ---------------- VISUALIZATION --------------------

		suturo_manipulation_msgs::Float64Map scalars;

		for (auto it = visScalars.begin(); it != visScalars.end(); it++) {
			scalars.names.push_back(it->first);
			scalars.values.push_back(it->second->value());
		}

		visScalarPub.publish(scalars);

		visManager.beginNewDrawCycle();
		visualization_msgs::MarkerArray ma;

		for (auto it = visPoints.begin(); it != visPoints.end(); it++) {
			KDL::Vector pKDL = it->second->value();
			Eigen::Vector3d p;
			tf::vectorKDLToEigen(pKDL, p);
			visManager.annotatedPoint(ma.markers, vPoint, p, it->first, 0, 0.8f, 0, 1, visRefFrameName);
		}

		for (auto it = visVectors.begin(); it != visVectors.end(); it++) {
			KDL::Vector vaKDL = it->second.first->value();
			KDL::Vector vbKDL = it->second.second->value(); 
			Eigen::Vector3d va, vb;
			tf::vectorKDLToEigen(vaKDL, va);
			tf::vectorKDLToEigen(vbKDL, vb);
			visManager.annotatedVector(ma.markers, vVector, vb, va, it->first, 0, 0.8f, 0, 1, visRefFrameName);
		}

		for (auto it = visFrames.begin(); it != visFrames.end(); it++) {
			KDL::Frame fKDL = it->second->value();
			Eigen::Affine3d f = Affine3d::Identity();
			tf::transformKDLToEigen(fKDL,f);
			visManager.poseMarker(ma.markers, vFrame, f, 0.1, 1.f, visRefFrameName);
		}

		visManager.endDrawCycle(ma.markers);
		visPub.publish(ma);
		ros::spinOnce();

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

void GiskardActionServer::generateVisualsFromScope(const giskard_core::Scope& scope) {
	map<string, KDL::Expression<double>::Ptr > scalars = scope.get_scalar_expressions();
	for (auto it = scalars.begin(); it != scalars.end(); it++) {
		string name = it->first;
		if (name.find("VIS__") == 0 && name.size() > 5) {
			string visName = name.substr(5);
			visScalars[visName] = it->second;	
		}
	}

	// VECTORS AND POINTS
	map<string, KDL::Expression<KDL::Vector>::Ptr > vectors = scope.get_vector_expressions(); 
	for (auto it = vectors.begin(); it != vectors.end(); it++) {
		string name = it->first;
		if (name.find("VIS__") == 0 && name.size() > 5) {
			size_t secondSep = name.find("__", 5);
			if (secondSep != string::npos) {
				string refPoint = name.substr(secondSep + 2);
				try {
					KDL::Expression<KDL::Vector>::Ptr b = scope.find_vector_expression(refPoint);
					string visName = name.substr(5, secondSep - 5);
					visVectors[visName] = TVecPair(it->second, b);
				} catch (const std::invalid_argument& e) { 
					ROS_ERROR("Failed to find vector value with name '%s'", refPoint.c_str());
					continue;
				}
			} else {
				string visName = name.substr(5);
				visPoints[visName] = it->second;
			}
		}
	}

	//map<string, KDL::Expression<KDL::Rotation>::Ptr > rotations = scope.get_rotation_expressions();
	map<string, KDL::Expression<KDL::Frame>::Ptr > frames = scope.get_frame_expressions(); 
	for (auto it = frames.begin(); it != frames.end(); it++) {
		string name = it->first;
		if (name.find("VIS__") == 0 && name.size() > 5) {
			string visName = name.substr(5);
			visFrames[visName] = it->second;	
		}
	}
}

bool GiskardActionServer::decodeDouble(const std::string& name, string value) {
	try {
		controller.set_input(state, name, ::atof(value.c_str()));
		return true;
	} catch (const std::exception& e) {
		cerr << e.what() << endl;
	}
	return false;
}

bool GiskardActionServer::decodeDouble(const std::string& name, double value) {
	try {
		controller.set_input(state, name, value);
		return true;
	} catch (const std::exception& e) {
		cerr << e.what() << endl;
	}
	return false;
}

bool GiskardActionServer::decodeVector(const std::string& name, string vector) {
	istringstream ss(vector);
	double x, y, z;
	x = y = z = 0;
	ss >> x;
	ss >> y;
	ss >> z;
	try {
		controller.set_input(state, name, Eigen::Vector3d(x,y,z));
		return true;
	} catch (const std::exception& e) {
		cerr << e.what() << endl;
	}
	return false;
}

bool GiskardActionServer::decodeVector(const std::string& name, Eigen::Vector3d vector) {
	try {
		controller.set_input(state, name, vector);
		return true;
	} catch (const std::exception& e) {
		cerr << e.what() << endl;
	}
	return false;
}

bool GiskardActionServer::decodeTransform(const std::string& name, string transform) {
	istringstream ss(transform);
	double x, y, z, ax, ay, az, a;
	x = y = z = ax = ay = az = a = 0;
	ss >> x;
	ss >> y;
	ss >> z;
	ss >> ax;
	ss >> ay;
	ss >> az;
	ss >> a;
	string remaining = ss.str();
	Eigen::Vector3d position = Eigen::Vector3d(x,y,z);
	Eigen::Vector3d axis = Eigen::Vector3d(ax,ay,az);
	double angle = a;
	try {
		controller.set_input(state, name, axis, angle, position);
		return true;
	} catch (const std::exception& e) {
		cerr << e.what() << endl;
	}
	return false;
}

bool GiskardActionServer::decodeTransform(const std::string& name, tf::Transform transform) {
	tf::Vector3 pos = transform.getOrigin();
    tf::Vector3 rot = transform.getRotation().getAxis();
    double angle = transform.getRotation().getAngle();
	try {
		controller.set_input(state, name, 
			Eigen::Vector3d(rot.x(), rot.y(), rot.z()), angle,
			Eigen::Vector3d(pos.x(), pos.y(), pos.z()));
		return true;
	} catch (const std::exception& e) {
		cerr << e.what() << endl;
	}
	return false;
}