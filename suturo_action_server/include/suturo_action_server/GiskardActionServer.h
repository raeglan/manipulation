#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <suturo_manipulation_msgs/MoveRobotAction.h>
#include <actionlib/server/simple_action_server.h>
#include "suturo_action_server/CollisionScene.h"

#include <r_libs/VisualizationManager.h>

#include <giskard_core/giskard_core.hpp>

#include <unordered_map>
#include <unordered_set>

#include <mutex>

using namespace std;

struct AQuery; 


/**
 * @brief      Class for giskard action server. Sets up action server, threads and callbacks.
 */
class GiskardActionServer {
	
	/**
	 * @brief      Enumeration for visualization types.
	 */
	enum VisTypes : int {
		vPoint,
		vVector,
		vFrame
	};
public:

	/**
	 * @brief      Constructor for giskard action server. 
	 * Constructor for giskard action server. During construction, an actionlib action server is created and command and visualization topics are advertised. Additional configuration information is read from the parameter ~config on the parameter server.
	 * 
	 * @param[in]  _name  The name that the actionlib action server will be created with
	 */
	GiskardActionServer(string _name);

	/**
	 * @brief      Goal callback for the action server
	 *
	 * @param[in]  goal  The received goal message
	 */
	virtual void setGoal(const suturo_manipulation_msgs::MoveRobotGoalConstPtr& goal);
	
	/**
	 * @brief      Configures the action server using a yaml configuration. 
	 * Configures the action server using a yaml configuration. The yaml structure must be a map.
	 * The map is scanned for the following keys:
	 * 	- position_controllers: Defines which joints are supposed be controlled using position commands  
	 * 	- gripper_controllers: Defines whoch joints should be controlled as grippers
	 * 	- joint_state_command_topic: Defines topic to which commands are published as sensor_msgs::JointState
	 * 	- visualization_target_frame: Names the root frame for visualization
	 * @param[in]  config  Configuration as yaml structure 
	 */
	virtual void loadConfig(YAML::Node config);

	/**
	 * @brief      Callback function for receiving joint states
	 * @param[in]  jointStateMsg  The new joint state message
	 */
	void updateJointState(const sensor_msgs::JointState::ConstPtr& jointStateMsg);

	/**
	 * @brief      Performs one controller update cycle
	 * This function performs one controller update. It evaluates all parameter queries, generates and sends new commands and updates the visualization.
	 * @param[in]  jointState  The current joint state
	 */
	void updateController(const sensor_msgs::JointState jointState);

	/**
	 * @brief      Decodes a scalar parameter from a string and writes it to the state vector.
	 * @param[in]  name   Name of the corresponding input in the current controller
	 * @param[in]  value  Scalar encoded as string
	 * @return     Success of decoding and value assignment.
	 */
	bool decodeDouble(const string& name, string value);
	
	/**
	 * @brief      Assigns a scalar parameter to the state vector.
	 * @param[in]  name   Name of the corresponding input in the current controller
	 * @param[in]  value  Scalar
	 * @return     Success of value assignment.
	 */
	bool decodeDouble(const string& name, double value);
	
	/**
	 * @brief      Decodes a vector parameter from a string and writes it to the state vector.
	 * @param[in]  name   Name of the corresponding input in the current controller
	 * @param[in]  vector  Vector encoded as string: "X Y Z"
	 * @return     Success of decoding and value assignment.
	 */
	bool decodeVector(const string& name, string vector);
	
	/**
	 * @brief      Assigns a vector parameter to the state vector.
	 * @param[in]  name   Name of the corresponding input in the current controller
	 * @param[in]  vector  Vector 
	 * @return     Success of value assignment.
	 */
	bool decodeVector(const string& name, Eigen::Vector3d vector);
	
	
	bool decodeTransform(const string& name, string transform);
	
	bool decodeTransform(const string& name, tf::Transform transform);


protected:
	int nWSR;
	bool terminateExecution;
	double rGripperEffort, lGripperEffort;
	int lGripperIdx, rGripperIdx;
	const string name;
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<suturo_manipulation_msgs::MoveRobotAction> server;

	giskard_core::QPController controller;
	Eigen::VectorXd state;
	map<std::string, ros::Publisher> velControllers;
	
	struct posController {
		ros::Publisher pub;
		double dT;
	};
	unordered_map<string, posController> posControllers;

	struct gripperController {
		ros::Publisher pub;
		double effort;
		double dT;
	}; 
	unordered_map<string, gripperController> gripperControllers;

	unordered_set<string> jointSet;

	vector<boost::shared_ptr<AQuery>> queries;

	map<string, double> lastCommand;
	map<string, double> lastControllablePos;
	ros::Time lastUpdate;

	string visRefFrameName;
private:
	mutex jsMutex;

	void generateVisualsFromScope(const giskard_core::Scope& scope);
	
	bool newJS;
	sensor_msgs::JointState currentJS;

	double dT, rGripper_dT, lGripper_dT;
	tf::TransformListener tfListener;
	double lastFeedback;
	ros::Subscriber jsSub;
	bool controllerInitialized;
	KDL::Expression<double>::Ptr feedbackExpr;
	ros::Publisher rGripperPub, lGripperPub;
	ros::Subscriber rGripperSub, lGripperSub;
	
	typedef std::pair<KDL::Expression<KDL::Vector>::Ptr, KDL::Expression<KDL::Vector>::Ptr> TVecPair;
	unordered_map<string, KDL::Expression<double>::Ptr> visScalars;
	unordered_map<string, KDL::Expression<KDL::Vector>::Ptr> visPoints;
	unordered_map<string, TVecPair> visVectors;
	unordered_map<string, KDL::Expression<KDL::Frame>::Ptr> visFrames;

	ros::Publisher jsCmdPub;
	ros::Publisher posErrorPub, velErrorPub;
	ros::Publisher visPub, visScalarPub;
	VisualizationManager visManager;

	CollisionScene collisionScene;
	CollisionScene::QueryMap collQueryMap;

	suturo_manipulation_msgs::MoveRobotFeedback feedback;
	suturo_manipulation_msgs::MoveRobotResult result;
};

struct AQuery {
	AQuery(GiskardActionServer* _pServer, string _name)
	: pServer(_pServer)
	, name(_name) 
	{
		assert(pServer);
	}

	virtual bool eval() = 0;
protected:
	const string name;
	GiskardActionServer* pServer;
};

struct TFQuery : public AQuery {
	TFQuery(GiskardActionServer* pS, string name, string _frameId, string _refFrame, tf::TransformListener* _tfListener) 
	: AQuery(pS, name)
	, frameId(_frameId)
	, refFrame(_refFrame)
	, tfListener(_tfListener)
	{
		assert(tfListener);
	}

	bool eval() {
		try {
			tf::StampedTransform temp;
			tfListener->waitForTransform(refFrame, frameId, ros::Time(0), ros::Duration(0.5));
			tfListener->lookupTransform(refFrame, frameId, ros::Time(0), temp);
			
			pServer->decodeTransform(name, temp);
		} catch(tf::TransformException ex) {
			cerr << ex.what() << endl;
			ROS_WARN("Query for frame '%s' in '%s' failed!", frameId.c_str(), refFrame.c_str());
			return false;
		}
		return true;
	}

private:
	const string frameId;
	const string refFrame;
	const tf::TransformListener* tfListener;
};

struct ElapsedTimeQuery : public AQuery {
	ElapsedTimeQuery(GiskardActionServer* _pServer, string _name)
	: AQuery(_pServer, _name) 
	, start(ros::Time::now())
	{ }

	bool eval() {
		ros::Duration elapsed = ros::Time::now() - start;

		pServer->decodeDouble(name, elapsed.toSec());
		return true;
	}

private:
	const ros::Time start;
};

struct CollisionQuery : public AQuery {
	CollisionQuery(GiskardActionServer* _pServer, string _link, CollisionScene::QueryMap& _map)
	: AQuery(_pServer, "Collision:"+_link) 
	, link(_link)
	, on_link("COLL:L:"+_link)
	, in_world("COLL:W:"+_link)
	, map(_map)
	{ }

	bool eval() {
		CollisionScene::SQueryPoints points;
		if (map.get(link, points)) {
			pServer->decodeVector(on_link, points.onLink);
			pServer->decodeVector(in_world, points.inScene);
			return true;
		}

		ROS_WARN("Collision query for link '%s' failed!", link.c_str());
		return false;
	}

private:
	const string link;
	const string on_link, in_world;
	CollisionScene::QueryMap& map;
};