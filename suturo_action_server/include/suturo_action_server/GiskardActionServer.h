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
	 * 
	 * During construction, an actionlib action server is created and command and visualization topics are advertised. Additional configuration information is read from the parameter ~config on the parameter server.
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
	 * 
	 * The yaml structure must be a map.
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
	 * 
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
	
	/**
	 * @brief      Decodes a transform parameter from a string and writes it to the state vector.
	 * @param[in]  name   Name of the corresponding input in the current controller
	 * @param[in]  vector  Transform encoded as string: "X Y Z AX AY AZ Angle"
	 * @return     Success of decoding and value assignment.
	 */
	bool decodeTransform(const string& name, string transform);
	
	/**
	 * @brief      Assigns a transform parameter to the state vector.
	 * @param[in]  name   Name of the corresponding input in the current controller
	 * @param[in]  vector  Transform
	 * @return     Success of value assignment.
	 */
	bool decodeTransform(const string& name, tf::Transform transform);


protected:
	/** Iteration limit for giskard per update cycle */
	int nWSR;

	/** Flag set to terminate controller execution from deeper layers */
	bool terminateExecution;

	/** Name of the action server */
	const string name;

	/** Server's node handle */
	ros::NodeHandle nh;

	/** Action server instance */
	actionlib::SimpleActionServer<suturo_manipulation_msgs::MoveRobotAction> server;

	/** Current generated controller */
	giskard_core::QPController controller;
	
	/** Controller state vector */
	Eigen::VectorXd state;

	/** Map of all velocity controllers */
	map<std::string, ros::Publisher> velControllers;
	
	/**
	 * @brief      Internal structure to manage a position controller.
	 */
	struct posController {
		ros::Publisher pub;
		double dT;
	};

	/** Map of joints being controlled with position controllers */
	unordered_map<string, posController> posControllers;

	/**
	 * @brief      Internal structure to manage a gripper controller. 
	 */
	struct gripperController {
		ros::Publisher pub;
		double effort;
		double dT;
	}; 

	/** Map of joints being controlled with gripper controllers */
	unordered_map<string, gripperController> gripperControllers;

	/** Set containing the names of all joints referenced in current controller */
	unordered_set<string> jointSet;

	/** List of queries to run during update cycle */
	vector<boost::shared_ptr<AQuery>> queries;

	/** Map of last velocity commands */
	map<string, double> lastCommand;

	/** Map of last position commands */
	map<string, double> lastControllablePos;

	/** Time of last update cycle */
	ros::Time lastUpdate;

	/** Name of origin frame for visualization */
	string visRefFrameName;
private:
	/** Mutex used to allow for asynchronous joint state updates */
	mutex jsMutex;

	/**
	 * @brief      Parses controller scope for values to visualize.
	 *
	 * Values beginning with "VIS__" will be visualized. Their visual name is determined by the part following the prefix. If a vector should be visualized as direction, the name of a base point must be provided. This point's name is separated from the visual name with a double underscore "__".  
	 *
	 * @param[in]  scope  The scope
	 */
	void generateVisualsFromScope(const giskard_core::Scope& scope);
	
	/** Flag showing whether a new joint state has been received since last update cycle */
	bool newJS;

	/** Last received joint state */
	sensor_msgs::JointState currentJS;

	/** Seconds passed since last update cycle */
	double dT;

	/** Tranform listener used by Tf queries */
	tf::TransformListener tfListener;
	
	/** Value of feedback after last update cycle */
	double lastFeedback;

	/** Joint state subscriber */
	ros::Subscriber jsSub;

	/** Flag showing whether current controller was initialized */
	bool controllerInitialized;

	/** Pointer to feedback expression */
	KDL::Expression<double>::Ptr feedbackExpr;
	
	/** Type definition for vector visualization. */
	typedef std::pair<KDL::Expression<KDL::Vector>::Ptr, KDL::Expression<KDL::Vector>::Ptr> TVecPair;
	
	/** Scalars to visualize */
	unordered_map<string, KDL::Expression<double>::Ptr> visScalars;
	
	/** Points to visualize */
	unordered_map<string, KDL::Expression<KDL::Vector>::Ptr> visPoints;
	
	/** Vectors to visualize */
	unordered_map<string, TVecPair> visVectors;
	
	/** Frames to visualize */
	unordered_map<string, KDL::Expression<KDL::Frame>::Ptr> visFrames;

	/** Publisher for joint state commands */
	ros::Publisher jsCmdPub;

	/** Publishers for deviations of measurements and expectations */
	ros::Publisher posErrorPub, velErrorPub;

	/** Publishers for visualization */
	ros::Publisher visPub, visScalarPub;

	/** Visualization manager to manage Ids and deletion of markers */
	VisualizationManager visManager;

	/** Collision Scene */
	CollisionScene collisionScene;
	
	/** Results of collision scene updates */
	CollisionScene::QueryMap collQueryMap;

	/** Action server feedback message */
	suturo_manipulation_msgs::MoveRobotFeedback feedback;
	
	/** Action server result message */
	suturo_manipulation_msgs::MoveRobotResult result;
};


/**
 * @brief      Abstract base class for all action server queries
 * 
 * This abstract base class provides the data and function declarations the action server needs from a query to run it.
 */
struct AQuery {
	AQuery(GiskardActionServer* _pServer, string _name)
	: pServer(_pServer)
	, name(_name) 
	{
		assert(pServer);
	}

	virtual bool eval() = 0;
protected:
	/** Name of the controller input */
	const string name;

	/** Pointer to the action server */
	GiskardActionServer* pServer;
};


/**
 * @brief      Query looking up a transformation in Tf.
 */
struct TFQuery : public AQuery {
	TFQuery(GiskardActionServer* pS, string name, string targetFrame, string sourceFrame, tf::TransformListener* _tfListener) 
	: AQuery(pS, name)
	, frameId(targetFrame)
	, refFrame(sourceFrame)
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
	/** Target frame */
	const string frameId;

	/** Source frame */
	const string refFrame;
	
	/** Pointer to transform listener used for lookup */
	const tf::TransformListener* tfListener;
};


/**
 * @brief      Query assigning the time passed since the start of the controller to an input.
 */
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
	/** Time when controller started */
	const ros::Time start;
};


/**
 * @brief      Query updating the closest points between a robot link and the surroundings.
 */
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
	/** Name of the link */
	const string link;

	/** Names of the inputs */
	const string on_link, in_world;
	
	/** Result map of collision scene */
	CollisionScene::QueryMap& map;
};