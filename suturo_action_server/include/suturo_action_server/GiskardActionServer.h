#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <suturo_manipulation_msgs/MoveRobotAction.h>
#include <actionlib/server/simple_action_server.h>
#include "suturo_action_server/CollisionScene.h"

#include <r_libs/VisualizationManager.h>

#include <giskard/giskard.hpp>

#include <unordered_map>
#include <unordered_set>

#include <mutex>

using namespace std;

struct AQuery; 

class GiskardActionServer {
	enum VisTypes : int {
		vPoint,
		vVector,
		vFrame
	};
public:
	GiskardActionServer(string _name);

	virtual void setGoal(const suturo_manipulation_msgs::MoveRobotGoalConstPtr& goal);

	virtual void updateLoop() {};
	void jointStateCallback(const sensor_msgs::JointState jointStateMsg);

	void updatejointState(const sensor_msgs::JointState::ConstPtr& jointState);

	bool decodeDouble(const string& name, string value);
	bool decodeDouble(const string& name, double value);
	bool decodeVector(const string& name, string vector);
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

	giskard::QPController controller;
	Eigen::VectorXd state;
	vector<ros::Publisher> velControllers;
	
	struct posController {
		ros::Publisher pub;
		int idx;
		double dT;
	};
	unordered_map<string, posController> posControllers;

	unordered_map<string, size_t> controlledIndices;

	unordered_set<string> jointSet;

	vector<boost::shared_ptr<AQuery>> queries;

	Eigen::VectorXd lastCommand;
	Eigen::VectorXd lastControllablePos;
	ros::Time lastUpdate;

private:
	mutex jsMutex;

	void generateVisualsFromScope(const giskard::Scope& scope);
	
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