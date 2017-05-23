#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <suturo_manipulation_msgs/MoveRobotAction.h>
#include <actionlib/server/simple_action_server.h>
#include "suturo_action_server/CollisionScene.h"

#include <giskard/giskard.hpp>

#include <unordered_map>

using namespace std;

struct AQuery; 

class GiskardActionServer {
public:
	GiskardActionServer(string _name);

	virtual void setGoal(const suturo_manipulation_msgs::MoveRobotGoalConstPtr& goal);

	virtual void updateLoop() {};
	void jointStateCallback(const sensor_msgs::JointState::ConstPtr& jointStateMsg);

	void decodeDouble(size_t startIdx, string value);
	void decodeDouble(size_t startIdx, double value);
	void decodeVector(size_t startIdx, string vector);
	void decodeVector(size_t startIdx, Eigen::Vector3d vector);
	void decodeTransform(size_t startIdx, string transform);
	void decodeTransform(size_t startIdx, tf::Transform transform);
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
	};
	unordered_map<string, posController> posControllers;


	unordered_map<string, size_t> jointIndexMap;

	vector<boost::shared_ptr<AQuery>> queries;

private:
	ros::Time lastUpdate;
	double dT;
	tf::TransformListener tfListener;
	double lastFeedback;
	ros::Subscriber jsSub;
	bool controllerInitialized;
	KDL::Expression<double>::Ptr feedbackExpr;
	ros::Publisher rGripperPub, lGripperPub;
	ros::Subscriber rGripperSub, lGripperSub;

	CollisionScene collisionScene;
	CollisionScene::QueryMap collQueryMap;

	suturo_manipulation_msgs::MoveRobotFeedback feedback;
	suturo_manipulation_msgs::MoveRobotResult result;
};

struct AQuery {
	AQuery(GiskardActionServer* _pServer, size_t _idx)
	: pServer(_pServer)
	, idx(_idx) 
	{
		assert(pServer);
	}

	virtual bool eval() = 0;
protected:
	const size_t idx;
	GiskardActionServer* pServer;
};

struct TFQuery : public AQuery {
	TFQuery(GiskardActionServer* pS, size_t idx, string _frameId, string _refFrame, tf::TransformListener* _tfListener) 
	: AQuery(pS, idx)
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
			
			pServer->decodeTransform(idx, temp);
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
	ElapsedTimeQuery(GiskardActionServer* _pServer, size_t _idx)
	: AQuery(_pServer, _idx) 
	, start(ros::Time::now())
	{ }

	bool eval() {
		ros::Duration elapsed = ros::Time::now() - start;

		pServer->decodeDouble(idx, elapsed.toSec());
		return true;
	}

private:
	const ros::Time start;
};

struct CollisionQuery : public AQuery {
	CollisionQuery(GiskardActionServer* _pServer, size_t _idx, string _link, CollisionScene::QueryMap& _map)
	: AQuery(_pServer, _idx) 
	, link(_link)
	, map(_map)
	{ }

	bool eval() {
		CollisionScene::SQueryPoints points;
		if (map.get(link, points, READER_PID)) {
			pServer->decodeVector(idx, points.onLink);
			pServer->decodeVector(idx + 3, points.inScene);
			return true;
		}

		ROS_WARN("Collision query for link '%s' failed!", link.c_str());
		return false;
	}

private:
	const string link;
	CollisionScene::QueryMap& map;
};