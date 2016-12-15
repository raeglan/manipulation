#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <suturo_manipulation_msgs/MoveRobotAction.h>
#include <actionlib/server/simple_action_server.h>

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