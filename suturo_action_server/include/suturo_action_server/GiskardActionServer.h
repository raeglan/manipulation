#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <suturo_manipulation_msgs/MoveRobotAction.h>
#include <actionlib/server/simple_action_server.h>

#include <r_libs/VisualizationManager.h>

#include <giskard/giskard.hpp>

#include <unordered_map>
#include <unordered_set>

using namespace std;

struct AQuery; 

class GiskardActionServer {
	enum VisTypes : int {
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
	};
	unordered_map<string, posController> posControllers;


	unordered_set<string> jointSet;

	vector<boost::shared_ptr<AQuery>> queries;

private:
	bool newJS;
	sensor_msgs::JointState currentJS;

	ros::Time lastUpdate;
	double dT;
	tf::TransformListener tfListener;
	double lastFeedback;
	ros::Subscriber jsSub;
	bool controllerInitialized;
	KDL::Expression<double>::Ptr feedbackExpr;
	ros::Publisher rGripperPub, lGripperPub;
	ros::Subscriber rGripperSub, lGripperSub;
	
	typedef std::pair<KDL::Expression<KDL::Vector>::Ptr, KDL::Expression<KDL::Vector>::Ptr> TVecPair;
	unordered_map<string, KDL::Expression<double>::Ptr> visScalars;
	unordered_map<string, TVecPair> visVectors;
	unordered_map<string, KDL::Expression<KDL::Frame>::Ptr> visFrames;

	ros::Publisher visPub, visScalarPub;
	VisualizationManager visManager;

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