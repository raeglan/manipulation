#pragma once

class GiskardActionServer {
public:
	GiskardActionServer(string _name);

	virtual void setGoal(const suturo_manipulation_msgs::MoveRobotGoalConstPtr& goal);

	virtual void updateLoop() {};
	void jointStateCallback(const sensor_msgs::JointState::ConstPtr& jointStateMsg);

protected:
	int nWSR;
	const string name;
	ros::NodeHandle nh;
	actionlib::ActionServer<suturo_manipulation_msgs::MoveRobotAction> server;

	griskard::QPController controller;
	Eigen::VectorXd state;
	vector<ros::Publisher<std_msgs::Float64>> velControllers;

	unordered_map<string, size_t> jointIndexMap;
private:
	ros::Subscriber jsSub;
	bool controllerInitialized;
};