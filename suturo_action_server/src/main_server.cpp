#include "suturo_action_server/GiskardActionServer.h"

int main(int argc, char** argv) {
	std::string server_name("movement_server");
	ros::init(argc, argv, server_name);

	ROS_INFO("#############");
	GiskardActionServer movement_server(server_name);
	ROS_INFO("--- GISKARD IS NOW AWAITING YOUR ORDERS ---");
	ros::spin();
	return 0;
}