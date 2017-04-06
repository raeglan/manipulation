#include <suturo_action_server/CollisionScene.h>
//#include <occupancy_map_monitor.h>
#include <moveit/depth_image_octomap_updater/depth_image_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>

CollisionScene::CollisionScene(QueryMap &_map) 
 : map(_map)
{
	nh.setCallbackQueue(&cbQueue);
	updateTimer = nh.createTimer(ros::Duration(0.025), &CollisionScene::updateQuery, this);
	ros::Subscriber sub = nh.subscribe("octomap_binary", 1000, &CollisionScene::update,this);
}

void CollisionScene::update(octomap_msgs::Octomap::ConstPtr& omap) {
	//occupancy_map_monitor::OccupancyMapMonitor monitor();
	//occupancy_map_monitor::DepthImageOctomapUpdater updater();


}

void CollisionScene::setRobotDescription(const string& urdfStr) {
	// TODO: GENERATE URDF
}

void CollisionScene::addQueryLink(const string& link) {
	links.insert(link);
}

void CollisionScene::clearQueryLinks() {
	links.clear();
}

void CollisionScene::updateQuery(const ros::TimerEvent& event) {
	// TODO: MAGIC
}