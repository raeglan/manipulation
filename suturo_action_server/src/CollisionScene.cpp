#include <suturo_action_server/CollisionScene.h>
//#include <occupancy_map_monitor.h>
#include <moveit/depth_image_octomap_updater/depth_image_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <tf/tf_eigen.h>

using namespace Eigen;

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
	octomap_msgs::Octomap::ConstPtr& omap;
	string refFrame = omap->header.frame_id;

	for (const string& linkName: links) {
		if (linkMap.find(linkName) != linkMap.end()) {
			SRobotLink& link = linkMap[linkName];

			try {
				tf::StampedTransform temp;
				tfListener->waitForTransform(refFrame, linkName, ros::Time(0), ros::Duration(0.5));
				tfListener->lookupTransform(refFrame, linkName, ros::Time(0), temp);

				Affine3d tLink = Affine3d::Identity();
				tf::transformTFToEigen (temp, tLink);

				Affine3d iLink = tLink.inverse();

				// Iterate over Octomap


				

			} catch(tf::TransformException ex) {
				cerr << ex.what() << endl;
				ROS_WARN("TF-Query for link '%s' in '%s' failed!", linkName.c_str(), refFrame.c_str());
			}
		}
	}
}