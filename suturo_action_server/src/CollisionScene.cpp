#include <suturo_action_server/CollisionScene.h>
//#include <occupancy_map_monitor.h>
#include <moveit/depth_image_octomap_updater/depth_image_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>

#include <tf_conversions/tf_eigen.h>
#include <octomap_msgs/conversions.h>

using namespace Eigen;

CollisionScene::CollisionScene(QueryMap &_map) 
 : map(_map)		
{
	nh.setCallbackQueue(&cbQueue);
	updateTimer = nh.createTimer(ros::Duration(0.025), &CollisionScene::updateQuery, this);
	ros::Subscriber sub = nh.subscribe("octomap_binary", 1000, &CollisionScene::update,this);
}

void CollisionScene::update(octomap_msgs::Octomap omap) {
	//occupancy_map_monitor::OccupancyMapMonitor monitor();
	//occupancy_map_monitor::DepthImageOctomapUpdater updater();

	octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(omap);

	if (tree){
    	octree = dynamic_cast<octomap::OcTree*>(tree);
	 }

	 if (!octree)
	 {
	  	return;
	 }

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


void CollisionScene::traverseTree(octomap::OcTreeNode *currentNode, double& minDist){
	double occupancy = currentNode->getOccupancy();
	if(occupancy == 0){
		return;
	}

	if(currentNode->hasChildren()){
		for(int i = 0; i<8; i++){
			if(currentNode->childExists(i)){
				traverseTree(currentNode->getChild(i), minDist);
			}
		}
	}else{
		//calc dist
	}
}

void CollisionScene::updateQuery(const ros::TimerEvent& event) {
	// TODO: MAGIC

	for (const string& linkName: links) {
		if (linkMap.find(linkName) != linkMap.end()) {
			SRobotLink& link = linkMap[linkName];

			try {
				tf::StampedTransform temp;
				tfListener.waitForTransform(refFrame, linkName, ros::Time(0), ros::Duration(0.5));
				tfListener.lookupTransform(refFrame, linkName, ros::Time(0), temp);

				Affine3d tLink = Affine3d::Identity();
				tf::transformTFToEigen (temp, tLink);

				Affine3d iLink = tLink.inverse();

				// Iterate over Octomap

				//typedef NODE octomap::OcTreeBaseImpl< NODE, INTERFACE >::NodeType;

				octomap::OcTreeNode *node = octree->getRoot();

				double dist;
				traverseTree(node, dist);

				/*
				double asd = node->getOccupancy();
				octomap::OcTreeNode *node2 = node->getChild(1);

				unsigned int tree_depth = octree->getTreeDepth();

				for (octomap::OcTree::iterator it = octree->begin(tree_depth), end = octree->end(); it != end; ++it){

				}*/

				SQueryPoints points;

				map.set(linkName, points, 1);
				

			} catch(tf::TransformException ex) {
				cerr << ex.what() << endl;
				ROS_WARN("TF-Query for link '%s' in '%s' failed!", linkName.c_str(), refFrame.c_str());
			}
		}
	}
}