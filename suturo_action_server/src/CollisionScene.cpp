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


void CollisionScene::traverseTree(SQueryPoints& qPoint){

	double dist = -1;

	for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end=octree->end_leafs(); it!= end; ++it)
 	{
   		//manipulate node, e.g.:
   		octomath::Vector3 point = it.getCoordinate();

   		Eigen::Vector3d pointEigen(point.x(), point.y(), point.z());

   		if(double newdist = (pointEigen - qPoint.onLink).norm() < dist || dist < 0){
   			qPoint.inScene = pointEigen;
   			dist = newdist;
   		}


   		


   		//std::cout << "Node center: " << it.getCoordinate() << std::endl;
   		//std::cout << "Node size: " << it.getSize() << std::endl;
   		//std::cout << "Node value: " << it->getValue() << std::endl;
	 }


	/*double occupancy = currentNode->getOccupancy();
	if(occupancy == 0){
		return;
	}

	if(currentNode->hasChildren()){
		for(int i = 0; i<8; i++){
			if(currentNode->childExists(i)){
				traverseTree(currentNode->getChild(i), minDist, qPoint);
			}
		}
	}else{
		octomath::Vector3 point3d = currentNode->getCoordinate();
	}*/
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

				//octomap::OcTreeNode *node = octree->getRoot();

				SQueryPoints qPoint;
				qPoint.onLink = iLink.translation();

				double dist;
				traverseTree(qPoint);

				map.set(linkName, qPoint, 1);
				

			} catch(tf::TransformException ex) {
				cerr << ex.what() << endl;
				ROS_WARN("TF-Query for link '%s' in '%s' failed!", linkName.c_str(), refFrame.c_str());
			}
		}
	}
}