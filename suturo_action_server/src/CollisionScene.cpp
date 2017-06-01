#include <suturo_action_server/CollisionScene.h>
//#include <occupancy_map_monitor.h>
#include <moveit/depth_image_octomap_updater/depth_image_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>

#include <tf_conversions/tf_eigen.h>
#include <octomap_msgs/conversions.h>

#include <tf/transform_broadcaster.h>

#include <iostream>

using namespace Eigen;

CollisionScene::CollisionScene(QueryMap &_map) 
 : map(_map)		
{
	//nh.setCallbackQueue(&cbQueue);
	//updateTimer = nh.createTimer(ros::Duration(0.025), &CollisionScene::updateQuery, this);
	sub = nh.subscribe("/octomap_binary", 10, &CollisionScene::update, this);
	//ros::spin();
}

void CollisionScene::update(const octomap_msgs::Octomap &omap) {
	//occupancy_map_monitor::OccupancyMapMonitor monitor();
	//occupancy_map_monitor::DepthImageOctomapUpdater updater();

	refFrame = omap.header.frame_id;

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


	tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin( tf::Vector3(qPoint.onLink.x(), qPoint.onLink.y(), qPoint.onLink.z()) );
	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "asdLink"));



	double dist = -1;

	tf::StampedTransform temp;
	tfListener.waitForTransform("odom_combined", "base_link", ros::Time(0), ros::Duration(0.5));
	tfListener.lookupTransform("odom_combined", "base_link", ros::Time(0), temp);

	Affine3d tPoint = Affine3d::Identity();
	tf::transformTFToEigen (temp, tPoint);

	//Affine3d tPoint = tPoint.inverse();


	if(octree == NULL){
		return;
	}

	for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end=octree->end_leafs(); it!= end; ++it)
 	{


   		//manipulate node, e.g.:
   		octomath::Vector3 point = it.getCoordinate();

   		Eigen::Vector3d pointEigen(point.x(), point.y(), point.z());

   		double newdist = (pointEigen - qPoint.onLink).norm();

   		if(newdist < 0.15){
   			transform.setOrigin( tf::Vector3(point.x(), point.y(), point.z()) );
			transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

			std::cout << "Node value: " << it->getValue() << std::endl;
			std::cout << "Node size: " << it.getSize() << std::endl;
			//std::cout << "Node asd: " << it.getOccupancy() << std::endl;

			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "currentPoint"));
   		}

   		double nodeValue = it->getValue();

   		if((newdist < dist && nodeValue > 0) || (dist < 0 && nodeValue > 0)){

   			qPoint.inScene = tPoint * pointEigen;
   			dist = newdist;
   		}


   		


   		//std::cout << "Node center: " << it.getCoordinate() << std::endl;
   		//std::cout << "Node size: " << it.getSize() << std::endl;
   		//std::cout << "Node value: " << it->getValue() << std::endl;
	 }

	 qPoint.onLink = Eigen::Vector3d(0, 0, 0);


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



/*
void CollisionScene::traverseTree(SQueryPoints& qPoint, double &minDist, octomap::OcTreeNode *currentNode){


	tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin( tf::Vector3(qPoint.onLink.x(), qPoint.onLink.y(), qPoint.onLink.z()) );
	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "asdLink"));


/*
	double dist = -1;

	tf::StampedTransform temp;
	tfListener.waitForTransform("odom_combined", "base_link", ros::Time(0), ros::Duration(0.5));
	tfListener.lookupTransform("odom_combined", "base_link", ros::Time(0), temp);

	Affine3d tPoint = Affine3d::Identity();
	tf::transformTFToEigen (temp, tPoint);

	//Affine3d tPoint = tPoint.inverse();


	if(octree == NULL){
		return;
	}

	for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end=octree->end_leafs(); it!= end; ++it)
 	{


   		//manipulate node, e.g.:
   		octomath::Vector3 point = it.getCoordinate();

   		Eigen::Vector3d pointEigen(point.x(), point.y(), point.z());

   		double newdist = (pointEigen - qPoint.onLink).norm();

   		if(newdist < 0.15){
   			transform.setOrigin( tf::Vector3(point.x(), point.y(), point.z()) );
			transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

			std::cout << "Node value: " << it->getValue() << std::endl;
			std::cout << "Node size: " << it.getSize() << std::endl;
			//std::cout << "Node asd: " << it.isOccupied() << std::endl;

			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "currentPoint"));
   		}

   		if(newdist < dist || dist < 0){

   			qPoint.inScene = tPoint * pointEigen;
   			dist = newdist;
   		}


   		


   		//std::cout << "Node center: " << it.getCoordinate() << std::endl;
   		//std::cout << "Node size: " << it.getSize() << std::endl;
   		//std::cout << "Node value: " << it->getValue() << std::endl;
	 }

	 */

/*
	double occupancy = currentNode->getOccupancy();
	if(occupancy == 0){
		return;
	}

	if(currentNode->hasChildren()){
		for(int i = 0; i<8; i++){
			if(currentNode->childExists(i)){
				traverseTree(qPoint, minDist, currentNode->getChild(i));
			}
		}
	}else{
		octomath::Vector3 point = currentNode->getCoordinate();


		Eigen::Vector3d pointEigen(point.x(), point.y(), point.z());

   		double newdist = (pointEigen - qPoint.onLink).norm();

   		if(newdist < 0.15){
   			transform.setOrigin( tf::Vector3(point.x(), point.y(), point.z()) );
			transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

			//std::cout << "Node value: " << it->getValue() << std::endl;
			//std::cout << "Node size: " << it.getSize() << std::endl;
			//std::cout << "Node asd: " << it.isOccupied() << std::endl;

			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "currentPoint"));
   		}

   		if(newdist < minDist || minDist < 0){

   			qPoint.inScene = tPoint * pointEigen;
   			minDist = newdist;
   		}





	}
}*/

void CollisionScene::updateQuery() {
	// TODO: MAGIC

	for (const string& linkName: links) {
		//if (linkMap.find(linkName) != linkMap.end()) {
			//SRobotLink& link = linkMap[linkName];

			try {
				tf::StampedTransform temp;
				tfListener.waitForTransform(refFrame, linkName, ros::Time(0), ros::Duration(0.5));
				tfListener.lookupTransform(refFrame, linkName, ros::Time(0), temp);

				Affine3d tLink = Affine3d::Identity();
				tf::transformTFToEigen (temp, tLink);

				//Affine3d iLink = tLink.inverse();

				// Iterate over Octomap

				octomap::OcTreeNode *node = octree->getRoot();

				SQueryPoints qPoint;
				qPoint.onLink = tLink.translation();
				//qPoint.onLink = Eigen::Vector3d(0, 0, 0);

				double dist = -1;
				traverseTree(qPoint);

				qPoint.onLink = Eigen::Vector3d(0, 0, 0);

				map.set(linkName, qPoint, 1);
				

			} catch(tf::TransformException ex) {
				cerr << ex.what() << endl;
				ROS_WARN("TF-Query for link '%s' in '%s' failed!", linkName.c_str(), refFrame.c_str());
			}
		//}
	}
}