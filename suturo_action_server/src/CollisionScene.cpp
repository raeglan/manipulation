#include <suturo_action_server/CollisionScene.h>
//#include <occupancy_map_monitor.h>
//#include <moveit/depth_image_octomap_updater/depth_image_octomap_updater.h>
//#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>

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
		// octoMapMutex.lock();
    	octree = dynamic_cast<octomap::OcTree*>(tree);
		// octoMapMutex.unlock();
	}

	 //if (!octree)
	 //{
	 // 	return;
	 //}

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


Vector3d CollisionScene::calcIntersection(const Vector3d &v, const bBox &box) {
	Vector3d scaledX = v / abs(v.x());
	Vector3d xintersect = scaledX * box.x;
	if (xintersect.y() <= box.y && xintersect.z() <= box.z && xintersect.y() >= -box.y && xintersect.z() >= -box.z) {
		return xintersect;
	}
	
	Vector3d scaledY = v / abs(v.y());
	Vector3d yintersect = scaledY * box.y;
	if (yintersect.x() <= box.x && yintersect.z() <= box.z && yintersect.x() >= -box.x && yintersect.z() >= -box.z) {
		return yintersect;
	}

	Vector3d scaledZ = v / abs(v.z());
	Vector3d zintersect = scaledZ * box.z;
	if (zintersect.y() <= box.y && zintersect.x() <= box.x && zintersect.y() >= -box.y && zintersect.x() >= -box.x) {
		return zintersect;
	}
	return Vector3d();
}


void CollisionScene::traverseTree(SQueryPoints& qPoint, const Affine3d tLink, const bBox &linkBox){
	//tf::TransformBroadcaster br;
	//tf::Transform transform;

	//transform.setOrigin( tf::Vector3(qPoint.onLink.x(), qPoint.onLink.y(), qPoint.onLink.z()) );
	//transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

	//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "asdLink"));

	double dist = -1;

	// octoMapMutex.lock();
	if(octree == NULL){
		// octoMapMutex.unlock();
		return;
	}

	Vector3d linkPos = tLink.translation();

	for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end=octree->end_leafs(); it!= end; ++it)
 	{
 		if(it->getOccupancy() > 0.5){
 			//manipulate node, e.g.:
   			octomath::Vector3 cellPos = it.getCoordinate();
			Eigen::Vector3d cellPosEigen(cellPos.x(), cellPos.y(), cellPos.z());

   			Vector3d linkToCell = linkPos - cellPosEigen;

   			Vector3d pointOnCell = cellPosEigen - (linkToCell * (0.025/linkToCell.norm()));



   			Vector3d vecInLink = tLink.inverse().rotation() * linkToCell;
   			Vector3d pointOnLink = calcIntersection(vecInLink, linkBox);
   			pointOnLink = linkPos + (tLink.rotation() * pointOnLink);


   			//if(it->getOccupancy() > 0.2){
   				//transform.setOrigin( tf::Vector3(point.x(), point.y(), point.z()) );
				//transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

				//std::cout << "Node value: " << it->getValue() << std::endl;
				//std::cout << "Node size: " << it.getSize() << std::endl;
				//std::cout << "Node asd: " << it->getOccupancy() << std::endl;

				//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "currentPoint"));
   			//}
   			double newdist = (pointOnCell - pointOnLink).norm();
   		
   			if(newdist < dist || dist < 0){
   			
   				qPoint.inScene = pointOnCell;
   				qPoint.onLink = pointOnLink;
   				dist = newdist;
   			}
 		}
	 }
	 // octoMapMutex.unlock();
	 std::cout << "distance: " << dist << std::endl;
	 
}



void CollisionScene::updateQuery() {
	// TODO: MAGIC
	ros::Time t1 = ros::Time::now();

	if (links.size() == 0 || refFrame.size() == 0)
		return;

	tf::StampedTransform temp;
	tfListener.waitForTransform(refFrame, "base_link", ros::Time(0), ros::Duration(0.5));
	tfListener.lookupTransform(refFrame, "base_link", ros::Time(0), temp);

	Affine3d tPoint = Affine3d::Identity();
	tf::transformTFToEigen (temp, tPoint);


	for (const string& linkName: links) {
			try {
				tfListener.waitForTransform(refFrame, linkName, ros::Time(0), ros::Duration(0.5));
				tfListener.lookupTransform(refFrame, linkName, ros::Time(0), temp);

				Affine3d tLink = Affine3d::Identity();
				tf::transformTFToEigen (temp, tLink);

				//Affine3d iLink = tLink.inverse();

				// Iterate over Octomap

				//octomap::OcTreeNode *node = octree->getRoot();

				SQueryPoints qPoint;

				bBox box;
				box.x = 0.1;
				box.y = 0.1;
				box.z = 0.1;

				traverseTree(qPoint, tLink, box);

				qPoint.inScene = tPoint * qPoint.inScene;

				map.set(linkName, qPoint);
				

			} catch(tf::TransformException ex) {
				cerr << ex.what() << endl;
				ROS_WARN("TF-Query for link '%s' in '%s' failed!", linkName.c_str(), refFrame.c_str());
			}
		//}
	}
	ros::Duration d1 = ros::Time::now() - t1;
	std::cout << "Duration: " << d1.toSec() << std::endl;
}