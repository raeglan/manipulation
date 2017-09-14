#include <suturo_action_server/CollisionScene.h>
//#include <occupancy_map_monitor.h>
//#include <moveit/depth_image_octomap_updater/depth_image_octomap_updater.h>
//#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>

#include <tf_conversions/tf_eigen.h>
#include <octomap_msgs/conversions.h>
#include <urdf/model.h>
#include <link.h>

#include <tf/transform_broadcaster.h>

#include <iostream>
#include "suturo_action_server/Octree.h"



using namespace Eigen;
using namespace suturo_octree;

/**
 * @brief      Constructs the CollisionScene.
 *
 * @param      _map  The mutex map which maps SQueryPoints to the query links
 */
CollisionScene::CollisionScene(QueryMap &_map) 
 : map(_map), controllerRefFrame("base_link")		
{
	//nh.setCallbackQueue(&cbQueue);
	//updateTimer = nh.createTimer(ros::Duration(0.025), &CollisionScene::updateQuery, this);
	//sub = nh.subscribe("/octomap_binary", 10, &CollisionScene::update, this);
	sub2 = nh.subscribe("/kinect_head/depth_registered/points", 1, &CollisionScene::updatePointCloud, this);
	octreeVisPub = nh.advertise<visualization_msgs::MarkerArray>("octreeVis", 1);
	t = std::thread(&CollisionScene::updateOctreeThread, this);
	

	//setRobotDescription("/opt/ros/indigo/share/robot_state_publisher/test/pr2.urdf");
	//ros::spin();
}

void CollisionScene::updateOctreeVisualization(){
	int start = 0;
	int end = 0;

	for (int i = 0; i < octree2->getDepth(); i++) {
			start += pow(8, i);
	}

	end = start + pow(8, octree2->getDepth());

		//1
		//8
		//73
		//585
		//4681
		//37449
		//299593

		visualization_msgs::Marker marker;
		visualization_msgs::MarkerArray marker_array_msg;

		for ( int i = start; i < end; i++){
			if(octree2->getRoot()[i].isOccupied()){
				marker.header.frame_id = "head_mount_kinect_rgb_optical_frame";
   				marker.header.stamp = ros::Time();
   				marker.ns = "my_namespace";
   				marker.id = i;
   		 		marker.type = visualization_msgs::Marker::CUBE;
   				marker.action = visualization_msgs::Marker::ADD;
   				marker.pose.position.x = octree2->getRoot()[i].getCenter().position.x;
   				marker.pose.position.y = octree2->getRoot()[i].getCenter().position.y;
    			marker.pose.position.z = octree2->getRoot()[i].getCenter().position.z;
   				marker.pose.orientation.x = 0.0;
    			marker.pose.orientation.y = 0.0;
    			marker.pose.orientation.w = 1.0;
    			marker.pose.orientation.z = 0.0;
   				marker.scale.x = 0.12;
    			marker.scale.y = 0.12;
    			marker.scale.z = 0.12;
    			marker.color.a = 1.0;
    			marker.color.g = 1.0;
    			marker_array_msg.markers.push_back(marker);
			}
    

		octreeVisPub.publish(marker_array_msg);
		}
	
}

void CollisionScene::swapPointClouds(){
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr temp;
	temp = activePointCloudPointer;
	activePointCloudPointer = pointCloudPointer;
	pointCloudPointer = temp;
}

void CollisionScene::updateOctreeThread(){
	while(!pointCloudPointer);

	while(true){
		
		pointCloudMutex.lock();
		if(swap){
			swapPointClouds();
			swap = false;
		}
		pointCloudMutex.unlock();

		suturo_octree::Octree* octree = new suturo_octree::Octree(2, 5, Point3f());
		octomapFrame = "head_mount_kinect_rgb_optical_frame";
		for(auto it = activePointCloudPointer->points.begin(); it != activePointCloudPointer->points.end(); it++){
			octree->addPoint(Point3f(it->x, it->y, it->z));
		}
		delete(octree2);
		octree2 = octree;
		//updateOctreeVisualization();
	}
	
}


void CollisionScene::updatePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input){
	pointCloudMutex.lock();
	pointCloudPointer = input->makeShared();
	swap = true;
	pointCloudMutex.unlock();
}



/**
 * @brief      updates the internal octree of of the CollisionScene.
 *
 * @param[in]  omap  The octomap_msg
 */
void CollisionScene::update(const octomap_msgs::Octomap &omap) {
	//occupancy_map_monitor::OccupancyMapMonitor monitor();
	//occupancy_map_monitor::DepthImageOctomapUpdater updater();

	octomapFrame = omap.header.frame_id;

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


/**
 * @brief      updates the bounding boxes of every query link in the CollisionScene.
 */
void CollisionScene::updateBboxes(){
	bboxMap.clear();
	for (const string& linkName: links) {
		boost::shared_ptr< const urdf::Link > link = robot.getLink(linkName);
		boost::shared_ptr< urdf::Visual > visual = link->visual;
		boost::shared_ptr< urdf::Geometry > geometry = visual->geometry;

		switch (geometry->type){
			case urdf::Geometry::SPHERE:
				{
					double radius = (boost::static_pointer_cast<urdf::Sphere>(geometry))->radius;
					bBox bbox(radius, radius, radius);
					bboxMap[linkName] = bbox;
				}
				break;

			case urdf::Geometry::BOX:
				{
					boost::shared_ptr< urdf::Box > box = boost::static_pointer_cast<urdf::Box>(geometry);
					bBox bbox(box->dim.x, box->dim.y, box->dim.z);
					bboxMap[linkName] = bbox;
				}
				break;

			case urdf::Geometry::CYLINDER:
				{
					boost::shared_ptr< urdf::Cylinder > cylinder = boost::static_pointer_cast<urdf::Cylinder>(geometry);
					bBox bbox(cylinder->length, cylinder->radius, cylinder->radius);
					bboxMap[linkName] = bbox;
				}
				break;

			case urdf::Geometry::MESH:
				{
					double x  = (boost::static_pointer_cast<urdf::Mesh>(geometry))->scale.x;
					bBox bbox(0.12, 0.07, 0.07);
					bboxMap[linkName] = bbox;
				}
				break;

			default:
				break;

		}

	}

}


/**
 * @brief      Sets the robot description.
 *
 * @param[in]  urdfStr  The urdf string
 */
void CollisionScene::setRobotDescription(const string& urdfStr) {
	// TODO: GENERATE URDF
	if (!robot.initString(urdfStr)){
       ROS_ERROR("Failed to parse urdf file");
    }
}


/**
 * @brief      Sets the reference frame of the CollisionScene.
 *
 * @param[in]  pRefFrame  The reference frame
 */
void CollisionScene::setRefFrame(const string& pcontrollerRefFrame){
	controllerRefFrame = pcontrollerRefFrame;
}


/**
 * @brief      Adds a query link to the CollisionScene.
 *
 * @param[in]  link  The query link
 */
void CollisionScene::addQueryLink(const string& link) {
	links.insert(link);
}

void CollisionScene::clearQueryLinks() {
	links.clear();
}


/**
 * @brief      Calculates the intersection of a Vector from the center of the bounding box and the bounding box.
 *
 * @param[in]  v     The Vector from the center of the bounding box
 * @param[in]  box   The box
 *
 * @return     The intersection.
 */
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












/**
 * @brief      iterates over every leaf of the octree to find the closest voxel to the link.
 *
 * @param      qPoint   The SQueryPoints for the link
 * @param[in]  tLink    The transform for the link
 * @param[in]  linkBox  The bounding box for the link
 */
void CollisionScene::traverseTree(SQueryPoints& qPoint, const Affine3d tLinkInOct, const bBox &linkBox){

	if(octree2 == NULL){
		return;
	}

	Node* n = octree2->getRoot();
	Point3f linkPos(tLinkInOct.translation().x(), tLinkInOct.translation().y(), tLinkInOct.translation().z());

	while(!n->isLeaf()){
		int closestNode;
		float closestDistance = -1;
		for (int i = 0; i < 8; i++) {
			if(n->operator[](i)->isOccupied()){
				float dist = (n->operator[](i)->getCenter() - linkPos).norm();
				if (dist < closestDistance || closestDistance < 0) {
					closestDistance = dist;
					closestNode = i;
				}
			}
		}
		n = n->operator[](closestNode);
	}

	Point3f center = n->getCenter();

	qPoint.inScene = Vector3d(center.position.x, center.position.y, center.position.z);
  	qPoint.onLink = Vector3d(0,0,0);


}


	/*
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

	Vector3d linkPos = tLink.translation() - tLink.rotation() * Vector3d(linkBox.x, 0, 0); //- tLink.rotation() * Vector3d(linkBox.x, 0, 0);

	Vector3d linkToCell;
	Vector3d pointOnCell;
	Vector3d vecInLink;
	Vector3d pointOnLink_link;
	Vector3d pointOnLink_baselink;

	Vector3d linkToCell_best;
	Vector3d vecInLink_best;
	Vector3d pointOnLink_link_best;
	Vector3d pointOnLink_baselink_best;

	for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end=octree->end_leafs(); it!= end; ++it)
 	{
 		if(it->getOccupancy() > 0.5){
 			//manipulate node, e.g.:
   			octomath::Vector3 cellPos = it.getCoordinate();
			Eigen::Vector3d cellPosEigen(cellPos.x(), cellPos.y(), cellPos.z());

   			linkToCell = cellPosEigen - linkPos;

   			pointOnCell = cellPosEigen - (linkToCell * (0.025/linkToCell.norm()));



   			vecInLink = tLink.inverse().rotation() * linkToCell;
   			pointOnLink_link = calcIntersection(vecInLink, linkBox);
   			
			pointOnLink_baselink = tLink.rotation() * pointOnLink_link;
   			pointOnLink_baselink = linkPos + pointOnLink_baselink;


   			//if(it->getOccupancy() > 0.2){
   				//transform.setOrigin( tf::Vector3(point.x(), point.y(), point.z()) );
				//transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

				//std::cout << "Node value: " << it->getValue() << std::endl;
				//std::cout << "Node size: " << it.getSize() << std::endl;
				//std::cout << "Node asd: " << it->getOccupancy() << std::endl;

				//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "currentPoint"));
   			//}
   			double newdist = (cellPosEigen - pointOnLink_baselink).norm();
   		
   			if(newdist < dist || dist < 0){
   			
   				qPoint.inScene = cellPosEigen;
   				qPoint.onLink = pointOnLink_link;
   				dist = newdist;

   				linkToCell_best = linkToCell;
				vecInLink_best = vecInLink;
				pointOnLink_link_best = pointOnLink_link;
				pointOnLink_baselink_best = pointOnLink_baselink;
   			}
 		}
	 }
	 cout << "linkToCell " << linkToCell_best.x() << " " << linkToCell_best.y() << " " << linkToCell_best.z() << endl;
	 cout << "vecInLink " << vecInLink_best.x() << " " << vecInLink_best.y() << " " << vecInLink_best.z() << endl;
	 cout << "pointOnLink_link " << pointOnLink_link_best.x() << " " << pointOnLink_link_best.y() << " " << pointOnLink_link_best.z() << endl;
	 cout << "pointOnLink_baselink " << pointOnLink_baselink_best.x() << " " << pointOnLink_baselink_best.y() << " " << pointOnLink_baselink_best.z() << endl;
	 // octoMapMutex.unlock();
	 std::cout << "distance: " << dist << std::endl;
	 
}*/



/**
 * @brief      updates the SQueryPoints for every link in the CollisionScene.
 */
void CollisionScene::updateQuery() {
	// TODO: MAGIC
	ros::Time t1 = ros::Time::now();

	if (links.size() == 0 || octomapFrame.size() == 0)
		return;

	tf::StampedTransform temp;
	tfListener.waitForTransform(controllerRefFrame, octomapFrame, ros::Time(0), ros::Duration(0.5));
	tfListener.lookupTransform(controllerRefFrame, octomapFrame, ros::Time(0), temp);

	Affine3d tPoint = Affine3d::Identity();
	tf::transformTFToEigen (temp, tPoint);


	for (const string& linkName: links) {
			try {
				tfListener.waitForTransform(octomapFrame, linkName, ros::Time(0), ros::Duration(0.5));
				tfListener.lookupTransform(octomapFrame, linkName, ros::Time(0), temp);

				Affine3d tLinkInOct = Affine3d::Identity();
				tf::transformTFToEigen (temp, tLinkInOct);

				//Affine3d iLink = tLink.inverse();

				// Iterate over Octomap

				//octomap::OcTreeNode *node = octree->getRoot();

				SQueryPoints qPoint;

				auto it = bboxMap.find(linkName);

				if(it == bboxMap.end()){
					updateBboxes();
					it = bboxMap.find(linkName);
					if(it == bboxMap.end()){
						return;
					}
				}

				traverseTree(qPoint, tLinkInOct, it->second);

				qPoint.inScene = tPoint * qPoint.inScene;//

				map.set(linkName, qPoint);
				
				cout << "linkname " << linkName << endl;

			} catch(tf::TransformException ex) {
				cerr << ex.what() << endl;
				ROS_WARN("TF-Query for link '%s' in '%s' failed!", linkName.c_str(), octomapFrame.c_str());
			}
		//}
	}
	ros::Duration d1 = ros::Time::now() - t1;
	std::cout << "Duration: " << d1.toSec() << std::endl;
}


















































/**
 * @brief      iterates over every leaf of the octree to find the closest voxel to the link.
 *
 * @param      qPoint   The SQueryPoints for the link
 * @param[in]  tLink    The transform for the link
 * @param[in]  linkBox  The bounding box for the link
 */
/*
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

	Vector3d linkPos = tLink.translation() - tLink.rotation() * Vector3d(linkBox.x, 0, 0); //- tLink.rotation() * Vector3d(linkBox.x, 0, 0);

	Vector3d linkToCell;
	Vector3d pointOnCell;
	Vector3d vecInLink;
	Vector3d pointOnLink_link;
	Vector3d pointOnLink_baselink;

	Vector3d linkToCell_best;
	Vector3d vecInLink_best;
	Vector3d pointOnLink_link_best;
	Vector3d pointOnLink_baselink_best;

	for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end=octree->end_leafs(); it!= end; ++it)
 	{
 		if(it->getOccupancy() > 0.5){
 			//manipulate node, e.g.:
   			octomath::Vector3 cellPos = it.getCoordinate();
			Eigen::Vector3d cellPosEigen(cellPos.x(), cellPos.y(), cellPos.z());

   			linkToCell = cellPosEigen - linkPos;

   			pointOnCell = cellPosEigen - (linkToCell * (0.025/linkToCell.norm()));



   			vecInLink = tLink.inverse().rotation() * linkToCell;
   			pointOnLink_link = calcIntersection(vecInLink, linkBox);
   			
			pointOnLink_baselink = tLink.rotation() * pointOnLink_link;
   			pointOnLink_baselink = linkPos + pointOnLink_baselink;


   			//if(it->getOccupancy() > 0.2){
   				//transform.setOrigin( tf::Vector3(point.x(), point.y(), point.z()) );
				//transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

				//std::cout << "Node value: " << it->getValue() << std::endl;
				//std::cout << "Node size: " << it.getSize() << std::endl;
				//std::cout << "Node asd: " << it->getOccupancy() << std::endl;

				//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "currentPoint"));
   			//}
   			double newdist = (cellPosEigen - pointOnLink_baselink).norm();
   		
   			if(newdist < dist || dist < 0){
   			
   				qPoint.inScene = cellPosEigen;
   				qPoint.onLink = pointOnLink_link;
   				dist = newdist;

   				linkToCell_best = linkToCell;
				vecInLink_best = vecInLink;
				pointOnLink_link_best = pointOnLink_link;
				pointOnLink_baselink_best = pointOnLink_baselink;
   			}
 		}
	 }
	 cout << "linkToCell " << linkToCell_best.x() << " " << linkToCell_best.y() << " " << linkToCell_best.z() << endl;
	 cout << "vecInLink " << vecInLink_best.x() << " " << vecInLink_best.y() << " " << vecInLink_best.z() << endl;
	 cout << "pointOnLink_link " << pointOnLink_link_best.x() << " " << pointOnLink_link_best.y() << " " << pointOnLink_link_best.z() << endl;
	 cout << "pointOnLink_baselink " << pointOnLink_baselink_best.x() << " " << pointOnLink_baselink_best.y() << " " << pointOnLink_baselink_best.z() << endl;
	 // octoMapMutex.unlock();
	 std::cout << "distance: " << dist << std::endl;
	 
}*/



/**
 * @brief      updates the SQueryPoints for every link in the CollisionScene.
 */
/*
void CollisionScene::updateQuery() {
	// TODO: MAGIC
	ros::Time t1 = ros::Time::now();

	if (links.size() == 0 || octomapFrame.size() == 0)
		return;

	tf::StampedTransform temp;
	tfListener.waitForTransform(controllerRefFrame, octomapFrame, ros::Time(0), ros::Duration(0.5));
	tfListener.lookupTransform(controllerRefFrame, octomapFrame, ros::Time(0), temp);

	Affine3d tPoint = Affine3d::Identity();
	tf::transformTFToEigen (temp, tPoint);


	for (const string& linkName: links) {
			try {
				tfListener.waitForTransform(octomapFrame, linkName, ros::Time(0), ros::Duration(0.5));
				tfListener.lookupTransform(octomapFrame, linkName, ros::Time(0), temp);

				Affine3d tLink = Affine3d::Identity();
				tf::transformTFToEigen (temp, tLink);

				//Affine3d iLink = tLink.inverse();

				// Iterate over Octomap

				//octomap::OcTreeNode *node = octree->getRoot();

				SQueryPoints qPoint;

				auto it = bboxMap.find(linkName);

				if(it == bboxMap.end()){
					updateBboxes();
					it = bboxMap.find(linkName);
					if(it == bboxMap.end()){
						return;
					}
				}

				traverseTree(qPoint, tLink, it->second);

				qPoint.inScene = tPoint * qPoint.inScene;//

				map.set(linkName, qPoint);
				
				cout << "linkname " << linkName << endl;

			} catch(tf::TransformException ex) {
				cerr << ex.what() << endl;
				ROS_WARN("TF-Query for link '%s' in '%s' failed!", linkName.c_str(), octomapFrame.c_str());
			}
		//}
	}
	ros::Duration d1 = ros::Time::now() - t1;
	std::cout << "Duration: " << d1.toSec() << std::endl;
}*/