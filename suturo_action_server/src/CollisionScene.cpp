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

using namespace Eigen;

CollisionScene::CollisionScene(QueryMap &_map) 
 : map(_map)		
{
	//nh.setCallbackQueue(&cbQueue);
	//updateTimer = nh.createTimer(ros::Duration(0.025), &CollisionScene::updateQuery, this);
	sub = nh.subscribe("/octomap_binary", 10, &CollisionScene::update, this);
	//setRobotDescription("/opt/ros/indigo/share/robot_state_publisher/test/pr2.urdf");
	//ros::spin();
}

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

void CollisionScene::setRobotDescription(const string& urdfStr) {
	// TODO: GENERATE URDF
	if (!robot.initString(urdfStr)){
       ROS_ERROR("Failed to parse urdf file");
    }
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
	 
}



void CollisionScene::updateQuery() {
	// TODO: MAGIC
	ros::Time t1 = ros::Time::now();

	if (links.size() == 0 || octomapFrame.size() == 0)
		return;

	tf::StampedTransform temp;
	tfListener.waitForTransform(refFrame, octomapFrame, ros::Time(0), ros::Duration(0.5));
	tfListener.lookupTransform(refFrame, octomapFrame, ros::Time(0), temp);

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
}