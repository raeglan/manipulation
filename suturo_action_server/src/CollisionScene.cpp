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
: map(_map), controllerRefFrame("base_link"), stopUpdateThread(false)	
{
	pointCloudSubscriber = nh.subscribe("/kinect_head/depth_registered/points", 1, &CollisionScene::updatePointCloud, this);
	octreeVisPublisher = nh.advertise<visualization_msgs::MarkerArray>("suturo/octreeVis", 1);
	t = std::thread(&CollisionScene::updateOctreeThread, this);
}


/**
 * @brief      Stops the update thread and deletes the octree.
 */
CollisionScene::~CollisionScene(){
	stopUpdateThread = true;
	t.join();
	delete(octree);
}


/**
 * @brief      Publishes the leaf nodes of the octree as marker.
 */
void CollisionScene::updateOctreeVisualization(){
	int start = 0;
	int end = 0;

	float makerSize = (octree->getSize() / pow(2, octree->getDepth() - 1))*0.9;

	for (int i = 0; i < octree->getDepth(); i++) {
		start += pow(8, i);
	}

	end = start + pow(8, octree->getDepth());
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
		if(octree->getRoot()[i].isOccupied()){
			marker.header.frame_id = "head_mount_kinect_rgb_optical_frame";
			marker.header.stamp = ros::Time();
			marker.ns = "my_namespace";
			marker.id = i;
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = octree->getRoot()[i].getCenter().position.x;
			marker.pose.position.y = octree->getRoot()[i].getCenter().position.y;
			marker.pose.position.z = octree->getRoot()[i].getCenter().position.z;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.pose.orientation.z = 0.0;
			marker.scale.x = makerSize;
			marker.scale.y = makerSize;
			marker.scale.z = makerSize;
			marker.color.a = 1.0;
			marker.color.g = 1.0;
			marker_array_msg.markers.push_back(marker);
		}
	}
	octreeVisPublisher.publish(marker_array_msg);
}


/**
 * @brief      Swaps the pointers of the active and inactive point cloud.
 */
void CollisionScene::swapPointClouds(){
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr temp;
	temp = activePointCloudPointer;
	activePointCloudPointer = pointCloudPointer;
	pointCloudPointer = temp;
}

/**
 * @brief      Updates the octree when a new point cloud arrives.
 */
void CollisionScene::updateOctreeThread(){
	while(!pointCloudPointer);

	while(!stopUpdateThread){

		unique_lock<mutex> lk(cvMutex);
		cv.wait(lk, [&]{return newPointCloud;});
		newPointCloud = false;

		pointCloudMutex.lock();
		swapPointClouds();
		pointCloudMutex.unlock();

		suturo_octree::Octree* newOctree = new suturo_octree::Octree(octreeSize, octreeDepth, Point3f());
		octomapFrame = "head_mount_kinect_rgb_optical_frame";
		for(auto it = activePointCloudPointer->points.begin(); it <= activePointCloudPointer->points.end(); it+=100){
			newOctree->addPoint(Point3f(it->x, it->y, it->z));
		}
		octreeMutex.lock();
		delete(octree);
		octree = newOctree;
		octreeMutex.unlock();
		updateOctreeVisualization();
	}
}

/**
 * @brief      Makes a copy of the point cloud to the point cloud pointer and notifies the octree update thread.
 *
 * @param[in]  input  The point cloud message
 */
void CollisionScene::updatePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input){
	pointCloudMutex.lock();
	pointCloudPointer = input->makeShared();
	pointCloudMutex.unlock();
	newPointCloud = true;
	cv.notify_one();
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

	if(octree == NULL){
		return;
	}

	Node* n = octree->getRoot();
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
			
				// Iterate over Octomap

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
	}
	ros::Duration d1 = ros::Time::now() - t1;
	std::cout << "Duration: " << d1.toSec() << std::endl;
}