#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <eigen3/Eigen/Eigen>
#include <urdf/model.h>

#include <unordered_map>

#include <tf/transform_listener.h>

#include <mutex>

#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>
#include <condition_variable>
//#include "suturo_action_server/Octree.h"

using namespace std;

#define READER_PID 0
#define WRITER_PID 1

namespace suturo_octree{
	class Octree;
}


template<typename K, typename V>
class MutexMap {
public:
	MutexMap() {}

	void set(const K &key, V &value) {
		while(!mtex.try_lock());
		map[key] = value;
		mtex.unlock();
	}

	bool get(const K &key, V &value) {
		while(!mtex.try_lock());
		if (map.find(key) != map.end()) {
			value = map[key];
			mtex.unlock();
			return true;
		}
		mtex.unlock();
		return false;
	}

	
	void clear() {
		while(!mtex.try_lock());
		map.clear();
		mtex.unlock();
	}

private:
	// 1 Mutex
	mutex mtex;
	unordered_map<K,V> map;
};

// Aboniert PointClouds
// Immer wenn neue Cloud reinkommt:
//      In Oct-Map integrieren
//      Punkte ermitteln
//      Ergebnisse in Map schreiben
class CollisionScene {
public:
	struct SQueryPoints {	
		Eigen::Vector3d onLink;
		Eigen::Vector3d inScene;
	};

	typedef MutexMap<string, CollisionScene::SQueryPoints> QueryMap;

	CollisionScene(QueryMap &_map);
	~CollisionScene();

	void setRobotDescription(const string& urdfStr);
	void addQueryLink(const string& link);
	void clearQueryLinks();
	void updateQuery();
	void updateBboxes();
	void setRefFrame(const string& pRefFrame);

private:
	struct SRobotLink {
		Eigen::Vector3d posBound;
		Eigen::Vector3d negBound;
	};

	struct bBox{
		bBox(float x, float y, float z):x(x), y(y), z(z){};
		bBox(): x(0), y(0), z(0){};
		float x;
		float y;
		float z;
	};

	void traverseTree(SQueryPoints& qPoint, const Eigen::Affine3d tLink, const bBox &linkBox);

	Eigen::Vector3d calcIntersection(const Eigen::Vector3d &v, const struct bBox &box);

	void updatePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input);


	void updateOctreeVisualization();
	void updateOctreeThread();
	void swapPointClouds();

	ros::NodeHandle nh;
	ros::CallbackQueue cbQueue;
	ros::Timer updateTimer;
	/**
	 * The depth of the octree. A depth greater than 7 is not recommended due to memory consumption.
	 */
	int octreeDepth = 6;
	/**
	 * The size of the octree in m.
	 */
	int octreeSize = 2;
	bool swap = false;
	bool stopUpdateThread;
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr activePointCloudPointer;
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloudPointer;
	/**
	 * The octree update thread.
	 */
	std::thread t;
	ros::Publisher octreeVisPublisher;
	ros::Subscriber pointCloudSubscriber;
	urdf::Model robot;
	/**
	 * This map contains the bounding boxes for each link.
	 */
	unordered_map<string, bBox> bboxMap;
	/**
	 * The mutex map which maps SQueryPoints to the query links.
	 */
	QueryMap& map;
	/**
	 * The set of all the links with collision queries.
	 */
	set<string> links;
	tf::TransformListener tfListener;
	string controllerRefFrame;
	string octomapFrame = "head_mount_kinect_rgb_optical_frame";
	suturo_octree::Octree* octree = NULL;

	mutex pointCloudMutex;
	mutex octreeMutex;
	mutex cvMutex;
	condition_variable cv;
	bool newPointCloud = false;
};