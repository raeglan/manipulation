#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <eigen3/Eigen/Eigen>
#include <urdf/model.h>

#include <unordered_map>

#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <tf/transform_listener.h>

#include <mutex>

using namespace std;

#define READER_PID 0
#define WRITER_PID 1

template<typename K, typename V>
class MutexMap {
public:
	MutexMap() {}

	void set(const K &key, V &value) {
		mtex.lock();
		map[key] = value;
		mtex.unlock();
	}

	bool get(const K &key, V &value) {
		mtex.lock();
		if (map.find(key) != map.end()) {
			value = map[key];
			mtex.unlock();
			return true;
		}
		mtex.unlock();
		return false;
	}

	
	void clear(int pid) {
		mtex.lock();
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

	void update(const octomap_msgs::Octomap &omap);
	void setRobotDescription(const string& urdfStr);
	void addQueryLink(const string& link);
	void clearQueryLinks();
	void updateQuery();

private:
	struct SRobotLink {
		Eigen::Vector3d posBound;
		Eigen::Vector3d negBound;
	};

	void traverseTree(SQueryPoints& qPoint, Eigen::Vector3d linkPos);

	ros::NodeHandle nh;
	ros::CallbackQueue cbQueue;
	ros::Timer updateTimer;

	ros::Subscriber sub;
	urdf::Model robot;
	unordered_map<string, SRobotLink> linkMap;
	QueryMap& map;
	set<string> links;
	tf::TransformListener tfListener;
	Eigen::Affine3d transform;
	string refFrame;
	octomap::OcTree *octree = NULL;

	mutex octoMapMutex;
};