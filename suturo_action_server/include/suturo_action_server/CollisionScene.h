#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <eigen3/Eigen/Eigen>
#include <urdf/model.h>

#include <unordered_map>

#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <tf/transform_listener.h>


using namespace std;

#define READER_PID 0
#define WRITER_PID 1

template<typename K, typename V>
class MutexMap {
public:
	MutexMap() 
	: turn(0)
	{
		interested[0] = 0;
		interested[1] = 0;
	}

	void set(const K &key, V &value, int pid) {
		enter(pid);
		map[key] = value;
		leave(pid);
	}

	bool get(const K &key, V &value, int pid) {
		enter(pid);
		if (map.find(key) != map.end()) {
			value = map[key];
			leave(pid);
			return true;
		}
		leave(pid);
		return false;
	}

	
	void clear(int pid) {
		enter(pid);
		map.clear();
		leave(pid);
	}

private:
	void enter(int pid) {
		interested[pid] = true;
		turn = pid;
		while(turn == pid && interested[1 - pid]);
	}

	void leave(int pid) {
		interested[pid] = false;
	}


	bool interested[2];
	int turn;

	// 1 Mutex
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

};