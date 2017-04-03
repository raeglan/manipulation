#include "CollisionScene.h"

CollisionScene::CollisionScene() {
	nh.setCallbackQueue(&cbQueue);
	updateTimer = nh.createTimer(ros::Duration(0.025), &CollisionScene::updateQuery, this);
}

void CollisionScene::init() {


	nh.spin(); // Thread should be started by this
}

void CollisionScene::setRobotDescription(const string& urdfStr) {

}

void CollisionScene::addQueryLink(const string& link) {

}

template <typename K, typename V>
MutexMap::MutexMap() 
: turn(0)
{
	interested = {0,0};
}

template <typename K, typename V>
void MutexMap::set(const K &key, V &value, int pid) {
	enter(pid);
	map[key] = value;
	leave(pid);
}

template <typename K, typename V>
bool MutexMap::get(const K &key, V &value, int pid) {
	enter(pid);
	if (map.find(key) != map.end()) {
		value = map[key];
		leave(pid);
		return true;
	}
	leave(pid);
	return false;
}

template <typename K, typename V>
void MutexMap::enter(int pid) {
	interested[pid] = true;
	turn = pid;
	while(turn == pid && interested[1 - pid]);
}

template <typename K, typename V>
void MutexMap::leave(int pid) {
	interested[pid] = false;
}