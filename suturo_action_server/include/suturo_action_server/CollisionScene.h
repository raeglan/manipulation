#include <ros/ros.h>
#include <ros/callback_queue.h>


#include <unordered_map>

template<typename K, typename V>
class MutexMap {
public:
	MutexMap();

	void set(const K &key, V &value);
	bool get(const K &key, V &value);
	void clear();

private:
	// 1 Mutex
	unordered_map<K,V> map;
}

// Aboniert PointClouds
// Immer wenn neue Cloud reinkommt:
//      In Oct-Map integrieren
//      Punkte ermitteln
//      Ergebnisse in Map schreiben
class CollisionScene {
public:
	struct SQueryPoints {	
		Vector3d onLink;
		Vector3d inScene;
	};

	typedef MutexMap<string, CollisionScene::SQueryPoints> QueryMap;

	CollisionScene(QueryMap &_map);

	void init();
	void setRobotDescription(const string& urdfStr);
	void addQueryLink(const string& link);
	void clearQueryLinks();

private:
	void updateQuery();

	ros::NodeHandle nh;
	ros::CallbackQueue cbQueue;
	ros::Timer updateTimer;

	urdf::Model robot;
	QueryMap& map;
	set<string> links;

};