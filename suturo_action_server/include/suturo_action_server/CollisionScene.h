#include <ros/ros.h>

#include <unordered_map>

template<typename K, typename V>
class MutexMap {
public:
	MutexMap();

	void set(const K &key, V &value);
	void get(const K &key, V &value);
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

	CollisionScene(ros::NodeHandle &nh);

	void setRobotDescription(const string& urdfStr);
	void setQueryLinks(const vector<string> &links);
	void setPointMap(MutexMap<string, SQueryPoints>* _map);

private:
	urdf::Model robot;
	MutexMap<string, SQueryPoints>* map;

};