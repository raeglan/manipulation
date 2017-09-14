#pragma once
#include "suturo_action_server/Vector3f.h"
#include "suturo_action_server/Node.h"

namespace suturo_octree{


class Octree {
public:
	Octree(float size, int depth, Point3f center);
	~Octree();
	void addPoint(Point3f point);
	suturo_octree::Node* getRoot();
	int getDepth();

private:
	void init();

	suturo_octree::Node* rootNode;
	float size;
	Point3f center;
	int depth;
};

}