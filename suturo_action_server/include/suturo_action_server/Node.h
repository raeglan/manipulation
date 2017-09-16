#pragma once
#include "suturo_action_server/Vector3f.h"


namespace suturo_octree{

	class Node {
	public:
		Node(Point3f center, bool isOccupied, int isLeaf);
		Node();
		bool isOccupied();
		void isOccupied(bool oc);
		bool isLeaf();
		Node*& operator[](int idx);
		Point3f getCenter();

	private:
		Node* child0;
		Node* child1;
		Node* child2;
		Node* child3;
		Node* child4;
		Node* child5;
		Node* child6;
		Node* child7;

		bool leaf;
		bool occupied;
		Point3f center;
	};

}
