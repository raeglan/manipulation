#include "suturo_action_server/Octree.h"


namespace suturo_octree{


Octree::Octree(float size, int depth, Point3f center) : size(size), depth(depth), center(center) {
	init();
}

Octree::~Octree() {
	delete(rootNode);
}

void Octree::init() {
	int numNodes = 0;
	for (int i = 0; i <= depth; i++) {
		numNodes += pow(8, i);
	}

	rootNode = new Node[numNodes];

	for (int currentDepth = 0; currentDepth < depth; currentDepth++) {
		int idx = 0;

		for (int j = 0; j < currentDepth; j++) {
			idx += pow(8, j);
		}

		int depthSize = pow(8, currentDepth);

		int nextStart = idx + depthSize;


		float a = size / pow(2, currentDepth + 1);
		Point3f parentCenter;
		for (int i = 0; i < depthSize; i++) {
			parentCenter = rootNode[idx + i].getCenter();

			rootNode[nextStart + 8 * i + 0] = Node(parentCenter + Vector3f(a, a, a), false, depth - (currentDepth + 1));
			rootNode[idx + i][0] = &rootNode[nextStart + 8 * i + 0];
			
			rootNode[nextStart + 8 * i + 1] = Node(parentCenter + Vector3f(-a, a, a), false, depth - (currentDepth + 1));
			rootNode[idx + i][1] = &rootNode[nextStart + 8 * i + 1];

			rootNode[nextStart + 8 * i + 2] = Node(parentCenter + Vector3f(a, -a, a), false, depth - (currentDepth + 1));
			rootNode[idx + i][2] = &rootNode[nextStart + 8 * i + 2];

			rootNode[nextStart + 8 * i + 3] = Node(parentCenter + Vector3f(-a, -a, a), false, depth - (currentDepth + 1));
			rootNode[idx + i][3] = &rootNode[nextStart + 8 * i + 3];

			rootNode[nextStart + 8 * i + 4] = Node(parentCenter + Vector3f(a, a, -a), false, depth - (currentDepth + 1));
			rootNode[idx + i][4] = &rootNode[nextStart + 8 * i + 4];

			rootNode[nextStart + 8 * i + 5] = Node(parentCenter + Vector3f(-a, a, -a), false, depth - (currentDepth + 1));
			rootNode[idx + i][5] = &rootNode[nextStart + 8 * i + 5];

			rootNode[nextStart + 8 * i + 6] = Node(parentCenter + Vector3f(a, -a, -a), false, depth - (currentDepth + 1));
			rootNode[idx + i][6] = &rootNode[nextStart + 8 * i + 6];

			rootNode[nextStart + 8 * i + 7] = Node(parentCenter + Vector3f(-a, -a, -a), false, depth - (currentDepth + 1));
			rootNode[idx + i][7] = &rootNode[nextStart + 8 * i + 7];
		}
	}
}


void Octree::addPoint(Point3f point) {
	Node* n = rootNode;
	if (n->isLeaf()) {
		return;
	}

	n->isOccupied(true);

	do {
		int closestNode;
		float closestDistance = -1;
		for (int i = 0; i < 8; i++) {
			float dist = (n->operator[](i)->getCenter() - point).norm();
			if (dist < closestDistance || closestDistance < 0) {
				closestDistance = dist;
				closestNode = i;
			}
		}
		n = n->operator[](closestNode);
		n->isOccupied(true);
	} while (!n->isLeaf());
}

Node* Octree::getRoot(){
	return rootNode;
}

/*
void Octree::addPoint(float size, int depth, Point3f point, Node& n) {
	
	if (!n.isOccupied()){
		float a = size / 4;
		n[0] = &Node(n.getCenter() + Vector3f(a, a, a), false, depth - 1);
		n[1] = &Node(n.getCenter() + Vector3f(-a, a, a), false, depth - 1);
		n[2] = &Node(n.getCenter() + Vector3f(a, -a, a), false, depth - 1);
		n[3] = &Node(n.getCenter() + Vector3f(-a, -a, a), false, depth - 1);
		n[4] = &Node(n.getCenter() + Vector3f(a, a, -a), false, depth - 1);
		n[5] = &Node(n.getCenter() + Vector3f(-a, a, -a), false, depth - 1);
		n[6] = &Node(n.getCenter() + Vector3f(a, -a, -a), false, depth - 1);
		n[7] = &Node(n.getCenter() + Vector3f(-a, -a, -a), false, depth - 1);
	}


	int closestNode;
	float closestDistance = -1;
	for (int i = 0; i < 8; i++) {
		float dist = (n[i]->getCenter() - point).norm();
		if (dist < closestDistance || closestDistance < 0) {
			closestDistance = dist;
			closestNode = i;
		}
	}

	if (depth > 0) {
		addPoint(size / 2, depth - 1, point, *n[closestNode]);
	}
	n.isOccupied(true);
	n[closestNode]->isOccupied(true);
}*/


}
