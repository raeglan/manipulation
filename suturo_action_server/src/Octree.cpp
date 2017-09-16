#include "suturo_action_server/Octree.h"
#include  <stdlib.h>


namespace suturo_octree{


	Octree::Octree(float size, int depth, Point3f center) : size(size), depth(depth), center(center) {
		int numNodes = 0;
		for (int i = 0; i <= depth; i++) {
			numNodes += pow(8, i);
		}

		rootNode = (Node*) calloc(numNodes, sizeof(Node));
	}

	Octree::~Octree() {
		free(rootNode);
	}


	Node* Octree::getRoot(){
		return rootNode;
	}

	int Octree::getDepth(){
		return depth;
	}

	float Octree::getSize(){
		return size;
	}

	void Octree::addPoint(Point3f point) {
		if(abs(point.position.x) > size/2 || abs(point.position.y) > size/2 || abs(point.position.z) > size/2)
			return;
		
		int parentId = 0;
		int start = 0;
		float newSize;
		for (int currentDepth = 0; currentDepth < depth; currentDepth++) {

			newSize = size / pow(2, currentDepth + 1);
			int depthSize = pow(8, currentDepth);
			int nextStart = start + depthSize;

			Point3f parentCenter = rootNode[start + parentId].getCenter();
			if (!rootNode[start + parentId].isOccupied()) {
				rootNode[nextStart + 8 * parentId + 0] = Node(parentCenter + Vector3f(newSize, newSize, newSize), false, depth - (currentDepth + 1));
				rootNode[start + parentId][0] = &rootNode[nextStart + 8 * parentId + 0];

				rootNode[nextStart + 8 * parentId + 1] = Node(parentCenter + Vector3f(-newSize, newSize, newSize), false, depth - (currentDepth + 1));
				rootNode[start + parentId][1] = &rootNode[nextStart + 8 * parentId + 1];

				rootNode[nextStart + 8 * parentId + 2] = Node(parentCenter + Vector3f(newSize, -newSize, newSize), false, depth - (currentDepth + 1));
				rootNode[start + parentId][2] = &rootNode[nextStart + 8 * parentId + 2];

				rootNode[nextStart + 8 * parentId + 3] = Node(parentCenter + Vector3f(-newSize, -newSize, newSize), false, depth - (currentDepth + 1));
				rootNode[start + parentId][3] = &rootNode[nextStart + 8 * parentId + 3];

				rootNode[nextStart + 8 * parentId + 4] = Node(parentCenter + Vector3f(newSize, newSize, -newSize), false, depth - (currentDepth + 1));
				rootNode[start + parentId][4] = &rootNode[nextStart + 8 * parentId + 4];

				rootNode[nextStart + 8 * parentId + 5] = Node(parentCenter + Vector3f(-newSize, newSize, -newSize), false, depth - (currentDepth + 1));
				rootNode[start + parentId][5] = &rootNode[nextStart + 8 * parentId + 5];

				rootNode[nextStart + 8 * parentId + 6] = Node(parentCenter + Vector3f(newSize, -newSize, -newSize), false, depth - (currentDepth + 1));
				rootNode[start + parentId][6] = &rootNode[nextStart + 8 * parentId + 6];

				rootNode[nextStart + 8 * parentId + 7] = Node(parentCenter + Vector3f(-newSize, -newSize, -newSize), false, depth - (currentDepth + 1));
				rootNode[start + parentId][7] = &rootNode[nextStart + 8 * parentId + 7];
			}

			rootNode[start + parentId].isOccupied(true);

			int closestChildId;
			float closestDistance = -1;
			for (int i = 0; i < 8; i++) {
				float dist = (rootNode[start + parentId].operator[](i)->getCenter() - point).norm();
				if (dist < closestDistance || closestDistance < 0) {
					closestDistance = dist;
					closestChildId = i;
				}
			}

			start += pow(8, currentDepth);
			parentId = parentId * 8 + closestChildId;
		}

		rootNode[start + parentId].isOccupied(true);
	}

}
