#include "suturo_action_server/Node.h"
#include <stdexcept>

namespace suturo_octree{

Node::Node(Point3f center, bool isOccupied, int isLeaf) : occupied(isOccupied), center(center), leaf(!isLeaf) {
}

Node::Node() : leaf(false), occupied(false), center(Point3f()) {};


bool Node::isOccupied() {
	return occupied;
}


void Node::isOccupied(bool oc) {
	occupied = oc;
}

Point3f Node::getCenter() {
	return center;
}

bool Node::isLeaf() {
	return leaf;
}



Node*& Node::operator[](int idx) {
	switch (idx) {
	case 0:
		return child0;
		break;
	case 1:
		return child1;
		break;
	case 2:
		return child2;
		break;
	case 3:
		return child3;
		break;
	case 4:
		return child4;
		break;
	case 5:
		return child5;
		break;
	case 6:
		return child6;
		break;
	case 7:
		return child7;
		break;
	default:
		throw std::out_of_range("child does not exist");
		break;
	}
}

}