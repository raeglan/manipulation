#include "suturo_action_server/Node.h"
#include <stdexcept>

namespace suturo_octree{

	/**
	 * @brief      Constructs the Node.
	 *
	 * @param[in]  center      The center of the Node
	 * @param[in]  isOccupied  Indicates if occupied
	 * @param[in]  isLeaf      Indicates if this Node is a leaf Node
	 */
	Node::Node(Point3f center, bool isOccupied, int isLeaf) : occupied(isOccupied), center(center), leaf(!isLeaf) {
	}

	/**
	 * @brief      Constructs the object.
	 */
	Node::Node() : leaf(false), occupied(false), center(Point3f()) {};


	bool Node::isOccupied() {
		return occupied;
	}


	/**
	 * @brief      Sets the occupancy of the Node.
	 *
	 * @param[in]  oc    { parameter_description }
	 */
	void Node::isOccupied(bool oc) {
		occupied = oc;
	}

	/**
	 * @brief      Returns the center of the Node.
	 *
	 * @return     The center.
	 */
	Point3f Node::getCenter() {
		return center;
	}

	/**
	 * @brief      Determines if leaf.
	 *
	 * @return     True if leaf, False otherwise.
	 */
	bool Node::isLeaf() {
		return leaf;
	}



	/**
	 * @brief      Returns a pointer the the idxs child of the Node.
	 *
	 * @param[in]  idx   The index
	 *
	 * @return     A pointer the the idxs child of the Node.
	 */
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