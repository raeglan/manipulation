#include "CollisionScene.h"

CollisionScene::CollisionScene() {
	ros::NodeHandle nh;

	nh.setCallbackQueue(&cbQueue);
	nh.createTimer(ros::Duration(0.025), &CollisionScene::)
}