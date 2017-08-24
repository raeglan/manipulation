#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <suturo_perception_msgs/ObjectDetection.h>
#include <iostream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "plate_publisher");

  ros::NodeHandle node;


  ros::Publisher plate_publisher = node.advertise<suturo_perception_msgs::ObjectDetection>("plate_publisher", 10);

  tf::TransformListener listener;




  ros::Rate rate(5.0);



  while (node.ok()){


    tf::StampedTransform transform;

    try{
      listener.waitForTransform("/base_link", "/plate", ros::Time(0), ros::Duration(10.0) );
      listener.lookupTransform("/base_link", "/plate", ros::Time(0), transform);


      suturo_perception_msgs::ObjectDetection od;

      geometry_msgs::PoseStamped poseStamped;

      tf::Vector3 b(0,0,0);
      tf::Quaternion q(0,0,0,1);

      tf::Pose platePoseTf(q, b);

      platePoseTf = transform*platePoseTf;

      geometry_msgs::Pose platePose;

      poseTFToMsg(platePoseTf, platePose);

      poseStamped.pose = platePose;
      poseStamped.header.stamp = ros::Time::now();

      od.name = "dinnerPlateForCake";
      od.pose = poseStamped;
      od.type = 7;


      /*float x = od.pose.pose.position.x;
      float y = od.pose.pose.position.y;
      float z = od.pose.pose.position.z;

      float qx = od.pose.pose.orientation.x;
      float qy = od.pose.pose.orientation.y;
      float qz = od.pose.pose.orientation.z;

    

      std::cout << "x " << qx << " y " << qy << " z " << qz << std::endl;*/


      plate_publisher.publish(od);


    } 
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  
    rate.sleep();
 }

  return 0;
}