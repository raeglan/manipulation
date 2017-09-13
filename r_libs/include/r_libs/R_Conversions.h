#pragma once
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Transform.h>

#include <string>

/**
 * @brief      Converts an Eigen::Afine3d to a PoseStamped message
 *
 * @param[in]  poseEigen  Eigen transform
 * @param[in]  frame_id   Parent frame
 *
 * @return     PoseStamped matching the eigen transformation
 */
geometry_msgs::PoseStamped EigenToGeometrymsgsStamped(Eigen::Affine3d poseEigen, std::string frame_id);

/**
 * @brief      Converts an Eigen::Afine3d to a Pose message
 *
 * @param[in]  poseEigen  Eigen transform
 *
 * @return     Pose message matching the eigen transformation
 */
geometry_msgs::Pose EigenToGeometrymsgs(Eigen::Affine3d poseEigen);

/**
 * @brief      Converts an Eigen::Afine3d to a Transform message
 *
 * @param[in]  poseEigen  Eigen transform
 *
 * @return     Transformation message matching the eigen transformation
 */
geometry_msgs::Transform EigenToGeometrymsgsTF(Eigen::Affine3f poseEigen);

/**
 * @brief      Converts a Pose message to an Eigen::Afine3d 
 *
 * @param[in]  pose  Pose message
 *
 * @return     Eigen transformation matching the message
 */
Eigen::Affine3d GeometrymsgsToEigenD(geometry_msgs::Pose pose);

/**
 * @brief      Converts a Pose message to an Eigen::Matrix4f 
 *
 * @param[in]  pose  Transform message
 *
 * @return     Eigen matrix matching the message
 */
Eigen::Matrix4f GeometrymsgsTFToEigenM44F(geometry_msgs::Transform pose);

/**
 * @brief      Converts a Transform message to an Eigen::Afine3d 
 *
 * @param[in]  pose  Transform message
 *
 * @return     Eigen transformation matching the message
 */
Eigen::Affine3d GeometrymsgsTFToEigen3d(geometry_msgs::Transform pose);

/**
 * @brief      Converts a Point message to an Eigen::Vector3d 
 *
 * @param[in]  point  Point message
 *
 * @return     Eigen vector matching the message
 */
Eigen::Vector3d PointToVector(geometry_msgs::Point point);

/**
 * @brief      Converts an Eigen::Vector3d to a Point message
 *
 * @param[in]  vec  Point message
 *
 * @return     Point message matching eigen vector
 */
geometry_msgs::Point VectorToPoint(Eigen::Vector3d vec);

/**
 * @brief      Converts an Eigen::Affine3d to a tf::Transform
 *
 * @param[in]  eTransform  eigen transform
 *
 * @return     Tf transform matching the eigen transform
 */
tf::Transform EigenToTfTransform(Eigen::Affine3d eTransform);