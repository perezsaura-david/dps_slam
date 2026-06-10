// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


/********************************************************************************************
 *  \file       conversions.hpp
 *  \brief      An state estimation server for AeroStack2
 *  \authors    David Pérez Saura
 *              Miguel Fernández Cortizas
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef UTILS__CONVERSIONS_HPP_
#define UTILS__CONVERSIONS_HPP_

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
// #include <array>
#include <string>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

struct PoseSE3
{
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

struct IsometryWithID
{
  std::string id;
  Eigen::Isometry3d isometry;
};

struct OdometryInfo
{
  // Eigen::Isometry3d measurement;      // Odometry measurement received.
  Eigen::Isometry3d increment;     // Increment from the last odometry pose.
  Eigen::Isometry3d odom_ref;      // Odometry pose referenced from the "odom" frame.
  Eigen::Isometry3d map_ref;       // Odometry pose referenced from the "map" frame.
  Eigen::MatrixXd covariance_matrix;  // Covariance matrix of the odometry measurement.
};

struct OdometryWithCovariance
{
  Eigen::Isometry3d odometry;
  Eigen::MatrixXd covariance;
};

struct FixedObject
{
  std::string id;
  std::string type;
  Eigen::Isometry3d isometry;
};

PoseSE3 convertToPoseSE3(
  const Eigen::Vector3d & _position,
  const Eigen::Quaterniond & _orientation);
PoseSE3 convertToPoseSE3(const geometry_msgs::msg::Pose & _pose);
PoseSE3 convertToPoseSE3(Eigen::Isometry3d _isometry);

Eigen::Isometry3d convertToIsometry3d(
  const Eigen::Vector3d & _position,
  const Eigen::Quaterniond & _orientation);
Eigen::Isometry3d convertToIsometry3d(const geometry_msgs::msg::Pose & _pose);
Eigen::Isometry3d convertToIsometry3d(geometry_msgs::msg::Transform & _transform);

geometry_msgs::msg::Pose convertToGeometryMsgPose(const Eigen::Isometry3d & _isometry);
geometry_msgs::msg::Pose convertToGeometryMsgPose(const Eigen::Vector3d & _vector3d);
geometry_msgs::msg::Point convertToGeometryMsgPoint(const Eigen::Vector3d & _vector3d);
geometry_msgs::msg::TransformStamped convertToTransformStamped(
  const Eigen::Isometry3d & _transform,
  const std::string & _parent_frame,
  const std::string & _child_frame,
  const rclcpp::Time & _stamp);

#endif  // UTILS__CONVERSIONS_HPP_
