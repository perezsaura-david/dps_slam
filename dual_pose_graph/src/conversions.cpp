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

/**
 * @file debug_utils.cpp
 *
 * Conversions implementation for semantic slam
 *
 * @author David Pérez Saura
 *         Miguel Fernández Cortizas
 */

#include "utils/conversions.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <string>
#include <geometry_msgs/msg/detail/pose__struct.hpp>

PoseSE3 convertToPoseSE3(const Eigen::Vector3d & _position, const Eigen::Quaterniond & _orientation)
{
  PoseSE3 pose;
  pose.position = _position;
  pose.orientation = _orientation;
  return pose;
}

PoseSE3 convertToPoseSE3(const geometry_msgs::msg::Pose & _pose)
{
  PoseSE3 pose;
  pose.position = Eigen::Vector3d(_pose.position.x, _pose.position.y, _pose.position.z);
  pose.orientation = Eigen::Quaterniond(
    _pose.orientation.w, _pose.orientation.x,
    _pose.orientation.y, _pose.orientation.z);
  return pose;
}

PoseSE3 convertToPoseSE3(Eigen::Isometry3d _isometry)
{
  PoseSE3 pose;
  pose.position = _isometry.translation();
  pose.orientation = Eigen::Quaterniond(_isometry.rotation());
  return pose;
}

Eigen::Isometry3d convertToIsometry3d(
  const Eigen::Vector3d & _position,
  const Eigen::Quaterniond & _orientation)
{
  Eigen::Isometry3d isometry = Eigen::Translation3d(_position) * _orientation;
  return isometry;
}

Eigen::Isometry3d convertToIsometry3d(const geometry_msgs::msg::Pose & _pose)
{
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translation() = Eigen::Vector3d(_pose.position.x, _pose.position.y, _pose.position.z);
  Eigen::Quaterniond quaternion(_pose.orientation.w, _pose.orientation.x, _pose.orientation.y,
    _pose.orientation.z);
  isometry.rotate(quaternion);
  return isometry;
}

geometry_msgs::msg::Pose convertToGeometryMsgPose(const Eigen::Isometry3d & _isometry)
{
  geometry_msgs::msg::Pose geometry_msg_pose;
  PoseSE3 pose = convertToPoseSE3(_isometry);
  geometry_msg_pose.position.x = pose.position.x();
  geometry_msg_pose.position.y = pose.position.y();
  geometry_msg_pose.position.z = pose.position.z();
  geometry_msg_pose.orientation.w = pose.orientation.w();
  geometry_msg_pose.orientation.x = pose.orientation.x();
  geometry_msg_pose.orientation.y = pose.orientation.y();
  geometry_msg_pose.orientation.z = pose.orientation.z();
  return geometry_msg_pose;
}

Eigen::Isometry3d convertToIsometry3d(geometry_msgs::msg::Transform & _transform) {
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translation() = Eigen::Vector3d(
       _transform.translation.x,
       _transform.translation.y,
       _transform.translation.z);

  Eigen::Quaterniond quaternion(
    _transform.rotation.w,
    _transform.rotation.x,
    _transform.rotation.y,
    _transform.rotation.z);

  isometry.rotate(quaternion);
  return isometry;
}


geometry_msgs::msg::Pose convertToGeometryMsgPose(const Eigen::Vector3d & _vector3d)
{
  geometry_msgs::msg::Pose geometry_msg_pose;
  geometry_msg_pose.position.x = _vector3d.x();
  geometry_msg_pose.position.y = _vector3d.y();
  geometry_msg_pose.position.z = _vector3d.z();
  geometry_msg_pose.orientation.w = 1.0;
  geometry_msg_pose.orientation.x = 0.0;
  geometry_msg_pose.orientation.y = 0.0;
  geometry_msg_pose.orientation.z = 0.0;
  return geometry_msg_pose;
}

geometry_msgs::msg::Point convertToGeometryMsgPoint(const Eigen::Vector3d & _vector3d)
{
  geometry_msgs::msg::Point geometry_msg_pose;
  // PoseSE3 pose = convertToPoseSE3(_vector3d);
  geometry_msg_pose.x = _vector3d.x();
  geometry_msg_pose.y = _vector3d.y();
  geometry_msg_pose.z = _vector3d.z();
  return geometry_msg_pose;
}

geometry_msgs::msg::TransformStamped convertToTransformStamped(
  const Eigen::Isometry3d & _transform,
  const std::string & _parent_frame,
  const std::string & _child_frame,
  const rclcpp::Time & _stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = _stamp;
  transform_msg.header.frame_id = _parent_frame;
  transform_msg.child_frame_id = _child_frame;
  transform_msg.transform.translation.x = _transform.translation().x();
  transform_msg.transform.translation.y = _transform.translation().y();
  transform_msg.transform.translation.z = _transform.translation().z();
  Eigen::Quaterniond q(_transform.rotation());
  transform_msg.transform.rotation.x = q.x();
  transform_msg.transform.rotation.y = q.y();
  transform_msg.transform.rotation.z = q.z();
  transform_msg.transform.rotation.w = q.w();

  return transform_msg;
}
