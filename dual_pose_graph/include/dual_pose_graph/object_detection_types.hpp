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
 *  \file       graph_node_types.hpp
 *  \brief      An state estimation server for AeroStack2
 *  \authors    David Pérez Saura
 *              Miguel Fernández Cortizas
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef AS2_SLAM__OBJECTS_TYPES_HPP_
#define AS2_SLAM__OBJECTS_TYPES_HPP_

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <string>
#include "dual_pose_graph/graph_node_types.hpp"
#include "dual_pose_graph/graph_edge_types.hpp"
#include "utils/conversions.hpp"
#include "utils/general_utils.hpp"
// #include "dual_pose_graph/optimizer_g2o.hpp"

class ObjectDetection
{
public:
  virtual std::string getId() = 0;
  virtual Eigen::MatrixXd getCovarianceMatrix() = 0;
  virtual Eigen::MatrixXd getInformationMatrix() = 0;
  virtual bool prepareMeasurements(const OdometryInfo & _detection_odometry) = 0;
  virtual GraphNode * createNode() = 0;
  virtual GraphEdge * createEdge(GraphNode * _node, GraphNode * _detection_node) = 0;
};

class ObjectDetectionBase : public ObjectDetection
{
public:
  ObjectDetectionBase(
    const std::string & _id, const Eigen::MatrixXd & _covariance,
    const bool _detections_are_absolute)
  : id_(_id), covariance_matrix_(_covariance), detections_are_absolute_(_detections_are_absolute)
  {
    information_matrix_ = covariance_matrix_.inverse();
  }
  ~ObjectDetectionBase() {}

  std::string getId() override {return id_;}
  Eigen::MatrixXd getCovarianceMatrix() override {return covariance_matrix_;}
  Eigen::MatrixXd getInformationMatrix() override {return information_matrix_;}

protected:
  std::string id_;
  Eigen::MatrixXd covariance_matrix_;
  Eigen::MatrixXd information_matrix_;
  bool detections_are_absolute_;
};

class ObjectDetectionSE3 : public ObjectDetectionBase
{
public:
  ObjectDetectionSE3(
    const std::string & _id, const Eigen::Isometry3d _pose,
    const Eigen::MatrixXd & _covariance, const bool _detections_are_absolute)
  : ObjectDetectionBase(_id, _covariance, _detections_are_absolute), measured_pose_(_pose) {}

  bool prepareMeasurements(const OdometryInfo & _detection_odometry) override
  {
    // TODO(dps): Check if we should use map_ref or odom_ref
    if (detections_are_absolute_) {
      edge_measurement_ = _detection_odometry.map_ref.inverse() * measured_pose_;
      node_estimation_ = measured_pose_;

    } else {
      edge_measurement_ = measured_pose_;
      node_estimation_ = _detection_odometry.map_ref * measured_pose_;
    }
    // INFO(PRINT_VAR(_detection_odometry.odom_ref.translation().transpose()));
    // INFO(PRINT_VAR(_detection_odometry.map_ref.translation().transpose()));
    // INFO(PRINT_VAR(edge_measurement_.translation().transpose()));
    // INFO(PRINT_VAR(node_estimation_.translation().transpose()));
    return true;
  }

protected:
  Eigen::Isometry3d measured_pose_;
  Eigen::Isometry3d node_estimation_;
  Eigen::Isometry3d edge_measurement_;
};

class ObjectDetectionPoint3D : public ObjectDetectionBase
{
public:
  ObjectDetectionPoint3D(
    const std::string & _id, const Eigen::Vector3d _position,
    const Eigen::MatrixXd & _covariance, const bool _detections_are_absolute)
  : ObjectDetectionBase(_id, _covariance, _detections_are_absolute), measured_position_(_position)
  {
  }

  bool prepareMeasurements(const OdometryInfo & _detection_odometry) override
  {
    if (detections_are_absolute_) {
      edge_measurement_ = _detection_odometry.odom_ref.inverse() * measured_position_;
      node_estimation_ = measured_position_;

    } else {
      edge_measurement_ = measured_position_;
      node_estimation_ = _detection_odometry.map_ref * measured_position_;
    }
    // INFO(PRINT_VAR(_detection_odometry.odom_ref.translation().transpose()));
    // INFO(PRINT_VAR(_detection_odometry.map_ref.translation().transpose()));
    // INFO(PRINT_VAR(edge_measurement_.transpose()));
    // INFO(PRINT_VAR(node_estimation_.transpose()));
    return true;
  }

protected:
  Eigen::Vector3d measured_position_;
  Eigen::Vector3d node_estimation_;
  Eigen::Vector3d edge_measurement_;
};

class ArucoDetection : public ObjectDetectionSE3
{
public:
  ArucoDetection(
    const std::string & _id, const Eigen::Isometry3d _pose,
    const Eigen::MatrixXd & _covariance, const bool _detections_are_absolute)
  : ObjectDetectionSE3(_id, _pose, _covariance, _detections_are_absolute) {}

  GraphNode * createNode() override
  {
    ArucoNode * node = new ArucoNode(node_estimation_);
    node->setCovariance(covariance_matrix_);
    return node;
  }

  GraphEdge * createEdge(GraphNode * _node, GraphNode * _detection_node) override
  {
    GraphNodeSE3 * node_se3 = dynamic_cast<GraphNodeSE3 *>(_node);
    GraphNodeSE3 * detection_node_se3 = dynamic_cast<GraphNodeSE3 *>(_detection_node);
    // if (!node_se3) {DEBUG_GRAPH("Reference node is null");}
    // if (!detection_node_se3) {DEBUG_GRAPH("Detection node is null");}
    // FIXME(Miguel): Check dynamic cast
    return new ArucoEdge(
      node_se3, detection_node_se3, edge_measurement_,
      information_matrix_);
  }
};

class GateDetection : public ObjectDetectionPoint3D
{
public:
  GateDetection(
    const std::string & _id, const Eigen::Vector3d _position,
    const Eigen::MatrixXd & _covariance, const bool _detections_are_absolute)
  : ObjectDetectionPoint3D(_id, _position, _covariance, _detections_are_absolute) {}

  GraphNode * createNode() override
  {
    GateNode * node = new GateNode(node_estimation_);
    node->setCovariance(covariance_matrix_);
    return node;
  }

  GraphEdge * createEdge(GraphNode * _node, GraphNode * _detection_node) override
  {
    GraphNodeSE3 * node_se3 = dynamic_cast<GraphNodeSE3 *>(_node);
    GraphNodePoint3D * detection_node_point3d = dynamic_cast<GraphNodePoint3D *>(_detection_node);
    if (!node_se3) {DEBUG("Reference node is null");}
    if (!detection_node_point3d) {DEBUG("Detection node is null");}
    // FIXME(Miguel): Check dynamic cast
    return new GateEdge(
      node_se3, detection_node_point3d, edge_measurement_,
      information_matrix_);
  }
};

#endif  // AS2_SLAM__OBJECTS_TYPES_HPP_
