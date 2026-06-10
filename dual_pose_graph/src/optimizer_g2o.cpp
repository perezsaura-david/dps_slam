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
 * @file optimizer_g2o.cpp
 *
 * OptimizerG2O class implementation for AeroStack2
 *
 * @author David Pérez Saura
 *         Miguel Fernández Cortizas
 */

#include "dual_pose_graph/optimizer_g2o.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <string>
#include <iostream>
#include "dual_pose_graph/graph_g2o.hpp"
#include "dual_pose_graph/graph_node_types.hpp"
#include "dual_pose_graph/object_detection_types.hpp"
#include "utils/conversions.hpp"
#include "utils/general_utils.hpp"
#include "utils/debug_utils.hpp"
#include "utils/csv_logger.hpp"
#include <chrono>
#include <fstream>

OptimizerG2O::OptimizerG2O()
{
  FLAG("STARTING SEMANTIC SLAM");

  main_graph = std::make_shared<GraphG2O>("Main Graph");
  temp_graph = std::make_shared<GraphG2O>("Temp Graph");
  DEBUG(PRINT_VAR(main_graph));
  DEBUG(PRINT_VAR(temp_graph));

  if (odometry_is_relative_) {
    PARAM("Relative odometry");
  } else {
    PARAM("Absolute odometry");
  }

  main_graph->initGraph();
  map_odom_tranform_ = Eigen::Isometry3d::Identity();
  last_odometry_added_.odometry = Eigen::Isometry3d::Identity();
  last_odometry_added_.covariance = Eigen::MatrixXd::Zero(6, 6);
  init_main_graph_ = true;
}

bool OptimizerG2O::generateOdometryInfo(
  const OdometryWithCovariance & _new_odometry,
  const OdometryWithCovariance & _last_odometry_added,
  OdometryInfo & _odometry_info)
{
  if (_new_odometry.covariance.isZero()) {
    WARN("Received covariance matrix is zero");
    return false;
  }

  if (odometry_is_relative_) {
    // TODO(dps): RELATIVE ODOMETRY
    // relative_pose = odom_pose;
    ERROR("RELATIVE ODOMETRY NOT IMPLEMENTED");
    return false;
  } else {
    // ABSOLUTE ODOMETRY
    _odometry_info.odom_ref = _new_odometry.odometry;
    _odometry_info.increment = _last_odometry_added.odometry.inverse() * _odometry_info.odom_ref;
    if (calculate_odom_covariance_) {
      // WARN("Calculating odometry covariance");
      _odometry_info.covariance_matrix = Eigen::MatrixXd::Zero(6, 6);
      _odometry_info.covariance_matrix(0, 0) = fabs(_new_odometry.covariance(0, 0) - 
        _last_odometry_added.covariance(0, 0));
      _odometry_info.covariance_matrix(1, 1) = fabs(_new_odometry.covariance(1, 1) - 
        _last_odometry_added.covariance(1, 1));
      _odometry_info.covariance_matrix(2, 2) = fabs(_new_odometry.covariance(2, 2) -
        _last_odometry_added.covariance(2, 2));
      _odometry_info.covariance_matrix(3, 3) = _new_odometry.covariance(3, 3);
      _odometry_info.covariance_matrix(4, 4) = _new_odometry.covariance(4, 4);
      _odometry_info.covariance_matrix(5, 5) = _new_odometry.covariance(5, 5);
    } else {
      // WARN("Using odometry covariance");
      _odometry_info.covariance_matrix = _new_odometry.covariance;
    }
    // _odometry_info.covariance_matrix = _new_odometry.covariance - _last_odometry_added.covariance;
  }
  _odometry_info.map_ref = initial_earth_to_map_transform_.inverse() * _odometry_info.odom_ref;
  // _odometry_info.map_ref = earth_map_transform_ * _odometry_info.odom_ref;
  // _odometry_info.map_ref = _odometry_info.odom_ref;

  if (_odometry_info.covariance_matrix.isZero()) {
    WARN("Generated odometry covariance matrix is zero");
    return false;
  }

  return true;
}

bool OptimizerG2O::handleNewOdom(
  const OdometryWithCovariance & _new_odometry)
{
  OdometryInfo new_odometry_info;
  if (!generateOdometryInfo(
      _new_odometry, last_odometry_added_,
      new_odometry_info))
  {
    return false;
  }
  if (use_dual_graph_ && temp_graph_generated_) {
    if (!checkAddingConditions(new_odometry_info, main_graph_odometry_distance_threshold_if_detections_,
          main_graph_odometry_orientation_threshold_)) {
      return false;
    }
  } else if (!checkAddingConditions(new_odometry_info, main_graph_odometry_distance_threshold_,
               main_graph_odometry_orientation_threshold_)) {
    return false;
  }

  // if (!checkAddingConditions(new_odometry_info, main_graph_odometry_distance_threshold_)) {
  //   return false;
  // }
  last_odometry_added_.odometry = new_odometry_info.odom_ref;
  last_odometry_added_.covariance = _new_odometry.covariance;

  // DEBUG(new_odometry_info.covariance_matrix);

  // FLAG("ADDING NEW ODOMETRY TO MAIN GRAPH");
  if (std::isnan(new_odometry_info.odom_ref.translation().x())) {
    WARN("Odometry generated is NaN");
    return false;
  }

  main_graph->addNewKeyframe(
    new_odometry_info.map_ref, new_odometry_info.increment,
    new_odometry_info.covariance_matrix);

  if (!use_dual_graph_) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    detections_since_last_keyframe_.clear();
  }

  if (use_dual_graph_) {
  graph_mutex_.lock();
  if (temp_graph == nullptr) {
    ERROR("Temp graph is null");
    return false;
  }
  if (temp_graph_generated_ && temp_graph) {
    static std::ofstream merge_log("slam_merge_debug.csv");
    static bool merge_log_header = false;
    if (!merge_log_header) {
      merge_log << "temp_nodes,temp_edges,temp_objects,opt_success,"
                << "gate_id,gate_x,gate_y,gate_z,cov_size,cov_diag,"
                << "odom_map_x,odom_map_y,odom_map_z,"
                << "main_nodes,main_edges" << std::endl;
      merge_log_header = true;
    }

    int temp_nodes = temp_graph->graph_->vertices().size();
    int temp_edges = temp_graph->graph_->edges().size();
    int temp_objects = temp_graph->getObjectNodes().size();

    if (temp_graph->optimizeGraph()) {

      for (auto object : temp_graph->getObjectNodes()) {
        if (!object.second) {
          ERROR("Object node is null");
          continue;
        }
        try {
          ObjectDetection * object_detection = nullptr;

          ArucoNode * aruco_node = dynamic_cast<ArucoNode *>(object.second);
          if (aruco_node) {
            Eigen::MatrixXd cov_matrix = temp_graph->computeNodeCovariance(aruco_node);
            merge_log << temp_nodes << "," << temp_edges << "," << temp_objects << ",1,"
                      << object.first << ","
                      << aruco_node->getPose().translation().x() << ","
                      << aruco_node->getPose().translation().y() << ","
                      << aruco_node->getPose().translation().z() << ","
                      << cov_matrix.size() << ",";
            if (cov_matrix.size() > 0) {
              merge_log << cov_matrix.diagonal().transpose();
            } else {
              merge_log << "empty";
            }
            merge_log << "," << new_odometry_info.map_ref.translation().x()
                      << "," << new_odometry_info.map_ref.translation().y()
                      << "," << new_odometry_info.map_ref.translation().z()
                      << "," << main_graph->graph_->vertices().size()
                      << "," << main_graph->graph_->edges().size() << std::endl;
            if (cov_matrix.size() == 0) { continue; }
            object_detection = new ArucoDetection(
              object.first, aruco_node->getPose(), cov_matrix, true);
          }

          GateNode * gate_node = dynamic_cast<GateNode *>(object.second);
          if (gate_node) {
            Eigen::MatrixXd cov_matrix = temp_graph->computeNodeCovariance(gate_node);
            merge_log << temp_nodes << "," << temp_edges << "," << temp_objects << ",1,"
                      << object.first << ","
                      << gate_node->getPosition().x() << ","
                      << gate_node->getPosition().y() << ","
                      << gate_node->getPosition().z() << ","
                      << cov_matrix.size() << ",";
            if (cov_matrix.size() > 0) {
              merge_log << cov_matrix.diagonal().transpose();
            } else {
              merge_log << "empty";
            }
            merge_log << "," << new_odometry_info.map_ref.translation().x()
                      << "," << new_odometry_info.map_ref.translation().y()
                      << "," << new_odometry_info.map_ref.translation().z()
                      << "," << main_graph->graph_->vertices().size()
                      << "," << main_graph->graph_->edges().size() << std::endl;
            if (cov_matrix.size() == 0) { continue; }
            object_detection = new GateDetection(
              object.first, gate_node->getPosition(), cov_matrix, true);
          }

          if (!object_detection) { continue; }
          if (!object_detection->prepareMeasurements(new_odometry_info)) {
            ERROR("Prepare detection ERROR");
            continue;
          }
          main_graph->addNewObjectDetection(object_detection);

        } catch (const std::exception & e) {
           ERROR("Exception: " << e.what());
           continue;
        }
      }
      auto sharing = temp_graph.use_count();
      if (sharing > 1) {
        DEBUG("Temp graph Shared: " << sharing);
      }
    } else {
      merge_log << temp_nodes << "," << temp_edges << "," << temp_objects << ",0,"
                << ",,,,,,,,,"
                << main_graph->graph_->vertices().size()
                << "," << main_graph->graph_->edges().size() << std::endl;
      ERROR("Temp graph optimization failed");
    }
  }

  if (temp_graph_generated_ && temp_graph) {
    temp_graph.reset();
    temp_graph = std::make_shared<GraphG2O>("Temp Graph");
    temp_graph_generated_ = false;
  }
  graph_mutex_.unlock();
  } // use_dual_graph_

  double chi2_before = main_graph->graph_->chi2();
  auto opt_start = std::chrono::steady_clock::now();
  main_graph->optimizeGraph();
  auto opt_end = std::chrono::steady_clock::now();
  double chi2_after = main_graph->graph_->chi2();
  double opt_ms = std::chrono::duration<double, std::milli>(opt_end - opt_start).count();

  if (generate_odom_map_transform_) {
    updateOdomMapTransform();
  }

  int temp_nodes = 0, temp_edges = 0;
  if (use_dual_graph_ && temp_graph) {
    temp_nodes = static_cast<int>(temp_graph->graph_->vertices().size());
    temp_edges = static_cast<int>(temp_graph->graph_->edges().size());
  }

  if (csv_logger_) {
    auto optimized = getOptimizedPose();
    auto corrected = earth_map_transform_ * map_odom_tranform_ * last_odometry_added_.odometry;
    csv_logger_->logKeyframe(0, 0,
      last_odometry_added_.odometry,
      static_cast<int>(main_graph->graph_->vertices().size()),
      static_cast<int>(main_graph->graph_->edges().size()),
      chi2_before, chi2_after, opt_ms,
      map_odom_tranform_, optimized, corrected);
  }

  return true;
}

bool OptimizerG2O::checkAddingConditions(
  const OdometryInfo & _odometry, const double _distance_threshold,
  const double _orientation_threshold)
{
  double translation_distance = _odometry.increment.translation().norm();
  double rotation_distance = Eigen::AngleAxisd(_odometry.increment.rotation()).angle();

  if (translation_distance < _distance_threshold &&
      rotation_distance < _orientation_threshold) {
    if (init_main_graph_) {
      init_main_graph_ = false;
      return true;
    }
    return false;
  }
  return true;
}

bool OptimizerG2O::checkAddingNewDetection(
  const OdometryWithCovariance & _detection_odometry,
  OdometryInfo & _detection_odometry_info)
{

  // graph_mutex_.lock();
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (temp_graph == nullptr) {
    ERROR("Temp graph is null");
    return false;
  }
  if (!temp_graph_generated_) {
    last_detection_odometry_added_ = last_odometry_added_;
  }

  if (!generateOdometryInfo(
    _detection_odometry, last_detection_odometry_added_,
    _detection_odometry_info)) {
    return false;
  }

  if (!temp_graph_generated_) {
    temp_graph->initGraph(_detection_odometry_info.map_ref);
    temp_graph_generated_ = true;
    // main_graph_object_covariance = _object->getCovarianceMatrix();  // FIXME(dps): remove this
  } else {
    if (!checkAddingConditions(_detection_odometry_info, temp_graph_odometry_distance_threshold_,
          temp_graph_odometry_orientation_threshold_)) {
      return false;
    }
    temp_graph->addNewKeyframe(
      _detection_odometry_info.map_ref, _detection_odometry_info.increment,
      _detection_odometry_info.covariance_matrix);
  }

  // graph_mutex_.unlock();

  last_detection_odometry_added_.odometry = _detection_odometry_info.odom_ref;
  last_detection_odometry_added_.covariance = _detection_odometry.covariance;

  return true;
}

// TODO(dps): return bool?
void OptimizerG2O::handleNewObjectDetection(
  ObjectDetection * _object,
  const OdometryInfo & _detection_odometry_info)
{
  if (!_object->prepareMeasurements(_detection_odometry_info)) {
    ERROR("Prepare detection ERROR");
    return;
  }

  if (use_dual_graph_) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    if (!temp_graph_generated_) { return; }
    temp_graph->addNewObjectDetection(_object);
  } else {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    std::string id = _object->getId();
    if (detections_since_last_keyframe_.count(id) > 0) { return; }
    detections_since_last_keyframe_.insert(id);
    main_graph->addNewObjectDetection(_object);
  }
}

void OptimizerG2O::setParameters(const OptimizerG2OParameters & _params)
{
  main_graph_odometry_distance_threshold_ = _params.main_graph_odometry_distance_threshold;
  main_graph_odometry_orientation_threshold_ = _params.main_graph_odometry_orientation_threshold;
  temp_graph_odometry_distance_threshold_ = _params.temp_graph_odometry_distance_threshold;
  temp_graph_odometry_orientation_threshold_ = _params.temp_graph_odometry_orientation_threshold;
  main_graph_odometry_distance_threshold_if_detections_ = _params.main_graph_odometry_distance_threshold_if_detections;
  map_odom_security_threshold_ = _params.map_odom_security_threshold;
  odometry_is_relative_ = _params.odometry_is_relative;
  generate_odom_map_transform_ = _params.generate_odom_map_transform;
  fixed_objects_ = _params.fixed_objects;
  initial_earth_to_map_transform_ = _params.earth_to_map_transform;
  map_odom_transform_alpha_ = _params.map_odom_transform_alpha;
  earth_map_transform_ = initial_earth_to_map_transform_;
  calculate_odom_covariance_ = _params.calculate_odom_covariance_;
  throttle_detections_ = _params.throttle_detections;
  use_dual_graph_ = _params.use_dual_graph;

  PARAM(PRINT_VAR(main_graph_odometry_distance_threshold_));
  PARAM(PRINT_VAR(temp_graph_odometry_distance_threshold_));
  PARAM(PRINT_VAR(main_graph_odometry_distance_threshold_if_detections_));
  PARAM(PRINT_VAR(map_odom_transform_alpha_));
  PARAM(PRINT_VAR(calculate_odom_covariance_));
  PARAM(PRINT_VAR(use_dual_graph_));

  Eigen::MatrixXd earth_to_map_covariance_ = Eigen::MatrixXd::Identity(6, 6) * 0.0001;
  earth_to_map_covariance_(5, 5) = 0.1;
  // odometry_received_.covariance(4, 4) *= 10e4;
  // odometry_received_.covariance(3, 3) *= 10e4;

  main_graph->addNewKeyframe(initial_earth_to_map_transform_.inverse(),
    initial_earth_to_map_transform_.inverse(),
    earth_to_map_covariance_);

  main_graph->setMapNode(main_graph->getLastOdomNode());

  // last_odometry_added_.odometry = earth_to_map_transform_;
  // last_odometry_added_.covariance = Eigen::MatrixXd::Zero(6, 6);
  //
  main_graph->setFixedObjects(fixed_objects_);
}

void OptimizerG2O::updateOdomMapTransform()
{
  earth_map_transform_ = getOptimizedMapPose();

  Eigen::Isometry3d new_map_odom_tranform = earth_map_transform_.inverse() * getOptimizedPose() * last_odometry_added_.odometry.inverse();
  // Eigen::Isometry3d map_odom_diff = new_map_odom_tranform.inverse() * map_odom_tranform_;
  // if (map_odom_diff.translation().norm() > map_odom_security_threshold_) {
  //   WARN("Big Map-Odom transform difference: " << map_odom_diff.translation().norm());
  // }
  if (map_odom_transform_alpha_ < 1.0) {
    Eigen::Isometry3d filtered_map_odom_transform = filterTransform(map_odom_tranform_, new_map_odom_tranform);
    map_odom_tranform_ = filtered_map_odom_transform;
  }
  else {
    map_odom_tranform_ = new_map_odom_tranform;
  }
}

Eigen::Isometry3d OptimizerG2O::filterTransform(Eigen::Isometry3d _last_transform, Eigen::Isometry3d _new_transform) {
  // Interpolate translation
  Eigen::Vector3d old_translation = _last_transform.translation();
  Eigen::Vector3d new_translation = _new_transform.translation();
  Eigen::Vector3d filtered_translation = (1.0 - map_odom_transform_alpha_) * old_translation + map_odom_transform_alpha_ * new_translation;
  // Interpolate rotation
  Eigen::Quaterniond old_rot(_last_transform.rotation());
  Eigen::Quaterniond new_rot(_new_transform.rotation());
  Eigen::Quaterniond filtered_rot = old_rot.slerp(map_odom_transform_alpha_, new_rot);
  // Compose the filtered transform
  Eigen::Isometry3d filtered_transform = Eigen::Isometry3d::Identity();
  filtered_transform.linear() = filtered_rot.toRotationMatrix();
  filtered_transform.translation() = filtered_translation;

  return filtered_transform;
}

Eigen::Isometry3d OptimizerG2O::getOptimizedPose()
{
  return main_graph->getLastOdomNode()->getPose();
}

Eigen::Isometry3d OptimizerG2O::getOptimizedMapPose()
{
  return main_graph->getMapNode()->getPose();
}

Eigen::Isometry3d OptimizerG2O::getMapOdomTransform()
{
  return map_odom_tranform_;
}

Eigen::Isometry3d OptimizerG2O::getMapTransform()
{
  return earth_map_transform_;
}

bool OptimizerG2O::generateDetectionOdometryInfo(
  const OdometryWithCovariance & _detection_odometry,
  OdometryInfo & _detection_odometry_info)
{
  return generateOdometryInfo(_detection_odometry, last_odometry_added_, _detection_odometry_info);
}
