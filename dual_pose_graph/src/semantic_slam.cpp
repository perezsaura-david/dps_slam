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
 * @file semantic_slam.cpp
 *
 * SemanticSlam class implementation for AeroStack2
 *
 * @author David Pérez Saura
 *         Miguel Fernández Cortizas
 */

#include "dual_pose_graph/semantic_slam.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <g2o/core/optimizable_graph.h>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <rclcpp/subscription_options.hpp>
#include <string>
#include <vector>

#include "dual_pose_graph/graph_node_types.hpp"
#include "optimizer_g2o.hpp"
#include "utils/conversions.hpp"
#include "dual_pose_graph/object_detection_types.hpp"
#include "utils/debug_utils.hpp"
#include "utils/general_utils.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

// #include <filesystem>
// #include <pluginlib/class_loader.hpp>
// #include "plugin_base.hpp"

SemanticSlam::SemanticSlam(rclcpp::NodeOptions & options)
: as2::Node("semantic_slam", options)
{
  rclcpp::QoS reliable_qos = rclcpp::QoS(10);
  rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();


// PARAMETERS
  std::string odometry_topic = this->get_parameter("odometry_topic").as_string();
  std::string pose_topic = this->get_parameter("pose_topic").as_string();
  std::string corrected_localization_topic =
    this->get_parameter("corrected_localization_topic").as_string();
  std::string corrected_path_topic = this->get_parameter("corrected_path_topic").as_string();

  std::string detections_topic = this->get_parameter("detections_topic").as_string();
  // std::string aruco_pose_topic = this->get_parameter("aruco_pose_topic").as_string();
  // std::string gate_pose_topic = this->get_parameter("gate_pose_topic").as_string();

  map_frame_ = this->get_parameter("map_frame").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  robot_frame_ = this->get_parameter("robot_frame").as_string();
  // estimated_map_frame_ = map_frame_ + "_estimated";
  estimated_map_frame_ = map_frame_;

  if (this->has_parameter("force_object_type")) {
    force_object_type_ = this->get_parameter("force_object_type").as_string();
    PARAM("Forcing object type to: " + force_object_type_);
  }

  detection_covariance_by_distance_ = this->get_parameter("detection_covariance_by_distance").as_bool();
  detection_covariance_by_distance2_ = this->get_parameter("detection_covariance_by_distance2").as_bool();
  detection_covariance_factor_ = this->get_parameter("detection_covariance_factor").as_double();
  detection_orientation_covariance_factor_ = this->get_parameter("detection_orientation_covariance_factor").as_double();
  generate_orientation_cov_by_distance_ = this->get_parameter("generate_orientation_cov_by_distance").as_bool();
  distance_for_orientation_covariance_increment_ = this->get_parameter(
    "distance_for_orientation_covariance_increment").as_double();
  detection_orientation_covariance_large_factor_ = this->get_parameter(
    "detection_orientation_covariance_large_factor").as_double();
  // map_odom_transform_alpha_ = this->get_parameter(
  //   "map_odom_transform_alpha").as_double();

  PARAM("Detection covariance by distance: " << std::boolalpha
    << detection_covariance_by_distance_);
  PARAM("Detection covariance by distance2: " << std::boolalpha
    << detection_covariance_by_distance2_);
  PARAM(PRINT_VAR(detection_covariance_factor_));
  // PARAM(PRINT_VAR(map_odom_transform_alpha_));
  PARAM(PRINT_VAR(detection_orientation_covariance_factor_));
  PARAM(PRINT_VAR(generate_orientation_cov_by_distance_));
  PARAM(PRINT_VAR(distance_for_orientation_covariance_increment_));
  PARAM(PRINT_VAR(detection_orientation_covariance_large_factor_));

// VISUALIZATION
  visualize_graphs_ = this->get_parameter("visualize_graphs").as_bool();
  std::string viz_main_markers_topic = this->get_parameter("viz_main_markers_topic").as_string();
  std::string viz_temp_markers_topic = this->get_parameter("viz_temp_markers_topic").as_string();


// ROS2
  if (!odometry_topic.empty()) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, sensor_qos,
      std::bind(&SemanticSlam::odomCallback, this, std::placeholders::_1));
    PARAM("Subscribed to Odometry topic: " << odometry_topic);

  } else if (!pose_topic.empty()) {
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, sensor_qos,
      std::bind(&SemanticSlam::poseStampedCallback, this, std::placeholders::_1));
    PARAM("Subscribed to PoseStamped topic: " << pose_topic);
  } else {
    RCLCPP_ERROR(this->get_logger(), "No odometry or pose topic provided");
  }

  detections_callback_group_= this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions det_options;
  det_options.callback_group = detections_callback_group_;
  detections_sub_ = this->create_subscription<as2_msgs::msg::PoseStampedWithIDArray>(
    detections_topic, sensor_qos,
    std::bind(&SemanticSlam::detectionsCallback, this, std::placeholders::_1),
    det_options);
  // aruco_pose_sub_ = this->create_subscription<as2_msgs::msg::PoseStampedWithID>(
  //   aruco_pose_topic, sensor_qos,
  //   std::bind(&SemanticSlam::arucoPoseCallback, this, std::placeholders::_1));
  // gate_pose_sub_ = this->create_subscription<as2_msgs::msg::PoseStampedWithID>(
  //   gate_pose_topic, sensor_qos,
  //   std::bind(&SemanticSlam::gatePoseCallback, this, std::placeholders::_1));

  corrected_localization_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    corrected_localization_topic, reliable_qos);
  corrected_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    corrected_path_topic, reliable_qos);
  if (visualize_graphs_) {
    viz_main_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      viz_main_markers_topic, reliable_qos);
    viz_temp_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      viz_temp_markers_topic, reliable_qos);
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);


  csv_logger_ = std::make_unique<CsvLogger>(".");

  optimizer_ptr_ = std::make_unique<OptimizerG2O>();
  optimizer_ptr_->setParameters(getOptimizerParameters());
  optimizer_ptr_->setCsvLogger(csv_logger_.get());

  std_msgs::msg::Header header;
  header.stamp = this->now();
  updateMapOdomTransform(header);
  tf_broadcaster_->sendTransform(map_odom_transform_msg_);
  updateEarthMapTransform(header);
  tf_broadcaster_->sendTransform(earth_map_transform_msg_);
  // Callback group
  tf_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
    if (!generate_odom_map_transform_) {return;}
    // Create a timer to publish the transform at a fixed rate
    tf_publish_timer_ = this->create_timer(
      std::chrono::duration<double>(1.0 / 100.0),
      [this]() {
        optimizer_ptr_->updateOdomMapTransform();
        std_msgs::msg::Header header;
        header.stamp = this->now();
        updateMapOdomTransform(header);
        tf_broadcaster_->sendTransform(map_odom_transform_msg_);
        updateEarthMapTransform(header);
        tf_broadcaster_->sendTransform(earth_map_transform_msg_);
      },
    tf_callback_group_);

}

// Eigen::Isometry3d SemanticSlam::filterTransform(Eigen::Isometry3d _last_transform, Eigen::Isometry3d _new_transform) {
//   // Interpolate translation
//   Eigen::Vector3d old_translation = _last_transform.translation();
//   Eigen::Vector3d new_translation = _new_transform.translation();
//   Eigen::Vector3d filtered_translation = (1.0 - map_odom_transform_alpha_) * old_translation + map_odom_transform_alpha_ * new_translation;
//   // Interpolate rotation
//   Eigen::Quaterniond old_rot(_last_transform.rotation());
//   Eigen::Quaterniond new_rot(_new_transform.rotation());
//   Eigen::Quaterniond filtered_rot = old_rot.slerp(map_odom_transform_alpha_, new_rot);
//   // Compose the filtered transform
//   Eigen::Isometry3d filtered_transform = Eigen::Isometry3d::Identity();
//   filtered_transform.linear() = filtered_rot.toRotationMatrix();
//   filtered_transform.translation() = filtered_translation;

//   return filtered_transform;
// }

////// CALLBACKS //////

void SemanticSlam::poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  Eigen::Isometry3d pose = convertToIsometry3d(msg->pose);
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(6, 6) * 0.001;

  processOdometryReceived(pose, covariance, msg->header);
}

void SemanticSlam::processOdometryReceived(
  const Eigen::Isometry3d _odom_pose,
  const Eigen::MatrixXd _odom_covariance,
  const std_msgs::msg::Header & _header)
{
  if (!_odom_pose.translation().isZero()) {
  // DEBUG_START_TIMER
    OdometryWithCovariance odometry_received_;
    odometry_received_.odometry = _odom_pose;
    odometry_received_.covariance = _odom_covariance;

    // TODO(dps): Define how to use this
    // msg->header.stamp;
    bool new_node_added = optimizer_ptr_->handleNewOdom(odometry_received_);

    last_odometry_received_ = odometry_received_;

    if (new_node_added) {
      if (visualize_graphs_) {
        visualizeMainGraph();
        visualizeCleanTempGraph();
      }
      updateMapOdomTransform(_header);
      updateEarthMapTransform(_header);
      // DEBUG_LOG_DURATION
    }
  }

  // Get map_odom_transform from optimizer and publish corrected localization
  Eigen::Isometry3d map_odom_transform = optimizer_ptr_->getMapOdomTransform();
  Eigen::Isometry3d map_transform = optimizer_ptr_->getMapTransform();
  // Eigen::Isometry3d corrected_odometry_pose = map_odom_transform * _odom_pose;
  Eigen::Isometry3d corrected_odometry_pose = map_transform * map_odom_transform * _odom_pose;
  geometry_msgs::msg::PoseWithCovarianceStamped corrected_localization_msg;
  geometry_msgs::msg::PoseStamped pose_stamped_msg;
  pose_stamped_msg.pose = convertToGeometryMsgPose(corrected_odometry_pose);
  corrected_localization_msg.pose.pose = pose_stamped_msg.pose;
  corrected_localization_msg.header.stamp = _header.stamp;
  corrected_localization_msg.header.frame_id = earth_frame_;
  corrected_localization_pub_->publish(corrected_localization_msg);

  if (csv_logger_) {
    csv_logger_->logOdom(
      _header.stamp.sec, _header.stamp.nanosec,
      _odom_pose, _odom_covariance, corrected_odometry_pose);
  }

  if (visualize_graphs_) {
    // Publish corrected Path
    static nav_msgs::msg::Path corrected_path_msg;
    corrected_path_msg.header.stamp = _header.stamp;
    corrected_path_msg.header.frame_id = earth_frame_;
    corrected_path_msg.poses.emplace_back(pose_stamped_msg);
    corrected_path_pub_->publish(corrected_path_msg);
  }
}

void SemanticSlam::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  Eigen::Isometry3d odom_isometry;
  if (msg->header.frame_id == "global" && msg->child_frame_id == "imu") {
    odom_isometry = getOdometryFromOpenVins(*msg);
  } else {
    odom_isometry = convertToIsometry3d(msg->pose.pose);
  }
  Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> odom_covariance(
    msg->pose.covariance.data());
  // WARN("Odometry covariance: " << odom_covariance);
  // Eigen::Matrix<double, 6, 6> odom_covariance = Eigen::MatrixXd::Identity(6, 6) * 10e-5;
  // odom_covariance(0,0) = 10e-3;
  processOdometryReceived(odom_isometry, odom_covariance, msg->header);
}

void SemanticSlam::detectionsCallback(
  const as2_msgs::msg::PoseStampedWithIDArray::SharedPtr msg)
{
  if (last_odometry_received_.odometry.translation().isZero()) {
    WARN("Detection received before odometry info");
    return;
  }
  // Safely check array size before access
  if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty PoseStampedWithIDArray");
      return;
  }

  // Lookup transform for baselink in odometry frame at timestamp
  OdometryWithCovariance detection_odometry;
  auto target_ts = msg->poses.front().pose.header.stamp;
  std::chrono::nanoseconds tf_timeout =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0));
  try {
    // DEBUG("Get Odom Pose at detection time: " << odom_frame_ << " to " << robot_frame_);
    auto detection_odometry_transform =
      tf_buffer_->lookupTransform(odom_frame_, robot_frame_, target_ts, tf_timeout);
    detection_odometry.odometry = convertToIsometry3d(detection_odometry_transform.transform);
    detection_odometry.covariance = last_odometry_received_.covariance;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform %s to %s: %s", odom_frame_.c_str(),
      robot_frame_.c_str(), ex.what());
  }

  // DEBUG_START_TIMER
  OdometryInfo detection_odometry_info;
  if (use_dual_graph_) {
    if (!optimizer_ptr_->checkAddingNewDetection(detection_odometry, detection_odometry_info)) {
      return;
    }
  } else {
    if (!optimizer_ptr_->generateDetectionOdometryInfo(detection_odometry, detection_odometry_info)) {
      return;
    }
  }

  std::string object_type;
  if (force_object_type_.empty()) {
    // FIXME: Create new msg with type
    // object_type = msg->type;
    object_type = force_object_type_;
  } else {
    object_type = force_object_type_;
  }
  if (object_type.empty()) {
    ERROR("No object type provided. Force object type or use 'type' field in msg");
    return;
  }

  for (auto & detection : msg->poses) {
    if (object_type == "aruco") {
      processArucoMsg(detection, detection_odometry_info);
    } else if (object_type == "gate") {
      processGateMsg(detection, detection_odometry_info);
    }
  }

  if (use_dual_graph_ && visualize_graphs_) {
    visualizeTempGraph();
  }
}

void SemanticSlam::arucoPoseCallback(const as2_msgs::msg::PoseStampedWithID::SharedPtr msg)
{
  OdometryInfo detection_odometry_info;
  if (use_dual_graph_) {
    if (!optimizer_ptr_->checkAddingNewDetection(last_odometry_received_, detection_odometry_info)) {
      return;
    }
  } else {
    if (!optimizer_ptr_->generateDetectionOdometryInfo(last_odometry_received_, detection_odometry_info)) {
      return;
    }
  }
  processArucoMsg(*msg, detection_odometry_info);
}

void SemanticSlam::gatePoseCallback(const as2_msgs::msg::PoseStampedWithID::SharedPtr msg)
{
  OdometryInfo detection_odometry_info;
  if (use_dual_graph_) {
    if (!optimizer_ptr_->checkAddingNewDetection(last_odometry_received_, detection_odometry_info)) {
      return;
    }
  } else {
    if (!optimizer_ptr_->generateDetectionOdometryInfo(last_odometry_received_, detection_odometry_info)) {
      return;
    }
  }
  processGateMsg(*msg, detection_odometry_info);
}

////// PROCESS //////

void SemanticSlam::processGateMsg(
  const as2_msgs::msg::PoseStampedWithID _msg,
  const OdometryInfo _detection_odometry_info)
{
  std::string gate_id = _msg.id;
  Eigen::Vector3d gate_position = generatePoseFromMsg(_msg).translation();
  Eigen::Matrix<double, 3, 3> gate_covariance = Eigen::MatrixXd::Identity(3, 3) * detection_covariance_factor_;
  if (detection_covariance_by_distance_) {
    double distance = gate_position.norm();
    gate_covariance = Eigen::MatrixXd::Identity(3, 3) * distance;
  }
  else if (detection_covariance_by_distance2_) {
    double distance = gate_position.norm();
    gate_covariance = Eigen::MatrixXd::Identity(3, 3) * distance * distance;
  }

  bool detections_are_absolute = false;

  if (csv_logger_) {
    csv_logger_->logDetection(
      _msg.pose.header.stamp.sec, _msg.pose.header.stamp.nanosec,
      gate_id, "gate", gate_position,
      _detection_odometry_info.odom_ref.translation(), detections_are_absolute);
  }

  GateDetection * gate(new GateDetection(
      gate_id, gate_position, gate_covariance,
      detections_are_absolute));

  optimizer_ptr_->handleNewObjectDetection(gate, _detection_odometry_info);
}

void SemanticSlam::processArucoMsg(
  const as2_msgs::msg::PoseStampedWithID _msg,
  const OdometryInfo _detection_odometry_info)
{
  std::string aruco_id = _msg.id;
  // TODO(dps): Define how to use this
  // msg->pose.header.stamp;
  Eigen::Isometry3d aruco_pose = generatePoseFromMsg(_msg);
  Eigen::Matrix<double, 6, 6> aruco_covariance = Eigen::MatrixXd::Identity(6, 6) * detection_covariance_factor_;
  if (detection_covariance_by_distance_) {
    double distance = aruco_pose.translation().norm();
    aruco_covariance = aruco_covariance * distance;
  }
  if (detection_covariance_by_distance2_) {
    double distance = aruco_pose.translation().norm();
    aruco_covariance = aruco_covariance * distance * distance;
  }
  aruco_covariance(3, 3) *= detection_orientation_covariance_factor_;
  aruco_covariance(4, 4) *= detection_orientation_covariance_factor_;
  aruco_covariance(5, 5) *= detection_orientation_covariance_factor_;
  // calculate distance
  if (generate_orientation_cov_by_distance_) {
    float distance = aruco_pose.translation().norm();
    if (distance > distance_for_orientation_covariance_increment_) {
      // ERROR("Increasing orientation covariance");
      aruco_covariance(3, 3) *= detection_orientation_covariance_large_factor_;
      aruco_covariance(4, 4) *= detection_orientation_covariance_large_factor_;
      aruco_covariance(5, 5) *= detection_orientation_covariance_large_factor_;
    }
  }
  // WARN(aruco_covariance);

  bool detections_are_absolute = false;

  if (csv_logger_) {
    csv_logger_->logDetection(
      _msg.pose.header.stamp.sec, _msg.pose.header.stamp.nanosec,
      aruco_id, "aruco", aruco_pose.translation(),
      _detection_odometry_info.odom_ref.translation(), detections_are_absolute);
  }

  ArucoDetection * aruco(new ArucoDetection(
      aruco_id, aruco_pose, aruco_covariance,
      detections_are_absolute));

  optimizer_ptr_->handleNewObjectDetection(aruco, _detection_odometry_info);
}

void SemanticSlam::updateMapOdomTransform(const std_msgs::msg::Header & _header)
{
  map_odom_transform_msg_ = convertToTransformStamped(
    optimizer_ptr_->getMapOdomTransform(), estimated_map_frame_, odom_frame_, _header.stamp);
}

void SemanticSlam::updateEarthMapTransform(const std_msgs::msg::Header & _header)
{
  earth_map_transform_msg_ = convertToTransformStamped(
    optimizer_ptr_->getMapTransform(), earth_frame_, estimated_map_frame_, _header.stamp);
}
Eigen::Isometry3d SemanticSlam::generatePoseFromMsg(
  const as2_msgs::msg::PoseStampedWithID & _msg)
{
  Eigen::Isometry3d pose;
  std::string ref_frame = _msg.pose.header.frame_id;
  // auto target_ts        = tf2::TimePointZero;
  auto target_ts = _msg.pose.header.stamp;
  std::chrono::nanoseconds tf_timeout =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0));
  auto tf_names = tf_buffer_->getAllFrameNames();

  // FIXME: We need this in simulation because of the current optical link implementation in
  // gazebo
  for (auto tf_name : tf_names) {
    if (tf_name.find(ref_frame) < tf_name.length()) {
      ref_frame = tf_name;
      // WARN("Ref frame changed to: " << ref_frame);
    }
  }

  // CHANGED REFERENCE_FRAME TO ROBOT_FRAME
  if (ref_frame != robot_frame_) {
    geometry_msgs::msg::TransformStamped ref_frame_transform;
    try {
      // INFO("Transform detection from " << ref_frame << " to " << robot_frame_);
      ref_frame_transform =
        tf_buffer_->lookupTransform(robot_frame_, ref_frame, target_ts, tf_timeout);
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(_msg.pose, transformed_pose, ref_frame_transform);
      pose = convertToIsometry3d(transformed_pose.pose);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s", ref_frame.c_str(),
        robot_frame_.c_str(), ex.what());
    }
  } else {
    pose = convertToIsometry3d(_msg.pose.pose);
  }
  return pose;
}

////// VISUALIZATION //////

void SemanticSlam::visualizeMainGraph()
{
  visualization_msgs::msg::MarkerArray viz_odom_nodes_msg =
    generateVizNodesMsg(optimizer_ptr_->main_graph);
  viz_main_markers_pub_->publish(viz_odom_nodes_msg);
  visualization_msgs::msg::MarkerArray viz_edges_msg =
    generateVizEdgesMsg(optimizer_ptr_->main_graph);
  viz_main_markers_pub_->publish(viz_edges_msg);
}

void SemanticSlam::visualizeTempGraph()
{
  optimizer_ptr_->graph_mutex_.lock();
  if (!optimizer_ptr_->temp_graph) {
    WARN("Temp graph not created yet");
    return;
  }
  visualization_msgs::msg::MarkerArray viz_odom_nodes_msg =
    generateVizNodesMsg(optimizer_ptr_->temp_graph);
  viz_temp_markers_pub_->publish(viz_odom_nodes_msg);
  visualization_msgs::msg::MarkerArray viz_edges_msg =
    generateVizEdgesMsg(optimizer_ptr_->temp_graph);
  viz_temp_markers_pub_->publish(viz_edges_msg);
  optimizer_ptr_->graph_mutex_.unlock();
}

void SemanticSlam::visualizeCleanTempGraph()
{
  visualization_msgs::msg::MarkerArray viz_clean_markers_msg = generateCleanMarkersMsg();
  viz_temp_markers_pub_->publish(viz_clean_markers_msg);
}

visualization_msgs::msg::MarkerArray SemanticSlam::generateVizNodesMsg(
  std::shared_ptr<GraphG2O> & _graph)
{
  bool main = false;
  std::string viz_frame = earth_frame_;
  if (_graph->getName() == "Main Graph") {
    main = true;
    viz_frame = earth_frame_;
  }
  visualization_msgs::msg::MarkerArray viz_markers_msg;
  std::vector<GraphNode *> graph_nodes = _graph->getNodes();
  for (auto & node : graph_nodes) {
    visualization_msgs::msg::Marker viz_marker_msg = node->getVizMarker(main);
    viz_marker_msg.header.frame_id = viz_frame;
    viz_markers_msg.markers.emplace_back(viz_marker_msg);
    // visualization_msgs::msg::Marker viz_cov_marker_msg = node->getVizCovMarker();
    // viz_markers_msg.markers.emplace_back(viz_marker_msg);
  }
  return viz_markers_msg;
}

visualization_msgs::msg::MarkerArray SemanticSlam::generateVizEdgesMsg(
  std::shared_ptr<GraphG2O> & _graph)
{
  bool main = false;
  std::string viz_frame = earth_frame_;
  if (_graph->getName() == "Main Graph") {
    main = true;
    viz_frame = earth_frame_;
  }
  visualization_msgs::msg::MarkerArray viz_markers_msg;
  std::vector<GraphEdge *> graph_edges = _graph->getEdges();
  for (auto & edge : graph_edges) {
    visualization_msgs::msg::Marker viz_marker_msg = edge->getVizMarker(main);
    viz_marker_msg.header.frame_id = viz_frame;
    viz_markers_msg.markers.emplace_back(viz_marker_msg);
  }
  return viz_markers_msg;
}

visualization_msgs::msg::MarkerArray SemanticSlam::generateCleanMarkersMsg()
{
  visualization_msgs::msg::MarkerArray markers_msg;
  visualization_msgs::msg::Marker marker_msg;
  // marker_msg.ns     = _namespace;
  marker_msg.id = 0;
  marker_msg.action = visualization_msgs::msg::Marker::DELETEALL;
  markers_msg.markers.emplace_back(marker_msg);
  return markers_msg;
}

// PARAMETERS

OptimizerG2OParameters SemanticSlam::getOptimizerParameters() {
  OptimizerG2OParameters optimizer_params;
  optimizer_params.main_graph_odometry_distance_threshold = this->get_parameter(
    "main_graph_odometry_distance_threshold").as_double();
  optimizer_params.main_graph_odometry_orientation_threshold = this->get_parameter(
    "main_graph_odometry_orientation_threshold").as_double();
  optimizer_params.temp_graph_odometry_distance_threshold = this->get_parameter(
    "temp_graph_odometry_distance_threshold").as_double();
  optimizer_params.temp_graph_odometry_orientation_threshold = this->get_parameter(
    "temp_graph_odometry_orientation_threshold").as_double();
  optimizer_params.map_odom_security_threshold = this->get_parameter(
    "map_odom_security_threshold").as_double();
  optimizer_params.main_graph_odometry_distance_threshold_if_detections = this->get_parameter(
    "main_graph_odometry_distance_threshold_if_detections").as_double();
  optimizer_params.odometry_is_relative = this->get_parameter("odometry_is_relative").as_bool();
  optimizer_params.generate_odom_map_transform =
    this->get_parameter("generate_odom_map_transform").as_bool();
  generate_odom_map_transform_ = optimizer_params.generate_odom_map_transform;
  optimizer_params.map_odom_transform_alpha = this->get_parameter(
    "map_odom_transform_alpha").as_double();
  optimizer_params.calculate_odom_covariance_ = this->get_parameter(
    "calculate_odom_covariance").as_bool();
  if (this->has_parameter("throttle_detections")) {
    optimizer_params.throttle_detections = this->get_parameter(
      "throttle_detections").as_bool();
  } else {
    optimizer_params.throttle_detections = true;
  }
  if (this->has_parameter("use_dual_graph")) {
    optimizer_params.use_dual_graph = this->get_parameter("use_dual_graph").as_bool();
  } else {
    optimizer_params.use_dual_graph = true;
  }
  use_dual_graph_ = optimizer_params.use_dual_graph;

  double earth_to_map_x = 0.0;
  double earth_to_map_y = 0.0;
  double earth_to_map_z = 0.0;
  double earth_to_map_roll = 0.0;
  double earth_to_map_pitch = 0.0;
  double earth_to_map_yaw = 0.0;

  if (this->has_parameter("earth_to_map.x")) {
    this->get_parameter("earth_to_map.x", earth_to_map_x);
  }
  if (this->has_parameter("earth_to_map.y")) {
    this->get_parameter("earth_to_map.y", earth_to_map_y);
  }
  if (this->has_parameter("earth_to_map.z")) {
    this->get_parameter("earth_to_map.z", earth_to_map_z);
  }
  if (this->has_parameter("earth_to_map.roll")) {
    this->get_parameter("earth_to_map.roll", earth_to_map_roll);
  }
  if (this->has_parameter("earth_to_map.pitch")) {
    this->get_parameter("earth_to_map.pitch", earth_to_map_pitch);
  }
  if (this->has_parameter("earth_to_map.yaw")) {
    this->get_parameter("earth_to_map.yaw", earth_to_map_yaw);
  }
  RCLCPP_INFO(
    this->get_logger(), "Earth to map: xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
    earth_to_map_x, earth_to_map_y, earth_to_map_z,
    earth_to_map_roll, earth_to_map_pitch, earth_to_map_yaw);

  Eigen::Vector3d e2m_translation =
    Eigen::Vector3d(earth_to_map_x, earth_to_map_y, earth_to_map_z);
  Eigen::Quaterniond e2m_rotation =
    Eigen::AngleAxisd(earth_to_map_yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(earth_to_map_pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(earth_to_map_roll, Eigen::Vector3d::UnitX());
  earth_to_map_transform_ = convertToIsometry3d(e2m_translation, e2m_rotation);
  optimizer_params.earth_to_map_transform = earth_to_map_transform_;

  // Storage for parsed gates
  std::map<std::string, std::pair<std::string, std::vector<double>>> fixed_objects;
  // List all parameters that start with "fixed_poses"
  auto result = this->list_parameters({"fixed_objects"}, 3);
  parseFixedObjects(result, fixed_objects, optimizer_params);

  WARN("odometry_is_relative not implemented yet");
  WARN("map_odom_security_threshold not implemented yet");

  return optimizer_params;
}

void SemanticSlam::parseFixedObjects(
  const rcl_interfaces::msg::ListParametersResult & result,
  std::map<std::string, std::pair<std::string, std::vector<double>>> & fixed_objects,
  OptimizerG2OParameters & optimizer_params)
{
  // Iterate through all parameters found
  for (const auto & param_name : result.names) {
    // Extract gate name by removing "fixed_objects." prefix
    std::string fixed_object_prefix = "fixed_objects.";
    if (param_name.find(fixed_object_prefix) != 0) {
      continue; // Safety check
    }

    // Extract gate ID and check if it's for type or pose
    std::string fixed_object_id = param_name.substr(fixed_object_prefix.length());

    size_t dot_pos = fixed_object_id.rfind('.');
    if (dot_pos == std::string::npos) {
      continue; // Ignore invalid parameters
    }

    std::string object_id = fixed_object_id.substr(0, dot_pos);
    std::string suffix = fixed_object_id.substr(dot_pos + 1);

    // Ensure an entry exists for this object
    if (fixed_objects.find(object_id) == fixed_objects.end()) {
      fixed_objects[object_id] = {"", {}}; // Initialize with empty values
    }

    if (suffix == "type") {
      std::string object_type;
      if (!this->get_parameter(param_name, object_type)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read 'type' for %s.", object_id.c_str());
        continue;
      }
      fixed_objects[object_id].first = object_type; // Store object type
    } else if (suffix == "pose") {
      std::vector<double> pose;
      // FIXME: Pose may be 4 (yaw only), 6 (euler angles), or 7 (quaternion)
      if (!this->get_parameter(param_name, pose) || pose.size() != 4) {
        RCLCPP_WARN(this->get_logger(), "Invalid or missing pose for %s.", object_id.c_str());
        continue;
      }
      fixed_objects[object_id].second = pose; // Store pose
    }
  }

  for (const auto &[id, data] : fixed_objects) {
    if (data.first.empty() || data.second.empty()) {
      RCLCPP_WARN(this->get_logger(), "Incomplete data for fixed object %s.", id.c_str());
      continue;
    }
  }

  for (const auto &[id, data] : fixed_objects) {
    if (data.first.empty() || data.second.empty()) {
      RCLCPP_WARN(this->get_logger(), "Incomplete data for object %s.", id.c_str());
      continue;
    }
    FixedObject fixed_object;
    fixed_object.id = id;
    fixed_object.type = data.first;
    if (!force_object_type_.empty()) {
      fixed_object.type = force_object_type_;
    }
    Eigen::Vector3d object_position =
      Eigen::Vector3d(data.second[0], data.second[1], data.second[2]);
    double yaw = data.second[3];
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Isometry3d earth_object_isometry = convertToIsometry3d(object_position, orientation);
    // fixed_object.isometry = earth_to_map_transform_.inverse() * earth_object_isometry;
    fixed_object.isometry = earth_object_isometry;
    optimizer_params.fixed_objects.emplace_back(fixed_object);
  }
}

// UTILS

Eigen::Isometry3d SemanticSlam::getOdometryFromOpenVins(const nav_msgs::msg::Odometry & _msg) {
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "Received odometry with frame_id 'global' and child_frame_id 'imu'. "
      "This is likely an OpenVINS odometry message. "
      "The odometry will be transformed to the correct frame.");
    nav_msgs::msg::Odometry odom_pose = _msg;
    // Create a quaternion representing a 180-degree rotation around the Z axis
    tf2::Quaternion rotation_z_180;
    rotation_z_180.setRPY(0, 0, M_PI); // Roll = 0, Pitch = 0, Yaw = π (180°)
    // Convert the original orientation from geometry_msgs to tf2
    tf2::Quaternion original_orientation;
    original_orientation.setX(_msg.pose.pose.orientation.x);
    original_orientation.setY(_msg.pose.pose.orientation.y);
    original_orientation.setZ(_msg.pose.pose.orientation.z);
    original_orientation.setW(_msg.pose.pose.orientation.w);
    // Apply the rotation
    tf2::Quaternion new_orientation = rotation_z_180 * original_orientation;
    new_orientation.normalize();
    // Set the new orientation back to the message
    odom_pose.pose.pose.orientation = tf2::toMsg(new_orientation);
    // Set the new position
    odom_pose.pose.pose.position.x *= -1.0;
    odom_pose.pose.pose.position.y *= -1.0;
    odom_pose.header.frame_id = odom_frame_;
    odom_pose.child_frame_id = robot_frame_;
    return convertToIsometry3d(odom_pose.pose.pose);
}
