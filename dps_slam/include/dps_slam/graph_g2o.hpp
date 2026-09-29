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
 *  \file       graph_g2o.hpp
 *  \brief      An state estimation server for AeroStack2
 *  \authors    David Pérez Saura
 *              Miguel Fernández Cortizas
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef AS2_SLAM__GRAPH_G2O_HPP_
#define AS2_SLAM__GRAPH_G2O_HPP_

#include <g2o/core/optimizable_graph.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "dps_slam/graph_edge_types.hpp"
#include "dps_slam/graph_node_types.hpp"
#include "dps_slam/object_detection_types.hpp"
#include "utils/general_utils.hpp"

struct ObjectNodeInfo
{
  ObjectNodeInfo(
    const std::string & _id,
    g2o::HyperGraph::Vertex * _node,
    const Eigen::MatrixXd & _covariance);
  ~ObjectNodeInfo() {}

  std::string object_id;
  g2o::HyperGraph::Vertex * node;
  Eigen::MatrixXd covariance;
};

class GraphG2O
{
public:
  explicit GraphG2O(std::string _name);
  ~GraphG2O() {}

  std::string getName();
  std::vector<GraphNode *> getNodes();
  std::vector<GraphEdge *> getEdges();
  std::unordered_map<std::string, GraphNode *> getObjectNodes();
  // Current optimized pose of every odometry keyframe, in insertion order -- the
  // full corrected trajectory, unlike getLastOdomNode() (most recent only).
  std::vector<Eigen::Isometry3d> getOdomKeyframePoses();
  // Raw odometry pose each keyframe was created with, before optimization, same
  // order as getOdomKeyframePoses().
  std::vector<Eigen::Isometry3d> getOdomKeyframeRawPoses();
  OdomNode * getLastOdomNode();
  OdomNode * getMapNode();
  void setMapNode(OdomNode * _map_node);

  void addNode(GraphNode & _node);
  void addEdge(GraphEdge & _edge);
  void addNewKeyframe(
    const Eigen::Isometry3d & _absolute_pose,
    const Eigen::Isometry3d & _relative_pose,
    const Eigen::MatrixXd & _relative_covariance);
  void addNewObjectDetection(ObjectDetection * _object);
  Eigen::MatrixXd computeNodeCovariance(GraphNode * _node);

  bool optimizeGraph();
  void setFixedObjects(const std::vector<FixedObject> & _fixed_objects);
  void initGraph(const Eigen::Isometry3d & _initial_pose = Eigen::Isometry3d::Identity());
  // Huber delta applied to every edge added from now on (addEdge()). Without a robust
  // kernel, a single mismatched/outlier observation has unbounded influence on the
  // optimized trajectory under plain least-squares. Needs empirical tuning against
  // real data; roughly, smaller = rejects outliers more aggressively.
  void setRobustKernelDelta(double _delta);
  // When true (default), every keyframe added from now on gets a HeightPriorEdge
  // pinning its z to raw odometry. See graph_edge_types.hpp.
  void setRestrictKeyframeHeightToOdometry(bool _restrict);
  // Std (rad) of a prior keeping every plane landmark added from now on vertical
  // (g2o_custom::EdgePlaneVertical); <= 0 disables it.
  void setWallVerticalPriorStd(double _std_rad);
  std::shared_ptr<g2o::SparseOptimizer> graph_;  // g2o graph

  std::unordered_map<std::string, GraphNode *> obj_id2node_;
  std::vector<ObjectNodeInfo> obj_nodes_info_;

private:
  int n_vertices_ = 0;
  int n_edges_ = 0;
  std::string name_;
  OdomNode * last_odom_node_;
  OdomNode * map_node_;
  std::vector<OdomNode *> odom_keyframes_;
  std::vector<Eigen::Isometry3d> odom_keyframe_raw_poses_;
  std::vector<GraphNode *> graph_nodes_;
  std::vector<GraphEdge *> graph_edges_;
  double robust_kernel_delta_ = 3.0;
  bool restrict_keyframe_height_to_odometry_ = true;
  double wall_vertical_prior_std_rad_ = 0.0;
};

#endif  // AS2_SLAM__GRAPH_G2O_HPP_
