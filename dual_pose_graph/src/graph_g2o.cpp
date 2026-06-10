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
 * @file graph_g2o.cpp
 *
 * OptimizerG2O class implementation for AeroStack2
 *
 * @author David Pérez Saura
 *         Miguel Fernández Cortizas
 */

#include "dual_pose_graph/graph_g2o.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/IndexedViewHelper.h>
#include <Eigen/src/Geometry/Transform.h>
#include <g2o/core/optimization_algorithm_factory.h>

#include <g2o/core/parameter.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>

#include "dual_pose_graph/graph_edge_types.hpp"
#include "dual_pose_graph/graph_node_types.hpp"
#include "dual_pose_graph/object_detection_types.hpp"
#include "utils/conversions.hpp"
#include "utils/general_utils.hpp"
#include "utils/debug_utils.hpp"

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

GraphG2O::GraphG2O(std::string _name)
{
  name_ = _name;
  FLAG_GRAPH("Create " << name_);

  graph_ = std::make_shared<g2o::SparseOptimizer>();                   // g2o graph
  std::string solver_type = "lm_var_cholmod";                          // Check list of solver types
  // INFO("construct solver: " << solver_type);
  g2o::OptimizationAlgorithmFactory * solver_factory =
    g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm * solver = solver_factory->construct(solver_type, solver_property);
  graph_->setAlgorithm(solver);

  g2o::ParameterSE3Offset * sensorOffset = new g2o::ParameterSE3Offset;
  sensorOffset->setOffset(Eigen::Isometry3d().Identity());
  sensorOffset->setId(0);
  graph_->addParameter(sensorOffset);


  if (!graph_->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate main solver!!" << std::endl;
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }

  n_vertices_ = 0;
  n_edges_ = 0;
}

std::string GraphG2O::getName() {return name_;}
std::vector<GraphNode *> GraphG2O::getNodes() {return graph_nodes_;}
std::vector<GraphEdge *> GraphG2O::getEdges() {return graph_edges_;}
std::unordered_map<std::string, GraphNode *> GraphG2O::getObjectNodes() {return obj_id2node_;}
OdomNode * GraphG2O::getLastOdomNode() {return last_odom_node_;}
OdomNode * GraphG2O::getMapNode() {return map_node_;}

void GraphG2O::setMapNode(OdomNode * _map_node)
{
  map_node_ = _map_node;
}

void GraphG2O::setFixedObjects(const std::vector<FixedObject> & _fixed_objects)
{
  for (auto object : _fixed_objects) {
    GraphNode * fixed_node = nullptr;

    if (object.type == "aruco") {
      fixed_node = new ArucoNode(object.isometry);
    } else if (object.type == "gate") {
      fixed_node = new GateNode(object.isometry.translation());
    }

    if (fixed_node) {   // Check if the pointer was initialized
      fixed_node->setFixed();
      // fixed_node->setCovariance(aruco_covariance);
      addNode(*fixed_node);
      obj_id2node_[object.id] = fixed_node;
      FLAG_GRAPH("Added fixed object ID: " << object.id);
    } else {
      FLAG_GRAPH("Warning: Unrecognized object type: " << object.type);
    }
  }
}

void GraphG2O::initGraph(const Eigen::Isometry3d & _initial_pose)
{
  // this position will help to find the correct solution (like the prior)
  // Initial_pose is set to (0,0,0) by default
  Eigen::Isometry3d node_pose = _initial_pose;
  OdomNode * fixed_node(new OdomNode(node_pose));
  fixed_node->setFixed();
  addNode(*fixed_node);
  last_odom_node_ = fixed_node;
}

bool GraphG2O::optimizeGraph()
{
  if (graph_->vertices().size() == 0) {
    ERROR_GRAPH("No vertices in the optimizer! Skipping optimization.");
    return false;
  }
  if (graph_->edges().size() == 0) {
    ERROR_GRAPH("No edges in the optimizer! Skipping optimization.");
    return false;
  }

  DEBUG_START_TIMER
  const int num_iterations = 100;
  // INFO_GRAPH("--- optimizing graph ---");
  INFO_GRAPH("nodes: " << graph_->vertices().size() << "   edges: " << graph_->edges().size());
  // std::cout << "optimizing... " << std::flush;
  // std::cout << "init" << std::endl;
  graph_->initializeOptimization();
  graph_->setVerbose(false);

  double chi2 = graph_->chi2();
  if (std::isnan(chi2)) {
    ERROR_GRAPH("GRAPH RETURNED A NAN BEFORE OPTIMIZATION");
    // return false;
  }
  // std::cout << "Start optimization" << std::endl;
  graph_->optimize(num_iterations);
  // int iterations = graph_->optimize(num_iterations);
  // FLAG_GRAPH("Optimization done");
  // std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
  // std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
  if (std::isnan(graph_->chi2())) {
    // FIXME(dps): If temp graph, reset
    // throw std::invalid_argument("GRAPH RETURNED A NAN...STOPPING THE EXPERIMENT");
    ERROR_GRAPH("GRAPH RETURNED A NAN AFTER OPTIMIZATION");
    return false;
  }
  DEBUG_LOG_DURATION_GRAPH
  return true;
}

void GraphG2O::addNode(GraphNode & _node)
{
  int id = n_vertices_++;
  _node.getVertex()->setId(id);
  if (!graph_){
    WARN_GRAPH("Graph is null");
    return;
  }
  if (!graph_->addVertex(_node.getVertex())) {
    WARN_GRAPH("Vertex not added");
    return;
  }
  graph_nodes_.emplace_back(&_node);
}

void GraphG2O::addEdge(GraphEdge & _edge)
{
  int id = n_edges_++;
  if (!graph_){
    WARN_GRAPH("Graph is null");
    return;
  }
  if (_edge.getEdge() == nullptr) {
    ERROR_GRAPH("Edge is null");
    return;
  }
  _edge.getEdge()->setId(id);
  if (!graph_->addEdge(_edge.getEdge())) {
    WARN_GRAPH("Edge not added");
    return;
  }
  graph_edges_.emplace_back(&_edge);
}

void GraphG2O::addNewKeyframe(
  const Eigen::Isometry3d & _absolute_pose,
  const Eigen::Isometry3d & _relative_pose,
  const Eigen::MatrixXd & _relative_covariance)
{
  OdomNode * odom_node(new OdomNode(_absolute_pose));
  addNode(*odom_node);

  Eigen::MatrixXd information_matrix = _relative_covariance.inverse();
  OdomEdge * odom_edge(new OdomEdge(
      last_odom_node_, odom_node, _relative_pose,
      information_matrix));
  if (odom_edge == nullptr) {
    ERROR_GRAPH("Odom edge is null");
    return;
  }
  addEdge(*odom_edge);
  last_odom_node_ = odom_node;
}

void GraphG2O::addNewObjectDetection(
  ObjectDetection * _object_detection)
{
  if (_object_detection == nullptr) {
    return;
  }
  GraphNode * object_node;
  // Get object node from list
  object_node = obj_id2node_[_object_detection->getId()];
  if (!object_node) {
    object_node = _object_detection->createNode();
    addNode(*object_node);

    obj_id2node_[_object_detection->getId()] = object_node;
    FLAG_GRAPH("Added new object ID: " << _object_detection->getId());
  } else {
    // INFO_GRAPH("Already detected object ID: " << _object_detection->getId());
  }
  if (_object_detection == nullptr) {
    return;
  }

  if (last_odom_node_ == nullptr) {
    ERROR_GRAPH("Last odom node is null, skipping edge creation");
    return;
  }
  GraphEdge * object_edge = _object_detection->createEdge(last_odom_node_, object_node);
  if (object_edge == nullptr) {
    ERROR_GRAPH("Object edge is null");
    return;
  }
  addEdge(*object_edge);
}

Eigen::MatrixXd GraphG2O::computeNodeCovariance(GraphNode * _node)
{
  // ERROR_GRAPH("Compute node covariance");
  // if (!_node->getVertex()) {
  //   ERROR_GRAPH("Node has no vertex");
  //   return Eigen::MatrixXd();
  // }
  // ERROR_GRAPH("Node has vertex");
  int node_id = _node->getVertex()->id();   // Get the g2o vertex ID
  g2o::SparseBlockMatrix<Eigen::MatrixXd> spinv;
  // ERROR_GRAPH("Node has id");

  auto node_se3 = dynamic_cast<g2o::VertexSE3 *>(_node->getVertex());
  if (node_se3) {
    graph_->computeMarginals(spinv, node_se3);
  }
  auto node_point3d = dynamic_cast<g2o::VertexPointXYZ *>(_node->getVertex());
  if (node_point3d) {
    graph_->computeMarginals(spinv, node_point3d);
  }
  // WARN_GRAPH("COVARIANCE\n" << spinv);

  // ERROR_GRAPH("Computer Marginals");
  if (spinv.nonZeroBlocks() < 1) {
    // std::cout << spinv.nonZeros() << std::endl;
    // WARN_GRAPH("No covariance block found for node ID " << node_id);
    return Eigen::MatrixXd();     // Return an empty matrix
  }

  // ERROR_GRAPH("Covariance block found");
  // for (size_t c = 0; c < spinv.blockCols().size(); ++c) {
  //   for (auto it = spinv.blockCols()[c].begin(); it != spinv.blockCols()[c].end(); ++it) {
  //     int row = it->first;
  //     // ERROR_GRAPH("Covariance block available at (" << row << ", " << c << ")");
  //   }
  // }

  // ERROR_GRAPH(PRINT_VAR(node_id));

  Eigen::MatrixXd covariance;
  auto block_cols = spinv.blockCols();
  // ERROR_GRAPH("Covariance block cols size: " << block_cols.size());
  if (node_id - 1 >= block_cols.size()) {
    ERROR_GRAPH("Node ID " << node_id << " is out of bounds for block_cols");
    return Eigen::MatrixXd();     // Return an empty matrix
  }
  auto it = block_cols[node_id - 1].find(node_id - 1);
  // ERROR_GRAPH("Covariance block found for node ID " << node_id);
  // std::cout << "CHECK IT" << (block_cols[node_id - 1].end() == block_cols[node_id-1].end()) << std::endl;
  if (it != block_cols[node_id-1].end()) {
    covariance = *it->second;
  }
  // if (it != block_cols[node_id].end()) {
  //   covariance = *it->second;
  // }
  // auto* block = spinv.block(node_id, node_id);
  // if (block) {
  //   covariance = *block;
  //   // ERROR_GRAPH("Covariance block successfully retrieved");
  // } else {
  //   ERROR_GRAPH("Covariance block not found for node ID " << node_id);
  //   return Eigen::MatrixXd();     // Return an empty matrix
  // }
  // if (spinv.block(node_id, node_id, false)) {
  //   covariance = *spinv.block(node_id, node_id, false);
  //   // ERROR_GRAPH("Covariance block successfully retrieved for node ID " << node_id);
  // } else {
  //   ERROR_GRAPH("Covariance block NOT found for node ID " << node_id);
  //   return Eigen::MatrixXd();     // Return an empty matrix
  // }

  return covariance;
}
