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
 * Debug utils implementation for semantic slam
 *
 * @author David Pérez Saura
 *         Miguel Fernández Cortizas
 */

#include "utils/debug_utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>

void debugGraphVertices(std::shared_ptr<GraphG2O> _graph)
{
  FLAG("Debugging vertices in " << _graph->getName());
  for (auto p : _graph->graph_->vertices()) {
    int id = p.first;

    g2o::SparseBlockMatrix<Eigen::MatrixXd> spinv;

    auto node_se3 = dynamic_cast<g2o::VertexSE3 *>(p.second);
    if (node_se3) {
      auto T = node_se3->estimate().translation().transpose();
      auto R = node_se3->estimate().rotation();
      INFO("NODE SE3 [" << id << "]: " << T);
      INFO("NODE SE3 [" << id << "]: " << R);
      _graph->graph_->computeMarginals(spinv, node_se3);
      // debugComputeCovariance(spinv, id);
      continue;
    }
    auto node_point3d = dynamic_cast<g2o::VertexPointXYZ *>(p.second);
    if (node_point3d) {
      auto T = node_point3d->estimate().transpose();
      INFO("NODE POINT3D [" << id << "]: " << T);
      _graph->graph_->computeMarginals(spinv, node_point3d);
      // debugComputeCovariance(spinv, id);
      continue;
    }
    WARN("Node [" << id << "] type is not recognized");
    // continue;
    // Get the block corresponding to this node
    // // Access the covariance for a particular node
    // auto block_cols = spinv.blockCols();
    // auto it = block_cols[id - 1].find(id - 1);
    // if (it != block_cols[id].end()) {
    //   Eigen::MatrixXd * covariance = it->second;
    //   if (covariance) {
    //     FLAG("Covariance block for node " << id << ":\n" << *covariance);
    //   } else {
    //     WARN("Covariance block for node " << id << " is nullptr.");
    //   }
    // } else {
    //   INFO("No covariance block found for node " << id);
    // }
  }

}

void debugComputeCovariance(const g2o::SparseBlockMatrix<Eigen::MatrixXd> & _spinv, int _node_id)
{
  std::cout << "covariance\n" << _spinv << std::endl;

  if (_spinv.nonZeroBlocks() < 1) {
    // std::cout << spinv.nonZeros() << std::endl;
    INFO("Empty block");
    return;
  }
  Eigen::MatrixXd covariance = *(_spinv.block(_node_id - 1, _node_id - 1));
  FLAG("Covariance block for node " << _node_id << ":\n" << covariance);
}
