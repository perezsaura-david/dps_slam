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
 *  \file       debug_utils.hpp
 *  \brief      An state estimation server for AeroStack2
 *  \authors    David Pérez Saura
 *              Miguel Fernández Cortizas
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef UTILS__DEBUG_UTILS_HPP_
#define UTILS__DEBUG_UTILS_HPP_

#include <memory>
#include "dual_pose_graph/graph_g2o.hpp"

#define DEBUG_START_TIMER // auto start_time = std::chrono::high_resolution_clock::now();
#define DEBUG_LOG_DURATION_GRAPH // auto end_time = std::chrono::high_resolution_clock::now(); \
  auto duration = \
    std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count(); \
  DEBUG_GRAPH(__func__ << " : " << duration << " ms");
#define DEBUG_LOG_DURATION // auto end_time = std::chrono::high_resolution_clock::now(); \
  auto duration = \
    std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count(); \
  DEBUG(__func__ << " : " << duration << " ms");

void debugGraphVertices(std::shared_ptr<GraphG2O> _graph);
void debugComputeCovariance(const g2o::SparseBlockMatrix<Eigen::MatrixXd> & _spinv, int _node_id);

#endif  // UTILS__DEBUG_UTILS_HPP_
