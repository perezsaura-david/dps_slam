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
 *  \file       edge_types.hpp
 *  \brief      Custom g2o edges for SemanticSlam
 *  \authors    David Pérez Saura
 *              Miguel Fernández Cortizas
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef G2O__EDGE_TYPES_HPP_
#define G2O__EDGE_TYPES_HPP_

#include <Eigen/Core>
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"

// Function to compute the skew-symmetric matrix of a 3D vector
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d & v);

namespace g2o_custom
{

class EdgeSE3Point3D : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSE3,
    g2o::VertexPointXYZ>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3Point3D() {}

  static EdgeSE3Point3D * create()
  {
    return new EdgeSE3Point3D();
  }

  void computeError() override
  {
    const g2o::VertexSE3 * se3 = static_cast<const g2o::VertexSE3 *>(_vertices[0]);
    const g2o::VertexPointXYZ * point = static_cast<const g2o::VertexPointXYZ *>(_vertices[1]);

    // Transform the point from the global frame to the SE3 frame
    Eigen::Vector3d transformedPoint = se3->estimate().inverse() * point->estimate();

    // Compute the error as the difference between the transformed point and the measurement
    _error = transformedPoint - _measurement;
    std::cout << "Edge: " << _id << std::endl;
    std::cout << "SE3 translation" << std::endl;
    std::cout << se3->estimate().translation().transpose() << std::endl;
    std::cout << "SE3 rotation" << std::endl;
    std::cout << se3->estimate().rotation().transpose() << std::endl;
    std::cout << "Point translation" << std::endl;
    std::cout << point->estimate().transpose() << std::endl;
    std::cout << "Computed error" << std::endl;
    std::cout << _error.transpose() << std::endl;
    std::cout << "---" << std::endl;
  }

  void linearizeOplus() override
  {
    const g2o::VertexSE3 * se3 = static_cast<const g2o::VertexSE3 *>(_vertices[0]);
    const g2o::VertexPointXYZ * point = static_cast<const g2o::VertexPointXYZ *>(_vertices[1]);

    Eigen::Matrix3d Ri = se3->estimate().rotation().transpose();
    Eigen::Vector3d transformedPoint = se3->estimate().inverse() * point->estimate();

    Eigen::Matrix<double, 3, 6> jacobian_pose;
    jacobian_pose.block<3, 3>(0, 0) = Ri * skewSymmetric(transformedPoint);
    jacobian_pose.block<3, 3>(0, 3) = -Ri;

    Eigen::Matrix<double, 3, 3> jacobian_point = -Ri;

    _jacobianOplusXi = jacobian_pose;
    _jacobianOplusXj = jacobian_point;
  }


  // Read method for deserialization
  bool read(std::istream & is) override
  {
    // Read the measurement (3D vector) from the input stream
    for (int i = 0; i < 3; ++i) {
      is >> _measurement[i];
    }
    return true;
  }

  // Write method for serialization
  bool write(std::ostream & os) const override
  {
    // Write the measurement (3D vector) to the output stream
    for (int i = 0; i < 3; ++i) {
      os << _measurement[i] << " ";
    }
    return os.good();
  }
};

}  // namespace g2o_custom

#endif  // G2O__EDGE_TYPES_HPP_
