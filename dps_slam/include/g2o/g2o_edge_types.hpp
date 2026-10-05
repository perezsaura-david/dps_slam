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
#include <Eigen/Geometry>
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include <cmath>
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d_addons/vertex_plane.h"
#include "g2o/types/slam3d_addons/plane3d.h"

// Function to compute the skew-symmetric matrix of a 3D vector
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d & v);

namespace g2o_custom
{

// Orthonormal basis (b1, b2) of the tangent plane at unit direction d, so that
// (d, b1, b2) is a right-handed frame. Used both for the minimal 2-DOF update
// of a cylinder axis direction and for projecting direction residuals.
inline void directionTangentBasis(
  const Eigen::Vector3d & d, Eigen::Vector3d & b1, Eigen::Vector3d & b2)
{
  const Eigen::Vector3d ref = (std::abs(d.z()) < 0.9)
    ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
  b1 = d.cross(ref).normalized();
  b2 = d.cross(b1).normalized();
}

// Cylinder-axis landmark: a 5-DOF state [anchor(3); direction(3, unit)].
// The anchor is a distinguished point ON the axis (the cylinder TOP in this
// pipeline -- finite cylinders make the along-axis position observable, unlike
// an infinite Line3D), and the direction is the axis unit vector. The internal
// estimate stores 6 numbers but the vertex exposes the minimal 5 DOF: 3 for the
// anchor plus a 2-DOF tangent-plane update that keeps the direction on S^2.
class VertexCylinderAxis : public g2o::BaseVertex<5, Eigen::Matrix<double, 6, 1>>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexCylinderAxis() {}

  void setToOriginImpl() override
  {
    _estimate.setZero();
    _estimate[5] = 1.0;  // direction = +Z
  }

  void oplusImpl(const double * update) override
  {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] += update[2];
    Eigen::Vector3d d = _estimate.tail<3>().normalized();
    Eigen::Vector3d b1, b2;
    directionTangentBasis(d, b1, b2);
    _estimate.tail<3>() = (d + b1 * update[3] + b2 * update[4]).normalized();
  }

  Eigen::Vector3d anchor() const {return _estimate.head<3>();}
  Eigen::Vector3d direction() const {return _estimate.tail<3>().normalized();}

  bool read(std::istream & is) override
  {
    for (int i = 0; i < 6; ++i) {is >> _estimate[i];}
    return true;
  }

  bool write(std::ostream & os) const override
  {
    for (int i = 0; i < 6; ++i) {os << _estimate[i] << " ";}
    return os.good();
  }
};

// Binary edge constraining a robot SE3 pose to a cylinder-axis landmark.
// Measurement: [anchor(3); direction(3, unit)] observed in the robot frame.
// Error (5-DOF): the world landmark predicted in the robot frame minus the
// measurement -- 3 anchor residuals plus the direction residual projected onto
// the 2-DOF tangent plane of the measured direction. Jacobians are numeric
// (same approach as the plane edge above).
class EdgeSE3CylinderAxis : public g2o::BaseBinaryEdge<5, Eigen::Matrix<double, 6, 1>,
    g2o::VertexSE3, VertexCylinderAxis>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3CylinderAxis() {}

  void computeError() override
  {
    const g2o::VertexSE3 * se3 = static_cast<const g2o::VertexSE3 *>(_vertices[0]);
    const VertexCylinderAxis * cyl = static_cast<const VertexCylinderAxis *>(_vertices[1]);

    const Eigen::Isometry3d T_inv = se3->estimate().inverse();
    const Eigen::Vector3d anchor_robot = T_inv * cyl->anchor();
    const Eigen::Vector3d dir_robot = (T_inv.rotation() * cyl->direction()).normalized();

    const Eigen::Vector3d meas_anchor = _measurement.head<3>();
    const Eigen::Vector3d meas_dir = _measurement.tail<3>().normalized();

    Eigen::Vector3d b1, b2;
    directionTangentBasis(meas_dir, b1, b2);
    const Eigen::Vector3d dir_diff = dir_robot - meas_dir;

    _error.head<3>() = anchor_robot - meas_anchor;
    _error[3] = b1.dot(dir_diff);
    _error[4] = b2.dot(dir_diff);
  }

  bool read(std::istream & is) override
  {
    for (int i = 0; i < 6; ++i) {is >> _measurement[i];}
    return true;
  }

  bool write(std::ostream & os) const override
  {
    for (int i = 0; i < 6; ++i) {os << _measurement[i] << " ";}
    return os.good();
  }
};

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

// Binary edge constraining a robot SE3 pose to a plane landmark.
// The error is the 3-DOF ominus (azimuth, elevation, distance) between the plane
// predicted in the robot frame and the measured plane. Jacobians are numeric.
class EdgeSE3Plane3D : public g2o::BaseBinaryEdge<3, g2o::Plane3D, g2o::VertexSE3,
    g2o::VertexPlane>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3Plane3D() {}

  void computeError() override
  {
    const g2o::VertexSE3 * se3 = static_cast<const g2o::VertexSE3 *>(_vertices[0]);
    const g2o::VertexPlane * plane = static_cast<const g2o::VertexPlane *>(_vertices[1]);

    // Predict the plane in the robot frame and compare with the measurement.
    g2o::Plane3D predicted_plane = se3->estimate().inverse() * plane->estimate();
    _error = predicted_plane.ominus(_measurement);
  }

  bool read(std::istream & is) override
  {
    Eigen::Vector4d v;
    for (int i = 0; i < 4; ++i) {is >> v[i];}
    setMeasurement(g2o::Plane3D(v));
    return true;
  }

  bool write(std::ostream & os) const override
  {
    Eigen::Vector4d v = _measurement.toVector();
    for (int i = 0; i < 4; ++i) {os << v[i] << " ";}
    return os.good();
  }
};

// Unary prior keeping a plane vertical: the error is the elevation of its normal (0 for a wall).
// Wall landmarks come from 2D lines, so they are vertical by construction; without this the
// optimizer can tip one over (seen: a wall drawn 11 m up, its plane turned near horizontal).
class EdgePlaneVertical : public g2o::BaseUnaryEdge<1, double, g2o::VertexPlane>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgePlaneVertical() {setMeasurement(0.0);}

  void computeError() override
  {
    const g2o::VertexPlane * plane = static_cast<const g2o::VertexPlane *>(_vertices[0]);
    const Eigen::Vector3d n = plane->estimate().normal();
    _error[0] = std::atan2(n.z(), n.head<2>().norm()) - _measurement;
  }

  bool read(std::istream & is) override {is >> _measurement; return true;}
  bool write(std::ostream & os) const override {os << _measurement << " "; return os.good();}
};

}  // namespace g2o_custom

#endif  // G2O__EDGE_TYPES_HPP_
