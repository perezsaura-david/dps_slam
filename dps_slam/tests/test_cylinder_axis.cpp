// Lowest-level checks for the cylinder-axis landmark (VertexCylinderAxis +
// EdgeSE3CylinderAxis), with no ROS involved: build a tiny g2o graph out of
// known ground truth, optimize, and require the landmark to be recovered.
//
// Run directly after building with tests:
//   colcon build --packages-select dps_slam --cmake-args -DBUILD_TESTING=ON
//   ./build/dps_slam/dps_slam_test

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <memory>
#include <random>
#include <vector>

#include "g2o/g2o_edge_types.hpp"

namespace
{

std::unique_ptr<g2o::SparseOptimizer> makeOptimizer()
{
  auto optimizer = std::make_unique<g2o::SparseOptimizer>();
  using BlockSolverX = g2o::BlockSolverX;
  auto linear_solver =
    std::make_unique<g2o::LinearSolverEigen<BlockSolverX::PoseMatrixType>>();
  auto block_solver = std::make_unique<BlockSolverX>(std::move(linear_solver));
  optimizer->setAlgorithm(
    new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver)));
  optimizer->setVerbose(false);
  return optimizer;
}

Eigen::Isometry3d makePose(double x, double y, double z, double yaw)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() << x, y, z;
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

// Measurement of a world cylinder (anchor + unit direction) seen from `pose`.
Eigen::Matrix<double, 6, 1> measureCylinder(
  const Eigen::Isometry3d & pose, const Eigen::Vector3d & anchor_w,
  const Eigen::Vector3d & dir_w)
{
  Eigen::Matrix<double, 6, 1> meas;
  meas.head<3>() = pose.inverse() * anchor_w;
  meas.tail<3>() = (pose.rotation().transpose() * dir_w).normalized();
  return meas;
}

}  // namespace

TEST(VertexCylinderAxis, OplusKeepsDirectionUnitAndMovesAnchor)
{
  g2o_custom::VertexCylinderAxis v;
  Eigen::Matrix<double, 6, 1> state;
  state << 1.0, 2.0, 3.0, 0.0, 0.0, 1.0;
  v.setEstimate(state);

  const double update[5] = {0.1, -0.2, 0.3, 0.05, -0.02};
  v.oplus(update);

  EXPECT_NEAR(v.anchor().x(), 1.1, 1e-12);
  EXPECT_NEAR(v.anchor().y(), 1.8, 1e-12);
  EXPECT_NEAR(v.anchor().z(), 3.3, 1e-12);
  EXPECT_NEAR(v.direction().norm(), 1.0, 1e-12);
  // A nonzero tangent update must actually tilt the direction.
  EXPECT_GT((v.direction() - Eigen::Vector3d::UnitZ()).norm(), 1e-3);
}

TEST(EdgeSE3CylinderAxis, ZeroErrorAtGroundTruth)
{
  const Eigen::Vector3d anchor_w(1.5, -1.5, 2.0);
  const Eigen::Vector3d dir_w = Eigen::Vector3d(0.1, -0.05, 1.0).normalized();
  const Eigen::Isometry3d pose = makePose(0.5, -3.0, 1.0, 0.7);

  g2o::VertexSE3 v_pose;
  v_pose.setEstimate(pose);
  g2o_custom::VertexCylinderAxis v_cyl;
  Eigen::Matrix<double, 6, 1> state;
  state.head<3>() = anchor_w;
  state.tail<3>() = dir_w;
  v_cyl.setEstimate(state);

  g2o_custom::EdgeSE3CylinderAxis edge;
  edge.vertices()[0] = &v_pose;
  edge.vertices()[1] = &v_cyl;
  edge.setMeasurement(measureCylinder(pose, anchor_w, dir_w));
  edge.computeError();

  for (int i = 0; i < 5; ++i) {
    EXPECT_NEAR(edge.error()[i], 0.0, 1e-12) << "error component " << i;
  }
}

TEST(EdgeSE3CylinderAxis, RecoversCylinderFromFixedPosesExactMeasurements)
{
  const Eigen::Vector3d anchor_gt(1.5144, -1.4688, 2.0);
  const Eigen::Vector3d dir_gt = Eigen::Vector3d(0.05, 0.02, 1.0).normalized();

  auto optimizer = makeOptimizer();

  const std::vector<Eigen::Isometry3d> poses = {
    makePose(0.0, -3.0, 1.0, 0.0),
    makePose(2.0, -2.0, 1.2, 1.2),
    makePose(3.0, 0.0, 0.8, 2.1),
  };
  int id = 0;
  std::vector<g2o::VertexSE3 *> pose_vertices;
  for (const auto & pose : poses) {
    auto * v = new g2o::VertexSE3();
    v->setId(id++);
    v->setEstimate(pose);
    v->setFixed(true);
    optimizer->addVertex(v);
    pose_vertices.push_back(v);
  }

  auto * v_cyl = new g2o_custom::VertexCylinderAxis();
  v_cyl->setId(id++);
  Eigen::Matrix<double, 6, 1> init;  // deliberately off ground truth
  init << 0.5, -0.5, 1.0, 0.3, -0.3, 1.0;
  init.tail<3>().normalize();
  v_cyl->setEstimate(init);
  optimizer->addVertex(v_cyl);

  for (auto * v_pose : pose_vertices) {
    auto * edge = new g2o_custom::EdgeSE3CylinderAxis();
    edge->vertices()[0] = v_pose;
    edge->vertices()[1] = v_cyl;
    edge->setMeasurement(
      measureCylinder(v_pose->estimate(), anchor_gt, dir_gt));
    edge->setInformation(Eigen::Matrix<double, 5, 5>::Identity());
    optimizer->addEdge(edge);
  }

  optimizer->initializeOptimization();
  optimizer->optimize(50);

  EXPECT_LT((v_cyl->anchor() - anchor_gt).norm(), 1e-6);
  EXPECT_LT((v_cyl->direction() - dir_gt).norm(), 1e-6);
}

TEST(EdgeSE3CylinderAxis, RecoversCylinderUnderNoiseToSubNoiseAccuracy)
{
  const Eigen::Vector3d anchor_gt(-0.4554, 0.5237, 2.0);
  const Eigen::Vector3d dir_gt = Eigen::Vector3d::UnitZ();
  const double pos_std = 0.03;
  const double dir_std = 0.01;
  const int n_obs = 60;

  auto optimizer = makeOptimizer();
  std::mt19937 rng(42);
  std::normal_distribution<double> pos_noise(0.0, pos_std);
  std::normal_distribution<double> dir_noise(0.0, dir_std);

  auto * v_cyl = new g2o_custom::VertexCylinderAxis();
  v_cyl->setId(0);
  Eigen::Matrix<double, 6, 1> init;
  init << 0.0, 0.0, 1.0, 0.0, 0.0, 1.0;
  v_cyl->setEstimate(init);
  optimizer->addVertex(v_cyl);

  Eigen::Matrix<double, 5, 5> information = Eigen::Matrix<double, 5, 5>::Identity();
  information.block<3, 3>(0, 0) /= pos_std * pos_std;
  information.block<2, 2>(3, 3) /= dir_std * dir_std;

  int id = 1;
  for (int k = 0; k < n_obs; ++k) {
    const double ang = 2.0 * M_PI * k / n_obs;
    auto * v_pose = new g2o::VertexSE3();
    v_pose->setId(id++);
    v_pose->setEstimate(
      makePose(3.0 * std::cos(ang), 3.0 * std::sin(ang), 1.0, ang));
    v_pose->setFixed(true);
    optimizer->addVertex(v_pose);

    Eigen::Matrix<double, 6, 1> meas =
      measureCylinder(v_pose->estimate(), anchor_gt, dir_gt);
    meas[0] += pos_noise(rng);
    meas[1] += pos_noise(rng);
    meas[2] += pos_noise(rng);
    meas[3] += dir_noise(rng);
    meas[4] += dir_noise(rng);
    meas.tail<3>().normalize();

    auto * edge = new g2o_custom::EdgeSE3CylinderAxis();
    edge->vertices()[0] = v_pose;
    edge->vertices()[1] = v_cyl;
    edge->setMeasurement(meas);
    edge->setInformation(information);
    optimizer->addEdge(edge);
  }

  optimizer->initializeOptimization();
  optimizer->optimize(100);

  // With N observations the fused estimate should be well under one
  // single-measurement sigma (~sigma/sqrt(N) in the ideal case).
  EXPECT_LT((v_cyl->anchor() - anchor_gt).norm(), pos_std);
  EXPECT_LT((v_cyl->anchor() - anchor_gt).norm(), 3.0 * pos_std / std::sqrt(n_obs));
  EXPECT_LT(std::acos(std::min(1.0, v_cyl->direction().dot(dir_gt))),
    3.0 * dir_std / std::sqrt(n_obs));
}
