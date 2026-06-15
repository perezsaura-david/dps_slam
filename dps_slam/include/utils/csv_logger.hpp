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

#ifndef UTILS__CSV_LOGGER_HPP_
#define UTILS__CSV_LOGGER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdio>
#include <string>

class CsvLogger
{
public:
  explicit CsvLogger(const std::string & output_dir = ".")
  {
    odom_file_ = fopen((output_dir + "/slam_odom.csv").c_str(), "w");
    keyframe_file_ = fopen((output_dir + "/slam_keyframes.csv").c_str(), "w");
    detection_file_ = fopen((output_dir + "/slam_detections.csv").c_str(), "w");
    merge_file_ = fopen((output_dir + "/slam_merges.csv").c_str(), "w");
    fixed_file_ = fopen((output_dir + "/slam_fixed_objects.csv").c_str(), "w");

    if (odom_file_) {
      fprintf(odom_file_,
        "sec,nsec,raw_x,raw_y,raw_z,raw_qx,raw_qy,raw_qz,raw_qw,"
        "cov_xx,cov_yy,cov_zz,cov_rr,cov_pp,cov_yy2,"
        "cor_x,cor_y,cor_z,cor_qx,cor_qy,cor_qz,cor_qw\n");
    }
    if (keyframe_file_) {
      fprintf(keyframe_file_,
        "sec,nsec,raw_x,raw_y,raw_z,raw_qx,raw_qy,raw_qz,raw_qw,"
        "main_nodes,main_edges,chi2_before,chi2_after,opt_time_ms,"
        "tf_x,tf_y,tf_z,tf_qx,tf_qy,tf_qz,tf_qw,"
        "opt_x,opt_y,opt_z,cor_x,cor_y,cor_z\n");
    }
    if (detection_file_) {
      fprintf(detection_file_,
        "sec,nsec,id,type,body_x,body_y,body_z,"
        "odom_x,odom_y,odom_z,is_absolute\n");
    }
    if (merge_file_) {
      fprintf(merge_file_,
        "sec,nsec,id,temp_x,temp_y,temp_z,"
        "odom_x,odom_y,odom_z,"
        "temp_nodes,temp_edges,temp_chi2\n");
    }
    if (fixed_file_) {
      fprintf(fixed_file_, "id,type,earth_x,earth_y,earth_z,map_x,map_y,map_z\n");
    }
  }

  ~CsvLogger()
  {
    if (odom_file_) { fclose(odom_file_); }
    if (keyframe_file_) { fclose(keyframe_file_); }
    if (detection_file_) { fclose(detection_file_); }
    if (merge_file_) { fclose(merge_file_); }
    if (fixed_file_) { fclose(fixed_file_); }
  }

  CsvLogger(const CsvLogger &) = delete;
  CsvLogger & operator=(const CsvLogger &) = delete;

  void logOdom(
    int64_t sec, uint32_t nsec,
    const Eigen::Isometry3d & raw,
    const Eigen::MatrixXd & cov,
    const Eigen::Isometry3d & corrected)
  {
    if (!odom_file_) { return; }
    if (++odom_counter_ % ODOM_LOG_EVERY_N != 0) { return; }

    Eigen::Quaterniond rq(raw.rotation());
    Eigen::Quaterniond cq(corrected.rotation());
    auto rt = raw.translation();
    auto ct = corrected.translation();

    fprintf(odom_file_,
      "%ld,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
      "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,"
      "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
      sec, nsec,
      rt.x(), rt.y(), rt.z(), rq.x(), rq.y(), rq.z(), rq.w(),
      cov(0, 0), cov(1, 1), cov(2, 2), cov(3, 3), cov(4, 4), cov(5, 5),
      ct.x(), ct.y(), ct.z(), cq.x(), cq.y(), cq.z(), cq.w());
  }

  void logKeyframe(
    int64_t sec, uint32_t nsec,
    const Eigen::Isometry3d & raw,
    int main_nodes, int main_edges,
    double chi2_before, double chi2_after, double opt_time_ms,
    const Eigen::Isometry3d & map_odom_tf,
    const Eigen::Isometry3d & optimized,
    const Eigen::Isometry3d & corrected)
  {
    if (!keyframe_file_) { return; }

    Eigen::Quaterniond rq(raw.rotation());
    Eigen::Quaterniond tq(map_odom_tf.rotation());
    auto rt = raw.translation();
    auto tt = map_odom_tf.translation();
    auto ot = optimized.translation();
    auto ct = corrected.translation();

    fprintf(keyframe_file_,
      "%ld,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
      "%d,%d,%.4f,%.4f,%.2f,"
      "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
      "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
      sec, nsec,
      rt.x(), rt.y(), rt.z(), rq.x(), rq.y(), rq.z(), rq.w(),
      main_nodes, main_edges, chi2_before, chi2_after, opt_time_ms,
      tt.x(), tt.y(), tt.z(), tq.x(), tq.y(), tq.z(), tq.w(),
      ot.x(), ot.y(), ot.z(), ct.x(), ct.y(), ct.z());
  }

  void logDetection(
    int64_t sec, uint32_t nsec,
    const std::string & id, const std::string & type,
    const Eigen::Vector3d & body_pos,
    const Eigen::Vector3d & odom_pos,
    bool is_absolute)
  {
    if (!detection_file_) { return; }
    fprintf(detection_file_,
      "%ld,%u,%s,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
      sec, nsec, id.c_str(), type.c_str(),
      body_pos.x(), body_pos.y(), body_pos.z(),
      odom_pos.x(), odom_pos.y(), odom_pos.z(),
      is_absolute ? 1 : 0);
  }

  void logMerge(
    int64_t sec, uint32_t nsec,
    const std::string & id,
    const Eigen::Vector3d & temp_pos,
    const Eigen::Vector3d & odom_pos,
    int temp_nodes, int temp_edges, double temp_chi2)
  {
    if (!merge_file_) { return; }
    fprintf(merge_file_,
      "%ld,%u,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.4f\n",
      sec, nsec, id.c_str(),
      temp_pos.x(), temp_pos.y(), temp_pos.z(),
      odom_pos.x(), odom_pos.y(), odom_pos.z(),
      temp_nodes, temp_edges, temp_chi2);
  }

  void logFixedObject(
    const std::string & id, const std::string & type,
    const Eigen::Vector3d & earth_pos,
    const Eigen::Vector3d & map_pos)
  {
    if (!fixed_file_) { return; }
    fprintf(fixed_file_,
      "%s,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
      id.c_str(), type.c_str(),
      earth_pos.x(), earth_pos.y(), earth_pos.z(),
      map_pos.x(), map_pos.y(), map_pos.z());
  }

private:
  FILE * odom_file_ = nullptr;
  FILE * keyframe_file_ = nullptr;
  FILE * detection_file_ = nullptr;
  FILE * merge_file_ = nullptr;
  FILE * fixed_file_ = nullptr;
  int odom_counter_ = 0;
  static constexpr int ODOM_LOG_EVERY_N = 10;
};

#endif  // UTILS__CSV_LOGGER_HPP_
