// Copyright 2026 Universidad Politecnica de Madrid (UPM).
//
// Author: Pedro Espinosa Angulo
// Contributor: Guanliang Li, Santiago Tapia Fernandez (supervised)
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


#include <memory>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "msg_interfaces/msg/lidar_data.hpp"
#include <eigen3/Eigen/Dense>
#include <chrono>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class converter : public rclcpp::Node
{
public:
  converter()
  : Node("converter")
  {
    // set & get parameters
    this->declare_parameter("HFOV", 360.0);
    this->declare_parameter("VFOV", 30.0);
    this->declare_parameter("ROWS", 16);
    this->declare_parameter("COLS", 1800);

    // Filter parameters
    this->declare_parameter("MIN_Z", 0.0);
    this->declare_parameter("MAX_Z", 2000.0);
    this->declare_parameter("publish_pointcloud", false);
  
    this->HFOV = static_cast<float>(this->get_parameter("HFOV").as_double());
    this->VFOV = static_cast<float>(this->get_parameter("VFOV").as_double());
    this->ROWS = this->get_parameter("ROWS").as_int();
    this->COLS = this->get_parameter("COLS").as_int();

    this->min_z_ = static_cast<float>(this->get_parameter("MIN_Z").as_double());
    this->max_z_ = static_cast<float>(this->get_parameter("MAX_Z").as_double());
    this->publish_pointcloud_ = this->get_parameter("publish_pointcloud").as_bool();

    this->subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "sensor_scans", 10, std::bind(&converter::topic_callback, this, _1));

    this->publisher_ = this->create_publisher<msg_interfaces::msg::LidarData>("lidar_data", 10);
    if (this->publish_pointcloud_) {
      this->publisher_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_points", 10);
    }

    const float PI_F = static_cast<float>(M_PI);
    const float DEG_TO_RAD = PI_F / 180.0f;

    this->input_matrix.resize(this->ROWS, this->COLS);

    this->vectorElevCos.resize(this->ROWS);
    this->vectorElevSin.resize(this->ROWS);
    this->vectorAzimuthCos.resize(this->COLS);
    this->vectorAzimuthSin.resize(this->COLS);

    // Vertical angles (elevation) 
    float elevation_deg, elevation_rad;
    float elev_step = this->VFOV / static_cast<float>(this->ROWS - 1);
    float elev_start = this->VFOV / 2.0f;
    for (uint32_t i = 0; i < this->ROWS; i++) {
      elevation_deg = elev_start - i * elev_step;
      elevation_rad = elevation_deg * DEG_TO_RAD;
      this->vectorElevCos(i) = std::cos(elevation_rad);
      this->vectorElevSin(i) = std::sin(elevation_rad);
    }

    // Horizontal angles (azimuth) 
    float azimuth_deg, azimuth_rad;
    float azim_step = this->HFOV / static_cast<float>(this->COLS);
    for (uint32_t j = 0; j < this->COLS; j++) {
      azimuth_deg = j * azim_step;
      azimuth_rad = azimuth_deg * DEG_TO_RAD;
      this->vectorAzimuthCos(j) = std::cos(azimuth_rad);
      this->vectorAzimuthSin(j) = std::sin(azimuth_rad);
    }

    // Geometric factors
    this->geomFactorX.resize(this->ROWS, this->COLS);
    this->geomFactorY.resize(this->ROWS, this->COLS);
    this->geomFactorZ.resize(this->ROWS, this->COLS);
    
    this->geomFactorX = this->vectorElevCos * this->vectorAzimuthCos;
    this->geomFactorY = this->vectorElevCos * this->vectorAzimuthSin;
    this->geomFactorZ = this->vectorElevSin.replicate(1, this->COLS);

    // Shape output arrays
    this->ranges_.resize(this->ROWS, this->COLS);
    this->matrixX.resize(this->ROWS, this->COLS);
    this->matrixY.resize(this->ROWS, this->COLS);
    this->matrixZ.resize(this->ROWS, this->COLS);

    // Save space for x, y and z data
    this->xArray.resize(this->ROWS * this->COLS);
    this->yArray.resize(this->ROWS * this->COLS);
    if (this->publish_pointcloud_)
      this->zArray.resize(this->ROWS * this->COLS);
  }

private:
  float HFOV;
  float VFOV;
  uint32_t ROWS;
  uint32_t COLS;
  float min_z_;
  float max_z_;
  bool publish_pointcloud_;

  typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixLidar;
  typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixLidarFloat;

  mutable MatrixLidar input_matrix;
  mutable std::vector<float> xArray;
  mutable std::vector<float> yArray;
  mutable std::vector<float> zArray; // buffer for Z coordinates

  mutable MatrixLidarFloat ranges_;
  mutable MatrixLidarFloat matrixX;
  mutable MatrixLidarFloat matrixY;
  mutable MatrixLidarFloat matrixZ;

  mutable rclcpp::Clock::SharedPtr throttle_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME); // mutable clock for RCLCPP_ERROR_THROTTLE

  Eigen::Matrix<float, Eigen::Dynamic, 1> vectorElevCos;
  Eigen::Matrix<float, Eigen::Dynamic, 1> vectorElevSin;
  Eigen::Matrix<float, 1, Eigen::Dynamic> vectorAzimuthCos;
  Eigen::Matrix<float, 1, Eigen::Dynamic> vectorAzimuthSin;

  MatrixLidarFloat geomFactorX;
  MatrixLidarFloat geomFactorY;
  MatrixLidarFloat geomFactorZ;

  void topic_callback(const sensor_msgs::msg::PointCloud2 & msg) const
  {
    // --- SECURITY CHECK ---
    if (msg.height != this->ROWS || msg.width != this->COLS) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->throttle_clock_, 2000, 
            "Dimension mismatch! Expected %dx%d, got %dx%d. Skipping frame.", 
            this->ROWS, this->COLS, msg.height, msg.width);
        return;
    }

    this->recover_binary_data(msg, this->input_matrix);
    this->ranges_ = this->input_matrix.cast<float>();

    this->matrixX = this->ranges_.cwiseProduct(this->geomFactorX);
    this->matrixY = this->ranges_.cwiseProduct(this->geomFactorY);
    this->matrixZ = this->ranges_.cwiseProduct(this->geomFactorZ);
    
    const float* r_ptr = this->ranges_.data();
    const float* x_ptr = this->matrixX.data();
    const float* y_ptr = this->matrixY.data();
    const float* z_ptr = this->matrixZ.data(); 
    size_t total_points = this->ROWS * this->COLS;

    size_t index = 0;
    const bool publish_cloud = this->publish_pointcloud_;

    for (size_t i = 0; i < total_points; ++i) {
        if (r_ptr[i] > 0.001f) { // filter unhit points
          float z_val = z_ptr[i];
          if (z_val >= this->min_z_ && z_val <= this->max_z_) { // filter by z value
            this->xArray[index] = x_ptr[i];
            this->yArray[index] = y_ptr[i];
            
            if (publish_cloud)
                this->zArray[index] = z_val;
            ++index;
          }
        }
    }

    auto message = std::make_unique<msg_interfaces::msg::LidarData>();
    message->x_data.assign(this->xArray.begin(), this->xArray.begin() + index);
    message->y_data.assign(this->yArray.begin(), this->yArray.begin() + index);

    this->publisher_->publish(std::move(message));

    if (this->publish_pointcloud_) {
      auto pc2_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    
      pc2_msg->header = msg.header;
      pc2_msg->height = 1;
      pc2_msg->width = index;

      sensor_msgs::PointCloud2Modifier modifier(*pc2_msg);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(index);

      sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2_msg, "z");

      for (size_t k = 0; k < index; ++k) {
          *iter_x = this->xArray[k] * 0.001f;
          *iter_y = this->yArray[k] * 0.001f;
          *iter_z = this->zArray[k] * 0.001f;
          ++iter_x; ++iter_y; ++iter_z;
      }

      this->publisher_cloud_->publish(std::move(pc2_msg));
    }
  }

  bool is_system_little_endian() const
  {
    int num = 1;
    return (*(char *)&num == 1);
  }

  void recover_binary_data(const sensor_msgs::msg::PointCloud2 & msg, MatrixLidar & matrix) const
  {
    size_t num_points = this->ROWS * this->COLS;
    uint32_t* matrix_ptr = matrix.data();

    bool system_is_little = this->is_system_little_endian();
    bool msg_is_little = !msg.is_bigendian;

    if (system_is_little == msg_is_little)
      std::memcpy(matrix_ptr, msg.data.data(), num_points * sizeof(uint32_t));
    else {
      const uint32_t * raw_ptr = reinterpret_cast<const uint32_t *>(msg.data.data());
      for (size_t i = 0; i < num_points; ++i) {
        matrix_ptr[i] = __builtin_bswap32(raw_ptr[i]);
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<msg_interfaces::msg::LidarData>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_cloud_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<converter>());
  rclcpp::shutdown();
  return 0;
}
