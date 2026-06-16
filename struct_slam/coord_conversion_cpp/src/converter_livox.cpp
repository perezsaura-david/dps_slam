#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "msg_interfaces/msg/lidar_data.hpp"

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "elevation_grid_filter.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

class converter : public rclcpp::Node
{
public:
  converter()
  : Node("converter"), packet_counter_(0)
  {
    this->declare_parameter("height_filter_flag", "normal"); // normal / elevation_grid_filter
    // Height filtering parameters
    this->declare_parameter("MIN_Z", 0.0); // mm
    this->declare_parameter("MAX_Z", 2000.0); // mm

    this->declare_parameter("GRID_SIZE", 100.0); // mm
    this->declare_parameter("Z_DIFF_THRESHOLD", 50.0); // mm
    this->declare_parameter("POINT_COUNT_THRESHOLD", 10); // Minimum number of points in a cell to be considered valid
    
    this->declare_parameter("scale_to_mm", 1000.0);
    // Each 10 packets will be published as one frame
    this->declare_parameter("accumulate_packets", 10); 
    this->declare_parameter("publish_pointcloud", false);
    
    this->height_filter_flag_ = this->get_parameter("height_filter_flag").as_string();
    this->min_z_ = static_cast<float>(this->get_parameter("MIN_Z").as_double());
    this->max_z_ = static_cast<float>(this->get_parameter("MAX_Z").as_double());
    this->grid_size_ = static_cast<float>(this->get_parameter("GRID_SIZE").as_double());
    this->z_diff_threshold_ = static_cast<float>(this->get_parameter("Z_DIFF_THRESHOLD").as_double());
    this->point_count_threshold_ = this->get_parameter("POINT_COUNT_THRESHOLD").as_int();
    this->scale_to_mm_ = static_cast<float>(this->get_parameter("scale_to_mm").as_double());
    this->accumulate_packets_ = this->get_parameter("accumulate_packets").as_int();
    this->publish_pointcloud_ = this->get_parameter("publish_pointcloud").as_bool();

    this->subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar", rclcpp::SensorDataQoS(), std::bind(&converter::topic_callback, this, _1));

    this->publisher_ = this->create_publisher<msg_interfaces::msg::LidarData>("lidar_data", 10);

    if (publish_pointcloud_) {
      this->publisher_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_points", 10);
    }
    
    if (height_filter_flag_ == "elevation_grid_filter") {
      grid_filter_ = std::make_unique<elevation_grid_filter::ElevationGridFilter>(grid_size_, z_diff_threshold_, point_count_threshold_);
    }
    
    raw_x_.reserve(300000);
    raw_y_.reserve(300000);
    if (height_filter_flag_ == "elevation_grid_filter" || publish_pointcloud_) {
      raw_z_.reserve(300000);
    }

    RCLCPP_INFO(this->get_logger(), "Livox Raw CustomMsg Converter initialized.");
    RCLCPP_INFO(this->get_logger(), "Accumulating %d packets per frame.", accumulate_packets_);
  }

private:
  std::string height_filter_flag_;
  float min_z_; // mm
  float max_z_;
  float grid_size_; // mm
  float z_diff_threshold_; // mm
  int point_count_threshold_;
  float scale_to_mm_;
  int accumulate_packets_;
  int packet_counter_;
  bool publish_pointcloud_;

  std::vector<float> raw_x_;
  std::vector<float> raw_y_;
  std::vector<float> raw_z_;
  std::vector<float> filtered_x_;
  std::vector<float> filtered_y_;
  std::vector<float> filtered_z_;

  std::unique_ptr<elevation_grid_filter::ElevationGridFilter> grid_filter_;

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<msg_interfaces::msg::LidarData>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_cloud_;

  void topic_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    for (const auto& point : msg->points) {
        if (point.z * scale_to_mm_ >= min_z_ && point.z * scale_to_mm_ <= max_z_) {
            raw_x_.push_back(point.x * scale_to_mm_);
            raw_y_.push_back(point.y * scale_to_mm_);
            if (height_filter_flag_ == "elevation_grid_filter" || publish_pointcloud_) {
              raw_z_.push_back(point.z * scale_to_mm_);
            }
        }
    }

    packet_counter_++;

    if (packet_counter_ >= accumulate_packets_) {
      if (height_filter_flag_ == "elevation_grid_filter") {
        grid_filter_->filter(raw_x_, raw_y_, raw_z_, filtered_x_, filtered_y_, filtered_z_);
      } else {
        filtered_x_ = raw_x_;
        filtered_y_ = raw_y_;
        filtered_z_ = raw_z_;
      }

      auto message = std::make_unique<msg_interfaces::msg::LidarData>();
      
      message->header = msg->header; // Frame name: livox_frame
      message->x_data = filtered_x_;
      message->y_data = filtered_y_;
      this->publisher_->publish(std::move(message));

      if (publish_pointcloud_)
      {
        auto pc2_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

        pc2_msg->header = msg->header; // Frame name: livox_frame
        // pc2_msg->header.stamp = this->get_clock()->now();
        // pc2_msg->header.frame_id = "spheric_frame";
        pc2_msg->height = 1;
        pc2_msg->width = filtered_z_.size();

        sensor_msgs::PointCloud2Modifier modifier(*pc2_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(filtered_z_.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2_msg, "z");

        for (size_t i = 0; i < filtered_z_.size(); ++i)
        {
          *iter_x = filtered_x_[i] * 0.001f;
          *iter_y = filtered_y_[i] * 0.001f;
          *iter_z = filtered_z_[i] * 0.001f;
          ++iter_x;
          ++iter_y;
          ++iter_z;
        }

        this->publisher_cloud_->publish(std::move(pc2_msg));
      }

      packet_counter_ = 0;
      raw_x_.clear();
      raw_y_.clear();
      raw_z_.clear();
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<converter>());
  rclcpp::shutdown();
  return 0;
}