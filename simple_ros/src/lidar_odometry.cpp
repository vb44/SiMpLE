#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "PointCloud.hpp"
#include "PointMap.hpp"
#include "Register.hpp"

using std::placeholders::_1;

class LidarOdometry : public rclcpp::Node {
public:
  LidarOdometry() : Node("SiMpLE_lidar_odometry") {
    subPointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_cloud", 10, std::bind(&LidarOdometry::pointcloudCallback, this, _1));
    pubOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("output_odom", 10);

    rNew_ = this->declare_parameter<float>("rNew", 0.5);
    float rMap = this->declare_parameter<float>("rMap", 2.0);
    rMin_ = this->declare_parameter<float>("rMin", 5.0);
    rMax_ = this->declare_parameter<float>("rMax", 120);
    float sigma = this->declare_parameter<float>("sigma", 0.3);
    float epsilon = this->declare_parameter<float>("epsilon", 1e-3);

    subMap_ = std::make_unique<PointMap>(rMap, rMax_);
    scanToMapRegister_ = std::make_unique<Register>(epsilon, sigma);
  }

private:
  void pointcloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    PointCloud newScan = PointCloud(rNew_, rMax_, rMin_, false);

    sensor_msgs::PointCloud2Iterator<float> iterX(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(*msg, "z");

    for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ) {
      newScan.addPoint(*iterX, *iterY, *iterZ);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPointcloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom_;
  float rNew_, rMin_, rMax_;

  std::unique_ptr<PointMap> subMap_;
  std::unique_ptr<Register> scanToMapRegister_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometry>());
  rclcpp::shutdown();
  return 0;
}
