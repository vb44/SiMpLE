#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using std::placeholders::_1;

class LidarOdometry : public rclcpp::Node {
public:
  LidarOdometry() : Node("SiMpLE_lidar_odometry") {
    subPointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_cloud", 10, std::bind(&LidarOdometry::pointcloudCallback, this, _1));
    pubOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("output_odom", 10);

    float rNew = this->declare_parameter<float>("rNew", 0.5);
    float rMap = this->declare_parameter<float>("rMap", 2.0);
    float rMin = this->declare_parameter<float>("rMin", 5.0);
    float rMax = this->declare_parameter<float>("rMax", 120);
    float sigma = this->declare_parameter<float>("sigma", 0.3);
    float epsilon = this->declare_parameter<float>("epsilon", 1e-3);
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2 & msg) const {
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPointcloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometry>());
  rclcpp::shutdown();
  return 0;
}
