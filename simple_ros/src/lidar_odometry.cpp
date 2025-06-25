#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
    odomMessage_.header.frame_id = this->declare_parameter<std::string>("odom_frame", "odom");

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

    newScan.processPointCloud();

    double roll = 0, pitch = 0, yaw = 0;

    if (initialised_) {
      scanToMapRegister_->registerScan(newScan.getPtCloud(), subMap_->getPcForKdTree());
      column_vector res = scanToMapRegister_->getRegResult();

      double time_passed = (msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9) - (odomMessage_.header.stamp.sec + odomMessage_.header.stamp.nanosec / 1e9);
      odomMessage_.pose.pose.position.x = res(3);
      odomMessage_.pose.pose.position.y = res(4);
      odomMessage_.pose.pose.position.z = res(5);

      roll = res(0);
      pitch = res(1);
      yaw = res(2);

      tf2::Quaternion q;
      q.setRPY(res(0), res(1), res(2));
      odomMessage_.pose.pose.orientation = tf2::toMsg(q);
    }

    Eigen::Matrix4d hypothesis = utils::homogeneous(roll, pitch, yaw, 
        odomMessage_.pose.pose.position.x, 
        odomMessage_.pose.pose.position.y, 
        odomMessage_.pose.pose.position.z);

    subMap_->updateMap(newScan.getPtCloud(), hypothesis);

    odomMessage_.header.stamp = msg->header.stamp;
    odomMessage_.child_frame_id = msg->header.frame_id;

    this->pubOdom_->publish(odomMessage_);

    initialised_ = true;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPointcloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom_;
  nav_msgs::msg::Odometry odomMessage_;

  float rNew_, rMin_, rMax_;
  bool initialised_ = false;

  std::unique_ptr<PointMap> subMap_;
  std::unique_ptr<Register> scanToMapRegister_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometry>());
  rclcpp::shutdown();
  return 0;
}
