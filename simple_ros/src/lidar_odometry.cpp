#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>

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
    pubMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_map", 10);

    rNew_ = this->declare_parameter<float>("rNew", 0.5);
    float rMap = this->declare_parameter<float>("rMap", 2.0);
    rMin_ = this->declare_parameter<float>("rMin", 5.0);
    rMax_ = this->declare_parameter<float>("rMax", 120);
    float sigma = this->declare_parameter<float>("sigma", 0.3);
    float epsilon = this->declare_parameter<float>("epsilon", 1e-3);
    // Odometry and TF child_frame_id, uses input_cloud.header.frame_id if not provided.
    // This will transform results by looking up the transform between child_frame and input_cloud.header.frame_id"
    childFrame_ = this->declare_parameter("child_frame", "");
    // Odometry and TF header.frame_id
    odomMessage_.header.frame_id = this->declare_parameter<std::string>("odom_frame", "odom");
    enablePubOdom_ = this->declare_parameter<bool>("publish_odom", true);
    enablePubTF_ = this->declare_parameter<bool>("publish_map", false);
    enablePubMap_ = this->declare_parameter<bool>("publish_map", false);

    if (enablePubOdom_) {
      pubOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("output_odom", 10);
    }

    if (enablePubTF_) {
      tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    if (!childFrame_.empty()) {
      tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    }

    if (enablePubMap_) {
      pubMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_map", 10);
    }

    subMap_ = std::make_unique<PointMap>(rMap, rMax_);
    scanToMapRegister_ = std::make_unique<Register>(epsilon, sigma);
  }

private:
  void publishMap(std_msgs::msg::Header header) {
    auto mapMessage = sensor_msgs::msg::PointCloud2();
    mapMessage.header = header;
    mapMessage.header.frame_id = odomMessage_.header.frame_id;
    mapMessage.height = 1;
    mapMessage.width = subMap_->getPtCloud().size();

    sensor_msgs::PointCloud2Modifier mod(mapMessage);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(mapMessage.height * mapMessage.width);

    sensor_msgs::PointCloud2Iterator<float> iterX(mapMessage, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(mapMessage, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(mapMessage, "z");
    std::vector<Eigen::Vector4d>::const_iterator points = subMap_->getPtCloud().begin();

    for (; points != subMap_->getPtCloud().end(); ++points, ++iterX, ++iterY, ++iterZ) {
      *iterX = (*points)[0];
      *iterY = (*points)[1];
      *iterZ = (*points)[2];
    }

    this->pubMap_->publish(mapMessage);
  }

  void pointcloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    PointCloud newScan = PointCloud(rNew_, rMax_, rMin_, false);

    sensor_msgs::PointCloud2Iterator<float> iterX(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(*msg, "z");

    for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ) {
      newScan.addPoint(*iterX, *iterY, *iterZ);
    }

    newScan.processPointCloud();

    double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;

    if (initialised_) {
      double timePassed = (msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9)
        - (odomMessage_.header.stamp.sec + odomMessage_.header.stamp.nanosec / 1e9);
      scanToMapRegister_->registerScan(newScan.getPtCloud(), subMap_->getPcForKdTree());
      column_vector res = scanToMapRegister_->getRegResult();

      roll = res(0);
      pitch = res(1);
      yaw = res(2);
      x = res(3);
      y = res(4);
      z = res(5);

      tf2::Quaternion orientation, past_orientation;
      orientation.setRPY(roll, pitch, yaw);
      tf2::Vector3 translation = tf2::Vector3(x, y, z);

      if (!childFrame_.empty()) {
        geometry_msgs::msg::TransformStamped t;
        std::string fromFrameRel = msg->header.frame_id;
        std::string toFrameRel = childFrame_;
        try {
          t = tfBuffer_->lookupTransform(toFrameRel, fromFrameRel, msg->header.stamp);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
			toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }

	tf2::Transform transform = tf2::Transform(orientation, translation);
	tf2::Transform offsetTransform;
	tf2::fromMsg(t.transform, offsetTransform);
	transform *= offsetTransform;

	orientation = transform.getRotation();
	translation = transform.getOrigin();
        odomMessage_.child_frame_id = childFrame_;
      } else {
        odomMessage_.child_frame_id = msg->header.frame_id;
      }

      tf2::fromMsg(odomMessage_.pose.pose.orientation, past_orientation);

      tf2::Vector3 relativeTranslation = tf2::Vector3(x - odomMessage_.pose.pose.position.x,
          y - odomMessage_.pose.pose.position.y, z - odomMessage_.pose.pose.position.z);
      tf2::Vector3 transformedLinearVelocity = tf2::quatRotate(orientation.inverse(), relativeTranslation / timePassed);

      tf2::Quaternion relativeRotationQuat = orientation *  past_orientation.inverse();
      tf2::Matrix3x3 m(relativeRotationQuat);
      tf2::Vector3 relativeRotation;
      m.getRPY(relativeRotation[0], relativeRotation[1], relativeRotation[2]);
      tf2::Vector3 transformedAngularVelocity = tf2::quatRotate(orientation.inverse(), relativeRotation / timePassed);

      odomMessage_.pose.pose.position.x = translation[1];
      odomMessage_.pose.pose.position.y = translation[1];
      odomMessage_.pose.pose.position.z = translation[2];
      odomMessage_.twist.twist.linear.x = transformedLinearVelocity[0];
      odomMessage_.twist.twist.linear.y = transformedLinearVelocity[1];
      odomMessage_.twist.twist.linear.z = transformedLinearVelocity[2];

      odomMessage_.pose.pose.orientation = tf2::toMsg(orientation);
      odomMessage_.twist.twist.angular.x = transformedAngularVelocity[0];
      odomMessage_.twist.twist.angular.y = transformedAngularVelocity[1];
      odomMessage_.twist.twist.angular.z = transformedAngularVelocity[2];
    }

    Eigen::Matrix4d hypothesis = utils::homogeneous(roll, pitch, yaw, x, y, z);

    odomMessage_.header.stamp = msg->header.stamp;

    if (enablePubOdom_) {
      this->pubOdom_->publish(odomMessage_);
    }

    if (enablePubTF_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odomMessage_.header;
      transform.child_frame_id = odomMessage_.child_frame_id;
      transform.transform.translation.x = odomMessage_.pose.pose.position.x;
      transform.transform.translation.y = odomMessage_.pose.pose.position.y;
      transform.transform.translation.z = odomMessage_.pose.pose.position.z;
      transform.transform.rotation = odomMessage_.pose.pose.orientation;

      tfBroadcaster_->sendTransform(transform);
    }

    subMap_->updateMap(newScan.getPtCloud(), hypothesis);

    if (enablePubMap_) {
      publishMap(odomMessage_.header);
    }

    initialised_ = true;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPointcloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMap_;
  nav_msgs::msg::Odometry odomMessage_;
  std::string childFrame_;

  float rNew_, rMin_, rMax_;
  bool initialised_ = false;
  bool enablePubOdom_, enablePubTF_, enablePubMap_;

  std::unique_ptr<PointMap> subMap_;
  std::unique_ptr<Register> scanToMapRegister_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometry>());
  rclcpp::shutdown();
  return 0;
}
