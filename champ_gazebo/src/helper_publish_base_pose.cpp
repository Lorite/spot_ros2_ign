#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

/**
 * @class HelperPublishBase
 */
class HelperPublishBase : public rclcpp::Node {
public:
  HelperPublishBase()
  : Node("helper_publisher")
  {
    // Parameters (with sensible defaults)
    this->declare_parameter<std::string>("input_topic", "/model/spot/pose");
    this->declare_parameter<std::string>("output_topic", "ground_truth_odom");

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();

    // Subscriber: geometry_msgs/TransformStamped
    sub_tf_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&HelperPublishBase::tf_cb, this, std::placeholders::_1));

    // Publisher: nav_msgs/Odometry
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(10));
  }

private:
  void tf_cb(const geometry_msgs::msg::TransformStamped & tf_msg)
  {
    nav_msgs::msg::Odometry odom;
    // Header/frame association mirrors the TransformStamped
    odom.header = tf_msg.header;               // e.g., frame_id = odom/world (as provided)
    odom.child_frame_id = tf_msg.child_frame_id; // e.g., base_link (possibly prefixed)

    // Pose from transform
    odom.pose.pose.position.x = tf_msg.transform.translation.x;
    odom.pose.pose.position.y = tf_msg.transform.translation.y;
    odom.pose.pose.position.z = tf_msg.transform.translation.z;
    odom.pose.pose.orientation = tf_msg.transform.rotation;

    // No velocity info in TransformStamped; publish zeros by default
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pub_odom_->publish(odom);
  }

  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr sub_tf_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::string input_topic_;
  std::string output_topic_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelperPublishBase>());
  rclcpp::shutdown();
  return 0;
}
