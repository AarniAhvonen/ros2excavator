#include <memory>
#include <string>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_sub_main");
static const rclcpp::Logger LOGGER_JOINT = rclcpp::get_logger("joint_position");
static const rclcpp::Logger LOGGER_COUNT = rclcpp::get_logger("joint_count");
class JointSubscriber : public rclcpp::Node
{
  public:
    JointSubscriber()
    : Node("joint_count_position_subscriber")
    { 
      count_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
      "/joint_count_topic", 1, std::bind(&JointSubscriber::joint_count_callback, this, _1));
      position_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/joint_position_topic", 10, std::bind(&JointSubscriber::joint_position_callback, this, _1));
    }

  private:
    void joint_count_callback(const std_msgs::msg::Int32::SharedPtr count_msg) const
    {
      RCLCPP_INFO(LOGGER_COUNT, "The total count of the trejectory is '%i'", count_msg->data);
    }
    void joint_position_callback(const geometry_msgs::msg::Vector3::SharedPtr position_msg) const
    {
      RCLCPP_INFO(LOGGER_JOINT,
          "Published values are:\n base position: %f, stick position: %f, bucket position: %f",
          position_msg->x, position_msg->y, position_msg->z);
    }
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr count_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr position_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(LOGGER, "Start spining");
  rclcpp::spin(std::make_shared<JointSubscriber>());
  RCLCPP_INFO(LOGGER, "Finished");
  rclcpp::shutdown();
  return 0;
}
