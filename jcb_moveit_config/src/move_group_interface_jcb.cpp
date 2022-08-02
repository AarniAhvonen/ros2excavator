#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/int32.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>  
#include <Eigen/Eigen>  
#include <stdlib.h>
#include <Eigen/Geometry>  
#include <Eigen/Core>  
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <memory>
#include <string>
#include <geometry_msgs/msg/vector3.hpp>


using namespace std;
using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_jcb");
static const rclcpp::Logger LOGGER_JOINT = rclcpp::get_logger("joint_position");
static const rclcpp::Logger LOGGER_COUNT = rclcpp::get_logger("joint_position_count");

float angle_to_amps(float minimum_angle, float minimum_amps, float maximum_angle, float maximum_amps, float current_angle) {
  float angles_difference = maximum_angle - minimum_angle;
  float amps_diference = maximum_amps - minimum_amps;
  float amps = minimum_amps + (current_angle - minimum_angle) * (amps_diference / angles_difference);
  if (amps > maximum_amps) {
    return maximum_amps;
  } else if (amps < minimum_amps) {
    return minimum_amps;
  } else {
    return amps;
  }
}
// Create the node class named JointPositionPublisher which inherits the attributes
// and methods of the rclcpp::Node class.

class JointPositionPublisher : public rclcpp::Node
{
  public:
    float boom_minimum[2] = {0, 0.006};
    float boom_maximum[2] = {-2.24414, 0.0164};
    float stick_minimum[2] = {0, 0.0039};
    float stick_maximum[2] = {-1.93801, 0.0168};
    float bucket_minimum[2] = {0, 0.00410};
    float bucket_maximum[2] = {-3.32415, 0.0139};

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    JointPositionPublisher(moveit::planning_interface::MoveGroupInterface::Plan &plan)
    : Node("joint_count_position"), count_(0)
    {
      this->my_plan = plan;
      publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/joint_position_topic",10);
      count_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/joint_count_topic",1);
      timer_callback_with_plan();
    }

  private:

    void timer_callback_with_plan()
    {
      // Create a new message of type String
    //   auto message = geometry_msgs::msg::Vector3();
      moveit::planning_interface::MoveGroupInterface::Plan plan_in_function = this->my_plan;
      moveit_msgs::msg::RobotTrajectory trajectory = plan_in_function.trajectory_;
      std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_points;
      trajectory_points = trajectory.joint_trajectory.points;
      std::vector<int>::size_type vectorSize = trajectory_points.size();  //Check if there is something in the array
      RCLCPP_INFO(LOGGER_COUNT, "The total number of the trajectory is = %i", vectorSize);
      auto count_massage = std_msgs::msg::Int32();
      count_massage.data = static_cast<int>((vectorSize + 1) / 6) + 1;
      RCLCPP_INFO(LOGGER_COUNT, "The number of points that are sending is %i", count_massage.data);
      count_publisher_->publish(count_massage);
      // rclcpp::WallRate loop_rate(10.0);
      // loop_rate.sleep();
      // Set our message's data attribute and increment the message count by 1
      std::vector<double> jcb_positions = plan_in_function.trajectory_.joint_trajectory.points[0].positions;
      for (unsigned i=0; i<vectorSize; i++)
      {
        if (i%6 != 0 and i != vectorSize -1){
          continue;
        }
        auto message = geometry_msgs::msg::Vector3();
        message.x = angle_to_amps(
          this->boom_minimum[0], this->boom_minimum[1],
          this->boom_maximum[0], this->boom_maximum[1],
          trajectory.joint_trajectory.points[i].positions[0]);
        message.y = angle_to_amps(
          this->stick_minimum[0], this->stick_minimum[1],
          this->stick_maximum[0], this->stick_maximum[1],
          trajectory.joint_trajectory.points[i].positions[1]);
        message.z = angle_to_amps(
          this->bucket_minimum[0], this->bucket_minimum[1],
          this->bucket_maximum[0], this->bucket_maximum[1],
          trajectory.joint_trajectory.points[i].positions[2]);
        RCLCPP_DEBUG(
          LOGGER_JOINT,
          "current base amps: %f", trajectory.joint_trajectory.points[i].positions[0]);
        RCLCPP_INFO(
          LOGGER_JOINT,
          "Published values are:\n base amps: %f, stick amps: %f, bucket amps: %f",
          message.x, message.y, message.z);
        publisher_->publish(message);
        RCLCPP_INFO(LOGGER_JOINT, "Published %ith message", i);
       }
    }
  
    // Declaration of the publisher_s attribute
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr count_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
   
    // Declaration of the count_ attribute
    size_t count_;
};
 
// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("jcb_moveit_config", node_options);
  rclcpp::Rate rate(5);
  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();
  rclcpp::executors::SingleThreadedExecutor executor_joint;

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "world", "move_group_jcb",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Jcb", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  RCLCPP_DEBUG(LOGGER, "Robot current state: \n or.x: %f, or.y:%f, or.z:%f, or.w:%f \n po.x:%f, po.y:%f, po.z:%f", 
              move_group.getCurrentPose().pose.orientation.x, move_group.getCurrentPose().pose.orientation.y,
              move_group.getCurrentPose().pose.orientation.z, move_group.getCurrentPose().pose.orientation.w,
              move_group.getCurrentPose().pose.position.x, move_group.getCurrentPose().pose.position.y,
              move_group.getCurrentPose().pose.position.z);

  // We can also print the name of the end-effector link for this group.
  RCLCPP_DEBUG(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_DEBUG(LOGGER, "Current goal orentation tolerance: %f", move_group.getGoalOrientationTolerance());

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // debug
  // Changed IK solver from kdl to lma!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // debug

  geometry_msgs::msg::Pose pose_target;
  float x, z, pitch;
  x = -1.6;
  z = 1.2;
  pitch = -3;
  RCLCPP_INFO(LOGGER, "The target position of x is set as %f , z is set as %f, pitch is set as %f", x, z, pitch);
  pose_target.position.x = x;
  pose_target.position.y = 0;
  pose_target.position.z = z;

  Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle(0, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

  pose_target.orientation.x = q.x();
  pose_target.orientation.y = q.y();
  pose_target.orientation.z = q.z();
  pose_target.orientation.w = q.w();

  RCLCPP_DEBUG(LOGGER, "Planning to goal point: x:%f, y:%f, z%f:, w:%f", pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w);
  move_group.setPoseTarget(pose_target);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == true)
    {
      JointPositionPublisher my_publisher_obj(my_plan);
      // rclcpp::spin(std::make_shared<JointPositionPublisher>(my_plan));
    }
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(pose_target, "pose1");
  visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // Moving to a pose goal
  move_group.move();
  
  // Start processing data from the node as well as the callbacks and the timer
 
  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}
