#include "std_msgs/msg/string.hpp"
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
 
// Create the node class named JointPositionPublisher which inherits the attributes
// and methods of the rclcpp::Node class.
class JointPositionPublisher : public rclcpp::Node
{
  public:
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // Constructor creates a node named minimal_publisher. 
    // The published message count is initialized to 0.
    JointPositionPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      // Publisher publishes String messages to a topic named "addison". 
      // The size of the queue is 10 messages.
      publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("addison",10);
       
      // Initialize the timer. The timer_callback function will execute every
      // 500 milliseconds.
      timer_ = this->create_wall_timer(
      500ms, std::bind(&JointPositionPublisher::timer_callback, this));
    }
    JointPositionPublisher(moveit::planning_interface::MoveGroupInterface::Plan &plan)
    : Node("joint_position"), count_(0)
    {
      this->my_plan = plan;
      // Publisher publishes String messages to a topic named "addison". 
      // The size of the queue is 10 messages.
      publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("joint_position_topic",10);
       
      // Initialize the timer. The timer_callback function will execute every
      // 500 milliseconds.
    //   timer_ = this->create_wall_timer(
    //   500ms, std::bind(&JointPositionPublisher::timer_callback_with_plan, this));
      timer_callback_with_plan();
    }
 
  private:
    // This method executes every 500 milliseconds
    void timer_callback()
    {
      // Create a new message of type String
      auto message = geometry_msgs::msg::Vector3();
       
      // Set our message's data attribute and increment the message count by 1
      message.x = 1;
      message.y = 2;
      message.z = 3;
 
      // Print every message to the terminal window      
      RCLCPP_INFO(LOGGER_JOINT,"Publishing message:\n x: %f, y: %f, z:%f", message.x, message.y, message.z);
      // Publish the message to the topic named "addison"
      publisher_->publish(message);
    }

    void timer_callback_with_plan()
    {
      // Create a new message of type String
    //   auto message = geometry_msgs::msg::Vector3();
      moveit::planning_interface::MoveGroupInterface::Plan plan_in_function = this->my_plan;
      moveit_msgs::msg::RobotTrajectory trajectory = plan_in_function.trajectory_;
      std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_points;
      trajectory_points = trajectory.joint_trajectory.points;
      std::vector<int>::size_type vectorSize = trajectory_points.size();  //Check if there is something in the array
      RCLCPP_INFO(LOGGER_JOINT, "The size of the vectorSize = %i", vectorSize);
      // Set our message's data attribute and increment the message count by 1
      std::vector<double> jcb_positions = plan_in_function.trajectory_.joint_trajectory.points[0].positions;
      float position_base;
      float position_stick;
      float position_bucket;
      for (unsigned i=0; i<vectorSize; i++)
      {
        auto message = geometry_msgs::msg::Vector3();
        RCLCPP_INFO(LOGGER, "i equal to %i", i);
        position_base = trajectory.joint_trajectory.points[i].positions[0];
        message.x = position_base;
        RCLCPP_INFO(LOGGER, "Float value for base position = %f", position_base);
        position_stick = trajectory.joint_trajectory.points[i].positions[1];
        message.y = position_stick;
        RCLCPP_INFO(LOGGER, "Float value for base position = %f", position_stick);
        position_bucket = trajectory.joint_trajectory.points[i].positions[2];
        message.y = position_bucket;
        RCLCPP_INFO(LOGGER, "Float value for base position = %f", position_bucket);
        publisher_->publish(message);
        RCLCPP_INFO(LOGGER, "Published %ith message", i);
       }
    //   message.x = 1;
    //   message.y = 2;
    //   message.z = 3;
 
    //   // Print every message to the terminal window      
    //   RCLCPP_INFO(LOGGER_JOINT,"Publishing message:\n x: %f, y: %f, z:%f", message.x, message.y, message.z);
    //   // Publish the message to the topic named "addison"
    //   publisher_->publish(message);
    }
     
    // Declaration of the timer_ attribute
    rclcpp::TimerBase::SharedPtr timer_;
  
    // Declaration of the publisher_ attribute
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
  

  // We can get a list of all the groups in the robot:
  RCLCPP_DEBUG(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // debug
  // Changed IK solver from kdl to lma!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // debug

  geometry_msgs::msg::Pose pose_target;
  pose_target.position.x = -1.6;
  pose_target.position.y = 0;
  pose_target.position.z = 1.2;

  Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle(0, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle(-3, Eigen::Vector3d::UnitY());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

  pose_target.orientation.x = q.x();
  pose_target.orientation.y = q.y();
  pose_target.orientation.z = q.z();
  pose_target.orientation.w = q.w();

  RCLCPP_DEBUG(LOGGER, "Planning to goal point: x:%f, y:%f, z%f:, w:%f", pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w);
  move_group.setPoseTarget(pose_target);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if (success == true)
    {
      JointPositionPublisher my_publisher_obj(my_plan);
    //   rclcpp::spin(std::make_shared<JointPositionPublisher>(my_plan));
    }

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(pose_target, "pose1");
  visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  cout << "The trajecttory is: " << my_plan.trajectory_.joint_trajectory.points[0].positions[1];

  // Moving to a pose goal
  move_group.move();
  
  // Start processing data from the node as well as the callbacks and the timer
 
  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}
