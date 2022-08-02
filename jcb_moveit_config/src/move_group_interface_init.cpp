#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/int32.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

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
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_init");

float amps_to_angle(float minimum_angle, float minimum_amps, float maximum_angle, float maximum_amps, float current_amps) {
  float angles_difference = maximum_angle - minimum_angle;
  float amps_diference = maximum_amps - minimum_amps;
  float angle = minimum_angle + (current_amps - minimum_amps) * (angles_difference / amps_diference);
  if (angle > minimum_angle) {
    return minimum_angle;
  } else if (angle < maximum_angle) {
    return maximum_angle;
  } else {
    return angle;
  }
}

// Node execution starts here
int main(int argc, char * argv[])
{
  // Geting the pose from excavator and convert it to angle values
  float boom_minimum[2] = {0, 0.006};
  float boom_maximum[2] = {-2.24414, 0.0164};
  float stick_minimum[2] = {0, 0.0039};
  float stick_maximum[2] = {-1.93801, 0.0168};
  float bucket_minimum[2] = {0, 0.00410};
  float bucket_maximum[2] = {-3.32415, 0.0139};
  float data[3] =  {0.006511, 0.007298, 0.008159};
  float boom_angle = amps_to_angle(
    boom_minimum[0], boom_minimum[1],
    boom_maximum[0], boom_maximum[1],
    data[0]);
  float stick_angle = amps_to_angle(
    stick_minimum[0], stick_minimum[1],
    stick_maximum[0], stick_maximum[1],
    data[1]);
  float bucket_angle = amps_to_angle(
    bucket_minimum[0], bucket_minimum[1],
    bucket_maximum[0], bucket_maximum[1],
    data[2]);

  // Initialize ROS 2
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("jcb_moveit_init_node", node_options);
  rclcpp::Rate rate(5);
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
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

  
  // Publish a floor obj in the world of simulation
  // const std::vector<std::string>& objects = getWorld()->getObjectIds();
  auto object_names = planning_scene_interface.getKnownObjectNames();
  bool floorNotExisted;
  floorNotExisted = true;
  for (auto& name : object_names)
  {
    if (name == "floor"){
      floorNotExisted = false;
      RCLCPP_INFO(LOGGER, "The floor existed");
    }
  }
  if (floorNotExisted) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "floor";
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 4;
    primitive.dimensions[primitive.BOX_Y] = 4;
    primitive.dimensions[primitive.BOX_Z] = 0.2;
    // DEFINE THE POSE
    geometry_msgs::msg::Pose floor_pose;
    floor_pose.orientation.w = 1.0;
    floor_pose.position.x = -0.5;
    floor_pose.position.y = 0.0;
    floor_pose.position.z = -0.15;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(floor_pose);
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    RCLCPP_INFO(LOGGER, "Add the floor into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
  }

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = boom_angle;  // radians
  joint_group_positions[1] = stick_angle;  // radians
  joint_group_positions[2] = bucket_angle;  // radians
  move_group.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz:
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // Moving to a pose goal
  move_group.move();
  rclcpp::shutdown();
  return 0;
}
