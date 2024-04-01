#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

double to_radians(const double deg_angle)
{
  return deg_angle * M_PI / 180.0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_arm(move_group_arm_node, "arm");
  move_group_arm.setMaxVelocityScalingFactor(1.0);
  move_group_arm.setMaxAccelerationScalingFactor(1.0);

  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper");
  move_group_gripper.setMaxVelocityScalingFactor(1.0);
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);
  auto gripper_joint_values = move_group_gripper.getCurrentJointValues();
  double GRIPPER_DEFAULT = 0.0;
  double GRIPPER_OPEN = to_radians(-30);
  double GRIPPER_CLOSE = to_radians(15);

  move_group_arm.setNamedTarget("vertical");
  move_group_arm.move();

  gripper_joint_values[0] = GRIPPER_DEFAULT;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();
#if 0
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();
#endif
  gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  double box_positions[] = {-0.06, -0.11, -0.16};
  int count = static_cast<int>(sizeof(box_positions) / sizeof(double));

  for (int i = 0; i < count; i++) {
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    target_pose.position.x = 0.0;
    target_pose.position.y = -0.09;
    target_pose.position.z = 0.19;
    q.setRPY(to_radians(0), to_radians(90), to_radians(-90));
    target_pose.orientation = tf2::toMsg(q);
    move_group_arm.setPoseTarget(target_pose);
    move_group_arm.move();

    q.setRPY(to_radians(0), to_radians(180), to_radians(-90));
    target_pose.orientation = tf2::toMsg(q);
    move_group_arm.setPoseTarget(target_pose);
    move_group_arm.move();

    target_pose.position.y = box_positions[i];
    target_pose.position.z = 0.12;
    move_group_arm.setPoseTarget(target_pose);
    move_group_arm.move();

    // Grasp
    gripper_joint_values[0] = GRIPPER_CLOSE;
    move_group_gripper.setJointValueTarget(gripper_joint_values);
    move_group_gripper.move();

    target_pose.position.z = 0.17;
    move_group_arm.setPoseTarget(target_pose);
    move_group_arm.move();

    move_group_arm.setNamedTarget("home");
    move_group_arm.move();

    target_pose.position.x = 0.10;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.06;
    q.setRPY(to_radians(0), to_radians(90), to_radians(0));
    target_pose.orientation = tf2::toMsg(q);
    move_group_arm.setPoseTarget(target_pose);
    move_group_arm.move();

    // Release
    gripper_joint_values[0] = GRIPPER_OPEN;
    move_group_gripper.setJointValueTarget(gripper_joint_values);
    move_group_gripper.move();

    move_group_arm.setNamedTarget("home");
    move_group_arm.move();
  }

  move_group_arm.setNamedTarget("vertical");
  move_group_arm.move();

  gripper_joint_values[0] = GRIPPER_DEFAULT;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  rclcpp::shutdown();

  return 0;
}
