// student ID: 2023105727
// name: 권구현
//step1_practice.cpp 를 참고하여 구현하세요

#include <rclcpp/rclcpp.hpp>

// MoveIt 2
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <chrono>
#include <thread>
#include <vector>
#include <cmath>
#include <iostream>

class Location {
public:
  double x;
  double y;
  Location() = default;
  Location(double x_value, double y_value) : x(x_value), y(y_value) {}
};

class Block {
public:
  double width;
  double length;
  double height;
  double radius;
  Location location;
  Block() = default;
  Block(double width_value, double length_value, double height_value, double radius_value, const Location &location_value)
    : width(width_value), length(length_value), height(height_value), radius(radius_value), location(location_value) {}
};

struct Slot {
  Location location;
  double height;
  Slot() = default;
  Slot(const Location &loc, double h) : location(loc), height(h) {}
};

geometry_msgs::msg::Pose list_to_pose(double x, double y, double z,
                                      double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose pose;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = tf2::toMsg(q);
  return pose;
}

void waypoint_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                     rclcpp::Node::SharedPtr node,
                     const std::vector<geometry_msgs::msg::Pose> &waypoints,
                     double time_scale = 1.0)
{
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "waypoint_sample");

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_interface.computeCartesianPath(
      waypoints, 0.005, 0.0, trajectory);
  RCLCPP_INFO(logger, "Fraction score : fraction=%.3f", fraction);

  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory;
  if (time_scale > 1.0) {
    for (auto &pt : cartesian_plan.trajectory_.joint_trajectory.points) {
      double t = static_cast<double>(pt.time_from_start.sec) +
                 static_cast<double>(pt.time_from_start.nanosec) * 1e-9;
      t *= time_scale;
      pt.time_from_start.sec = static_cast<int32_t>(t);
      pt.time_from_start.nanosec = static_cast<uint32_t>((t - static_cast<double>(pt.time_from_start.sec)) * 1e9);
    }
  }
  move_group_interface.execute(cartesian_plan);
}

void open_gripper(moveit::planning_interface::MoveGroupInterface &gripper_interface)
{
  RCLCPP_INFO(rclcpp::get_logger("gripper"), "Gripper opening...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<double> joint_group_positions = gripper_interface.getCurrentJointValues();
  joint_group_positions[0] = 0.07;
  joint_group_positions[1] = 0.0;
  gripper_interface.setJointValueTarget(joint_group_positions);

  auto planning_result = gripper_interface.plan(my_plan);
  bool success = (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    gripper_interface.execute(my_plan);
    rclcpp::sleep_for(std::chrono::milliseconds(300));
  }
}

void close_gripper(moveit::planning_interface::MoveGroupInterface &gripper_interface, double value)
{
  RCLCPP_INFO(rclcpp::get_logger("gripper"), "Gripper closing...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<double> joint_group_positions = gripper_interface.getCurrentJointValues();
  joint_group_positions[0] = value;
  joint_group_positions[1] = 0.0;
  gripper_interface.setJointValueTarget(joint_group_positions);

  auto planning_result = gripper_interface.plan(my_plan);
  bool success = (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    gripper_interface.execute(my_plan);
  }
}

void initial_pose(moveit::planning_interface::MoveGroupInterface &arm_interface)
{
  RCLCPP_INFO(rclcpp::get_logger("arm"), "Moving to initial pose...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<double> joint_group_positions = arm_interface.getCurrentJointValues();
  joint_group_positions = {0.0, -2.03, 1.58, -1.19, -1.58, 0.78};
  arm_interface.setJointValueTarget(joint_group_positions);

  auto planning_result = arm_interface.plan(my_plan);
  bool success = (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    arm_interface.execute(my_plan);
  }
}

class HomeworkPickAndPlaceNode : public rclcpp::Node
{
public:
  HomeworkPickAndPlaceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("homework_pick_and_place", options)
  {
    RCLCPP_INFO(this->get_logger(), "Homework Pick-and-Place Node started.");
  }

  void run()
  {
    auto node_ptr = this->shared_from_this();

    moveit::planning_interface::MoveGroupInterface arm(node_ptr, "manipulator");
    moveit::planning_interface::MoveGroupInterface gripper(node_ptr, "gripper");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    arm.setPlanningTime(10.0);
    rclcpp::sleep_for(std::chrono::seconds(1));

    moveit_msgs::msg::CollisionObject ground_plane;
    ground_plane.header.frame_id = arm.getPlanningFrame();
    ground_plane.id = "ground_plane";

    shape_msgs::msg::SolidPrimitive plane_primitive;
    plane_primitive.type = plane_primitive.BOX;
    plane_primitive.dimensions = {4.0, 4.0, 0.01};

    geometry_msgs::msg::Pose plane_pose;
    plane_pose.orientation.w = 1.0;
    plane_pose.position.z = -0.005;

    ground_plane.primitives.push_back(plane_primitive);
    ground_plane.primitive_poses.push_back(plane_pose);
    ground_plane.operation = ground_plane.ADD;

    planning_scene_interface.applyCollisionObjects({ground_plane});

    initial_pose(arm);
    close_gripper(gripper, 0.0);
    open_gripper(gripper);

    block1(node_ptr, arm, gripper);
    block2(node_ptr, arm, gripper);
    block3(node_ptr, arm, gripper);
    block4(node_ptr, arm, gripper);
    block5(node_ptr, arm, gripper);
    block6(node_ptr, arm, gripper);
    block8(node_ptr, arm, gripper);
    block7(node_ptr, arm, gripper);

    RCLCPP_INFO(this->get_logger(), "Task finished.");
  }

private:
  void pick_object(rclcpp::Node::SharedPtr node,
                   moveit::planning_interface::MoveGroupInterface &arm_interface,
                   moveit::planning_interface::MoveGroupInterface &gripper_interface,
                   const Block &object, double grip_value, double yaw)
  {
    const double SAFE_Z = 0.5;
    geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
    geometry_msgs::msg::Pose lift_pose = current_pose;
    lift_pose.position.z = SAFE_Z;

    geometry_msgs::msg::Pose above_pose =
        list_to_pose(object.location.x, object.location.y, SAFE_Z,
                     M_PI, 0.0, yaw);

    geometry_msgs::msg::Pose grasp_pose =
        list_to_pose(object.location.x, object.location.y,
                     object.height + 0.185,
                     M_PI, 0.0, yaw);

    std::vector<geometry_msgs::msg::Pose> waypoints2;
    std::vector<geometry_msgs::msg::Pose> waypoints3;
    std::vector<geometry_msgs::msg::Pose> waypoints_up;

    if (current_pose.position.z < SAFE_Z - 1e-3) {
      waypoints_up.clear();
      waypoints_up.push_back(lift_pose);
      waypoint_sample(arm_interface, node, waypoints_up);
    }

    waypoints2.clear();
    waypoints2.push_back(above_pose);
    waypoint_sample(arm_interface, node, waypoints2);

    open_gripper(gripper_interface);

    waypoints3.clear();
    waypoints3.push_back(grasp_pose);
    waypoint_sample(arm_interface, node, waypoints3);

    close_gripper(gripper_interface, grip_value);

    waypoints_up.clear();
    waypoints_up.push_back(above_pose);
    waypoint_sample(arm_interface, node, waypoints_up);
  }

  void place_object(rclcpp::Node::SharedPtr node,
                    moveit::planning_interface::MoveGroupInterface &arm_interface,
                    moveit::planning_interface::MoveGroupInterface &gripper_interface,
                    const Block &object,
                    const Slot &slot,
                    double yaw,
                    double descend_time_scale = 1.0,
                    double place_z_offset = 0.0)
  {
    const double SAFE_Z = 0.50;
    geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
    geometry_msgs::msg::Pose lift_pose = current_pose;
    lift_pose.position.z = SAFE_Z;

    geometry_msgs::msg::Pose above_pose =
        list_to_pose(slot.location.x, slot.location.y, SAFE_Z,
                     M_PI, 0.0, yaw);

    geometry_msgs::msg::Pose place_pose =
        list_to_pose(slot.location.x, slot.location.y,
                     slot.height + object.height + 0.185 + place_z_offset,
                     M_PI, 0.0, yaw);

    std::vector<geometry_msgs::msg::Pose> waypoints1;
    std::vector<geometry_msgs::msg::Pose> waypoints2;
    std::vector<geometry_msgs::msg::Pose> waypoints_up;

    if (current_pose.position.z < SAFE_Z - 1e-3) {
      waypoints_up.clear();
      waypoints_up.push_back(lift_pose);
      waypoint_sample(arm_interface, node, waypoints_up);
    }

    waypoints1.clear();
    waypoints1.push_back(above_pose);
    waypoint_sample(arm_interface, node, waypoints1);
    waypoints1.clear();
    waypoints1.push_back(place_pose);
    waypoint_sample(arm_interface, node, waypoints1, descend_time_scale);

    open_gripper(gripper_interface);

    waypoints2.push_back(above_pose);
    waypoint_sample(arm_interface, node, waypoints2);
  }

  void block1(const rclcpp::Node::SharedPtr &node,
              moveit::planning_interface::MoveGroupInterface &arm_interface,
              moveit::planning_interface::MoveGroupInterface &gripper_interface)
  {
    Block object(0.05, 0.05, 0.06, 0.0, Location(0.40, 0.10));
    Slot slot(Location(-0.15, 0.45), 0.0);
    double yaw = -M_PI / 4.0;
    double grip_value = 0.02;
    pick_object(node, arm_interface, gripper_interface, object, grip_value, yaw);
    place_object(node, arm_interface, gripper_interface, object, slot, yaw);
  }

  void block2(const rclcpp::Node::SharedPtr &node,
              moveit::planning_interface::MoveGroupInterface &arm_interface,
              moveit::planning_interface::MoveGroupInterface &gripper_interface)
  {
    Block object(0.05, 0.05, 0.06, 0.0, Location(0.40, 0.00));
    Slot slot(Location(-0.15, 0.55), 0.0);
    double yaw = -M_PI / 4.0;
    double grip_value = 0.02;
    pick_object(node, arm_interface, gripper_interface, object, grip_value, yaw);
    place_object(node, arm_interface, gripper_interface, object, slot, yaw);
  }

  void block3(const rclcpp::Node::SharedPtr &node,
              moveit::planning_interface::MoveGroupInterface &arm_interface,
              moveit::planning_interface::MoveGroupInterface &gripper_interface)
  {
    Block object(0.05, 0.025, 0.06, 0.0, Location(0.40, -0.10));
    Slot slot(Location(0.15, 0.45), 0.0);
    double yaw = -M_PI / 4.0;
    const double yaw_adjusted_pick = yaw + M_PI / 2.0;  
    const double yaw_adjusted_place = yaw;  
    double grip_value = 0.02;
    pick_object(node, arm_interface, gripper_interface, object, grip_value, yaw_adjusted_pick);
    place_object(node, arm_interface, gripper_interface, object, slot, yaw_adjusted_place);
  }

  void block4(const rclcpp::Node::SharedPtr &node,
              moveit::planning_interface::MoveGroupInterface &arm_interface,
              moveit::planning_interface::MoveGroupInterface &gripper_interface)
  {
    Block object(0.05, 0.05, 0.08, 0.0, Location(0.50, 0.10));
    Slot slot(Location(0.05, 0.55), 0.0);
    double yaw = -M_PI / 4.0;
    double grip_value = 0.02;
    pick_object(node, arm_interface, gripper_interface, object, grip_value, yaw);
    place_object(node, arm_interface, gripper_interface, object, slot, yaw);
  }

  void block5(const rclcpp::Node::SharedPtr &node,
              moveit::planning_interface::MoveGroupInterface &arm_interface,
              moveit::planning_interface::MoveGroupInterface &gripper_interface)
  {
    Block object(0.05, 0.05, 0.06, 0.025, Location(0.50, 0.00));
    Slot slot(Location(-0.05, 0.45), 0.0);
    double yaw = 0.0;
    double grip_value = 0.02;
    pick_object(node, arm_interface, gripper_interface, object, grip_value, yaw);
    place_object(node, arm_interface, gripper_interface, object, slot, yaw);
  }

  void block6(const rclcpp::Node::SharedPtr &node,
              moveit::planning_interface::MoveGroupInterface &arm_interface,
              moveit::planning_interface::MoveGroupInterface &gripper_interface)
  {
    Block object(0.05, 0.05, 0.07, 0.0, Location(0.50, -0.10));
    Slot slot(Location(-0.05, 0.55), 0.0);
    double yaw = -M_PI / 4.0;
    double grip_value = 0.02;
    pick_object(node, arm_interface, gripper_interface, object, grip_value, yaw);
    place_object(node, arm_interface, gripper_interface, object, slot, yaw);
  }

  void block7(const rclcpp::Node::SharedPtr &node,
              moveit::planning_interface::MoveGroupInterface &arm_interface,
              moveit::planning_interface::MoveGroupInterface &gripper_interface)
  {
    Block object(0.05, 0.05, 0.06, 0.0, Location(0.60, 0.10));
    Slot slot(Location(0.05, 0.45), 0.0);
    double yaw = M_PI / 4.0;
    double grip_value = 0.0175;
    Block pick_object_with_offset = object;
    pick_object_with_offset.location.x += 0.00625;
    pick_object(node, arm_interface, gripper_interface, pick_object_with_offset, grip_value, yaw);
    double yaw_adjusted_place = -M_PI / 4.0;
    Slot place_slot = slot;
    place_slot.location.y -= 0.00625;
    place_object(node, arm_interface, gripper_interface, object, place_slot, yaw_adjusted_place, 5.0, 0.015);
  }

  void block8(const rclcpp::Node::SharedPtr &node,
              moveit::planning_interface::MoveGroupInterface &arm_interface,
              moveit::planning_interface::MoveGroupInterface &gripper_interface)
  {
    Block object(0.05, 0.05, 0.09, 0.0, Location(0.60, 0.00));
    Slot slot(Location(0.15, 0.55), 0.0);
    double yaw = -M_PI / 4.0;
    double grip_value = 0.0243;
    pick_object(node, arm_interface, gripper_interface, object, grip_value, yaw);
    place_object(node, arm_interface, gripper_interface, object, slot, yaw);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HomeworkPickAndPlaceNode>();

  std::thread spinner([node]() {
    rclcpp::spin(node);
  });
  spinner.detach();

  node->run();

  rclcpp::shutdown();
  return 0;
}
