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

// 2D 위치 표현용
class Location {
public:
  double x;
  double y;
  Location() = default;
  Location(double x_value, double y_value) : x(x_value), y(y_value) {}
};

// 오브젝트(큐브, 원기둥, 삼각기둥) 정보 표현용
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

// case 안의 슬롯 정보 (위치 + 높이)
struct Slot {
  Location location;
  double height;
  Slot() = default;
  Slot(const Location &loc, double h) : location(loc), height(h) {}
};

// RPY + xyz → Pose 변환
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

// 카르테시안 경로(waypoint) 계산 및 실행
void waypoint_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                     rclcpp::Node::SharedPtr node,
                     const std::vector<geometry_msgs::msg::Pose> &waypoints)
{
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "waypoint_sample");

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_interface.computeCartesianPath(
      waypoints, 0.005, 0.0, trajectory);
  RCLCPP_INFO(logger, "Fraction score : fraction=%.3f", fraction);

  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory;
  move_group_interface.execute(cartesian_plan);
}

// 그리퍼 열기
void open_gripper(moveit::planning_interface::MoveGroupInterface &gripper_interface)
{
  RCLCPP_INFO(rclcpp::get_logger("gripper"), "Gripper opening...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<double> joint_group_positions = gripper_interface.getCurrentJointValues();
  joint_group_positions[0] = 0.07; // 충분히 여는 값 (필요시 조정)
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

// 그리퍼 닫기 (value: grip 폭)
void close_gripper(moveit::planning_interface::MoveGroupInterface &gripper_interface, double value)
{
  RCLCPP_INFO(rclcpp::get_logger("gripper"), "Gripper closing...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<double> joint_group_positions = gripper_interface.getCurrentJointValues();
  joint_group_positions[0] = value; // 과제에서 튜닝해야 하는 값
  joint_group_positions[1] = 0.0;
  gripper_interface.setJointValueTarget(joint_group_positions);

  auto planning_result = gripper_interface.plan(my_plan);
  bool success = (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    gripper_interface.execute(my_plan);
  }
}

// 초기 joint posture
void initial_pose(moveit::planning_interface::MoveGroupInterface &arm_interface)
{
  RCLCPP_INFO(rclcpp::get_logger("arm"), "Moving to initial pose...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<double> joint_group_positions = arm_interface.getCurrentJointValues();
  // step1_practice.cpp 에서 사용하던 초기 자세
  joint_group_positions = {0.0, -2.03, 1.58, -1.19, -1.58, 0.78};
  arm_interface.setJointValueTarget(joint_group_positions);

  auto planning_result = arm_interface.plan(my_plan);
  bool success = (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    arm_interface.execute(my_plan);
  }
}

// 메인 노드
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

    // 1) ground plane 충돌 객체 추가 (step1_practice와 동일)
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

    // 2) 초기 자세 및 그리퍼 초기화
    initial_pose(arm);
    close_gripper(gripper, 0.0);
    open_gripper(gripper);

    // 3) 각 블록별 함수 호출로 분리 (개별 함수 내부에서 파라미터 조정 가능)
    // block1(node_ptr, arm, gripper);
    // block2(node_ptr, arm, gripper);
    // block3(node_ptr, arm, gripper);
    // block4(node_ptr, arm, gripper);
    // block5(node_ptr, arm, gripper);
    // block6(node_ptr, arm, gripper);
    block7(node_ptr, arm, gripper);
    // block8(node_ptr, arm, gripper);

    RCLCPP_INFO(this->get_logger(), "Task finished.");
  }

private:
  // 물체 집기
  void pick_object(rclcpp::Node::SharedPtr node,
                   moveit::planning_interface::MoveGroupInterface &arm_interface,
                   moveit::planning_interface::MoveGroupInterface &gripper_interface,
                   const Block &object, double grip_value, double yaw)
  {
    const double SAFE_Z = 0.40;
    // 현재 자세
    geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
    geometry_msgs::msg::Pose lift_pose = current_pose;
    lift_pose.position.z = SAFE_Z;

    // 접근 자세: 물체 위 안전 높이
    geometry_msgs::msg::Pose above_pose =
        list_to_pose(object.location.x, object.location.y, SAFE_Z,
                     M_PI, 0.0, yaw);

    // grasp 자세: 물체 높이 + 엔드이펙터 오프셋 (0.185는 step1 예제와 동일)
    geometry_msgs::msg::Pose grasp_pose =
        list_to_pose(object.location.x, object.location.y,
                     object.height + 0.185,
                     M_PI, 0.0, yaw);

    std::vector<geometry_msgs::msg::Pose> waypoints2;
    std::vector<geometry_msgs::msg::Pose> waypoints3;
    std::vector<geometry_msgs::msg::Pose> waypoints_up;

    // 0) 수직 상승하여 안전 높이 도달
    if (current_pose.position.z < SAFE_Z - 1e-3) {
      waypoints_up.clear();
      waypoints_up.push_back(lift_pose);
      waypoint_sample(arm_interface, node, waypoints_up);
    }

    // 1) 같은 Z에서 대상 위로 수평 이동 (카르테시안)
    waypoints2.clear();
    waypoints2.push_back(above_pose);
    waypoint_sample(arm_interface, node, waypoints2);

    // 2) 그리퍼 열기
    open_gripper(gripper_interface);

    // 3) grasp_pose까지 내려가기
    waypoints3.clear();
    waypoints3.push_back(grasp_pose);
    waypoint_sample(arm_interface, node, waypoints3);

    // 4) 물체를 잡기
    close_gripper(gripper_interface, grip_value);

    // 5) 다시 위로 올라가기
    waypoints_up.clear();
    waypoints_up.push_back(above_pose);
    waypoint_sample(arm_interface, node, waypoints_up);
  }

  // 물체 내려놓기
  void place_object(rclcpp::Node::SharedPtr node,
                    moveit::planning_interface::MoveGroupInterface &arm_interface,
                    moveit::planning_interface::MoveGroupInterface &gripper_interface,
                    const Block &object,
                    const Slot &slot,
                    double yaw)
  {
    const double SAFE_Z = 0.40;
    // 현재 자세
    geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
    geometry_msgs::msg::Pose lift_pose = current_pose;
    lift_pose.position.z = SAFE_Z;

    // 접근 자세 (슬롯 위 안전 높이)
    geometry_msgs::msg::Pose above_pose =
        list_to_pose(slot.location.x, slot.location.y, SAFE_Z,
                     M_PI, 0.0, yaw);

    // 실제 놓을 위치의 자세
    // slot.height는 case 바닥 기준 높이. 오브젝트 높이를 포함해 판 위에 놓이도록 설정
    geometry_msgs::msg::Pose place_pose =
        list_to_pose(slot.location.x, slot.location.y,
                     slot.height + object.height + 0.185,
                     M_PI, 0.0, yaw);

    std::vector<geometry_msgs::msg::Pose> waypoints1;
    std::vector<geometry_msgs::msg::Pose> waypoints2;
    std::vector<geometry_msgs::msg::Pose> waypoints_up;

    // 0) 수직 상승하여 안전 높이 도달
    if (current_pose.position.z < SAFE_Z - 1e-3) {
      waypoints_up.clear();
      waypoints_up.push_back(lift_pose);
      waypoint_sample(arm_interface, node, waypoints_up);
    }

    // 1) 같은 Z에서 슬롯 위로 수평 이동 (카르테시안)
    waypoints1.clear();
    waypoints1.push_back(above_pose);
    waypoint_sample(arm_interface, node, waypoints1);
    // 1-2) 슬롯 위에서 수직 하강 (카르테시안)
    waypoints1.clear();
    waypoints1.push_back(place_pose);
    waypoint_sample(arm_interface, node, waypoints1);

    // 2) 물체 내려놓기
    open_gripper(gripper_interface);

    // 3) 다시 위로 올라가기
    waypoints2.push_back(above_pose);
    waypoint_sample(arm_interface, node, waypoints2);
  }

  // 블록 1: box1 (0.40, 0.10)
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

  // 블록 2: box2 (0.40, 0.00)
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

  // 블록 3: box3 (0.40, -0.10) 얇은 Y(0.025)
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

  // 블록 4: box5 (0.50, 0.10) 높이 0.08
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

  // 블록 5: cylinder (0.50, 0.00) r=0.025, h=0.06
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

  // 블록 6: box4 (0.50, -0.10) 높이 0.07
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

  // 블록 7: triangle (0.60, 0.10) 메쉬, 높이 0.06 가정
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
    // pick에서 적용한 X 오프셋만큼 엔드이펙터 기준이 이동되어 있으므로
    // 동일한 X 오프셋을 place 목표에도 적용하여 물체 중심이 슬롯 중심에 놓이도록 보정
    Slot place_slot = slot;
    place_slot.location.x += 0.00625;
    place_object(node, arm_interface, gripper_interface, object, place_slot, yaw_adjusted_place);
  }

  // 블록 8: box6 (0.60, 0.00) 높이 0.09
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
