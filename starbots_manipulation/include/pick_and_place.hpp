#ifndef PICK_AND_PLACE_HPP
#define PICK_AND_PLACE_HPP

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <starbots_detection_msgs/msg/detected_cupholder.hpp>
#include <starbots_detection_msgs/msg/detected_cupholders.hpp>
#include <starbots_detection_msgs/msg/detected_objects.hpp>
#include <starbots_detection_msgs/msg/detected_surfaces.hpp>
#include <starbots_manipulation/srv/deliver_cup.hpp>
#include <std_srvs/srv/empty.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using DeliverCup = starbots_manipulation::srv::DeliverCup;

class PickAndPlace : public rclcpp::Node {
public:
  PickAndPlace(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions{});
  ~PickAndPlace();

  bool gotoPredefined(std::string pose_name);
  void holeDetectionCallback(
      const starbots_detection_msgs::msg::DetectedCupholders::SharedPtr msg);
  bool executeKinematicsPlan(float pos_x, float pos_y, float pos_z);
  void executeCartesianPlan(float x_delta, float y_delta, float z_delta);
  void executeGripperPlan(std::string pose_name);
  void gripperGrasp();
  void attachCollisionObject(const std::string &object_id);
  void detachCollisionObject(const std::string &object_id);
  void createSceneObjects();
  moveit_msgs::msg::CollisionObject
  createCollisionObject(const std::string &object_id, const std::string &type,
                        const std::vector<double> &dimensions,
                        const geometry_msgs::msg::Pose &pose);
  void createTrajectoryConstraint();
  void createOrientationConstraint();
  void clearOrientationConstraints();
  void clearOctomap();

  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<MoveGroupInterface> move_group_robot_, move_group_gripper_;
  const moveit::core::JointModelGroup *joint_model_group_robot_,
      *joint_model_group_gripper_;
  moveit::core::RobotStatePtr robot_current_state_, gripper_current_state_;
  std::vector<double> joint_group_positions_robot_,
      joint_group_positions_gripper_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octo_client_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      constraint_marker_pub_;
  rclcpp::Subscription<starbots_detection_msgs::msg::DetectedCupholders>::
      SharedPtr holepose_sub_;
  rclcpp::Service<DeliverCup>::SharedPtr service_server_;
  moveit_msgs::msg::Constraints path_constraints_;
  geometry_msgs::msg::Point cup_position_;
  std::vector<geometry_msgs::msg::Point> goal_poses_;
  bool goal_poses_received_;
};

#endif // PICK_AND_PLACE_HPP
