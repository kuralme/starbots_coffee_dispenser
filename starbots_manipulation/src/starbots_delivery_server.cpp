#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include <chrono>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <starbots_detection_msgs/msg/detected_cupholder.hpp>
#include <starbots_detection_msgs/msg/detected_cupholders.hpp>
#include <starbots_detection_msgs/msg/detected_objects.hpp>
#include <starbots_detection_msgs/msg/detected_surfaces.hpp>
#include <starbots_manipulation/srv/deliver_cup.hpp>

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
  PickAndPlace(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("starbots_delivery_service_server", node_options),
        goal_poses_received_(false) {

    RCLCPP_INFO(LOGGER, "Initializing Starbots UR3e Coffee Dispenser...");
    this->declare_parameter("automatically_declare_parameters_from_overrides",
                            true);

    move_group_node_ =
        rclcpp::Node::make_shared("ur3e_move_group_node", node_options);

    // start move_group node in a seperate thread and spin it
    auto move_group_executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    move_group_executor->add_node(move_group_node_);
    std::thread([move_group_executor]() {
      move_group_executor->spin();
    }).detach();

    // initialize move_group interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    // get initial state of robot and gripper
    move_group_robot_->startStateMonitor();
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // get current state of robot and gripper
    robot_current_state_ = move_group_robot_->getCurrentState(10);
    robot_current_state_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    gripper_current_state_ = move_group_gripper_->getCurrentState(10);
    gripper_current_state_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);
    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();
    move_group_robot_->setPlanningTime(10.0);

    // Prepare ROS2 communiation
    rclcpp::CallbackGroup::SharedPtr callback_group;
    callback_group = move_group_node_->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group;

    holepose_sub_ = move_group_node_->create_subscription<
        starbots_detection_msgs::msg::DetectedCupholders>(
        "/cup_holder_detected", 100,
        std::bind(&PickAndPlace::holeDetectionCallback, this, _1), sub_options);
    service_server_ = move_group_node_->create_service<DeliverCup>(
        "/deliver_coffee",
        std::bind(&PickAndPlace::handleService, this, _1, _2),
        rmw_qos_profile_services_default, callback_group);
    octo_client_ =
        move_group_node_->create_client<std_srvs::srv::Empty>("/clear_octomap");

    createSceneObjects();
    gotoPredefined("quick_pick");
    RCLCPP_INFO(LOGGER, "UR3e ready for coffee delivery");
  }
  ~PickAndPlace() {
    RCLCPP_INFO(LOGGER, "Class Terminated: UR3e Coffee Dispenser");
  }

private:
  void gotoPredefined(std::string pose_name) {
    // Move robot(joints) to predefined home configuration
    RCLCPP_INFO(LOGGER, "Going to '%s' Pose...", pose_name.c_str());
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory...");
    move_group_robot_->setNamedTarget(pose_name);

    MoveGroupInterface::Plan kinematics_trajectory_plan;
    bool plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan) ==
         moveit::core::MoveItErrorCode::SUCCESS);
    if (plan_success_robot_) {
      size_t num_points =
          kinematics_trajectory_plan.trajectory_.joint_trajectory.points.size();
      if (num_points > 1) {
        move_group_robot_->execute(kinematics_trajectory_plan);
      } else {
        RCLCPP_INFO(LOGGER, "Already at the '%s' pose.", pose_name.c_str());
      }
    }
  }
  void handleService(const std::shared_ptr<DeliverCup::Request> request,
                     std::shared_ptr<DeliverCup::Response> response) {

    RCLCPP_INFO(LOGGER, "Coffee Delivery Requested");
    RCLCPP_INFO(LOGGER, "Goal cup holder on tray: %d",
                request->goal_cup_holder);

    // ============ Receive detection phase =================
    geometry_msgs::msg::Point pregrasp_pos;
    pregrasp_pos.x = 0.23;
    pregrasp_pos.y = 0.32; // actual 0.33
    pregrasp_pos.z = 0.37;

    int no_detection_count{0};
    while (!goal_poses_received_) {
      RCLCPP_WARN(LOGGER, "Cup Holder Poses not received yet!");
      std::this_thread::sleep_for(std::chrono::seconds(3));
      if (no_detection_count > 5) {
        response->result = "Coffee Delivery failed";
        return;
      }
      no_detection_count++;
    }
    auto goal_position = goal_poses_[request->goal_cup_holder];
    goal_position.x += .08;
    goal_position.y -= .11;
    goal_position.z += .4;

    try {
      // ============ Pregrasp phase =======================
      RCLCPP_INFO(LOGGER, "Going to Pre-grasp Position: [%.3f, %.3f, %.3f]",
                  pregrasp_pos.x, pregrasp_pos.y, pregrasp_pos.z);
      setupPoseTarget(pregrasp_pos.x, pregrasp_pos.y, pregrasp_pos.z, -1.000,
                      0.000, 0.000, 0.000);
      executeKinematicsPlan();

      // ============ Picking phase ========================
      RCLCPP_INFO(LOGGER, "Opening Gripper...");
      executeGripperPlan("gripper_open");

      RCLCPP_INFO(LOGGER, "Approaching to grasp...");
      executeCartesianPlan(+0.000, +0.000, -0.07, pregrasp_pos);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      // closeGripperIncremental();
      executeGripperPlan("gripper_grasp");
      attachObject("coffee_cup", pregrasp_pos);

      RCLCPP_INFO(LOGGER, "Retreating...");
      executeCartesianPlan(+0.000, +0.000, +0.100, pregrasp_pos);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // ============ Placing phase =========================
      createOrientationConstraint();
      RCLCPP_INFO(LOGGER, "Going to the Pre-drop Position: [%.3f, %.3f, %.3f]",
                  goal_position.x, goal_position.y, goal_position.z);
      setupPoseTarget(goal_position.x, goal_position.y, goal_position.z, -1.000,
                      0.000, 0.000, 0.000);
      executeKinematicsPlan();
      clearOrientationConstraints();

      std::this_thread::sleep_for(std::chrono::seconds(5));

      RCLCPP_INFO(LOGGER, "Approaching to place...");
      executeCartesianPlan(+0.000, +0.000, -0.08, goal_position);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      executeGripperPlan("gripper_open");
      detachObject("coffee_cup");
      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      std::this_thread::sleep_for(std::chrono::seconds(7));

      RCLCPP_INFO(LOGGER, "Retreating...");
      executeCartesianPlan(+0.000, +0.000, +0.100, goal_position);

      if (rclcpp::ok()) {
        response->result = "Coffee Delivery successful";
      }

      // ============ Back to initial phase ==================
      gotoPredefined("quick_pick");
      goal_poses_received_ = false;
      RCLCPP_INFO(LOGGER, "Coffee Delivery Completed");

    } catch (const std::exception &e) {
      RCLCPP_ERROR(LOGGER, "Error during Coffee Delivery execution: %s",
                   e.what());
      response->result = "Coffee Delivery failed";
    }
  }

  void holeDetectionCallback(
      const starbots_detection_msgs::msg::DetectedCupholders::SharedPtr msg) {
    if (!goal_poses_received_) {
      for (const auto &cupholder : msg->cup_holders) {
        goal_poses_.push_back(cupholder.position);
        // RCLCPP_INFO(LOGGER, "===========================");
        // RCLCPP_INFO(LOGGER, "Cupholder ID: %u", cupholder.cupholder_id);
        // RCLCPP_INFO(LOGGER, "Position: (%.2f, %.2f, %.2f)",
        // cupholder.position.x, cupholder.position.y,
        // cupholder.position.z);
        // RCLCPP_INFO(LOGGER, "Radius: %.2f", cupholder.radius);
        // RCLCPP_INFO(LOGGER, "Height: %.2f", cupholder.height);
      }
      //   RCLCPP_INFO(LOGGER, "===========================");
      goal_poses_received_ = true;
    }
  }

  void setupJointTarget(float angle0, float angle1, float angle2, float angle3,
                        float angle4, float angle5) {
    // set the joint values for each joint of robot arm
    joint_group_positions_robot_[0] = angle0; // Shoulder Pan
    joint_group_positions_robot_[1] = angle1; // Shoulder Lift
    joint_group_positions_robot_[2] = angle2; // Elbow
    joint_group_positions_robot_[3] = angle3; // Wrist 1
    joint_group_positions_robot_[4] = angle4; // Wrist 2
    joint_group_positions_robot_[5] = angle5; // Wrist 3
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }
  void setupPoseTarget(float pos_x, float pos_y, float pos_z, float quat_x,
                       float quat_y, float quat_z, float quat_w) {
    // set the pose values for end effector of robot arm
    geometry_msgs::msg::Pose target_pose_robot;
    target_pose_robot.position.x = pos_x;
    target_pose_robot.position.y = pos_y;
    target_pose_robot.position.z = pos_z;
    target_pose_robot.orientation.x = quat_x;
    target_pose_robot.orientation.y = quat_y;
    target_pose_robot.orientation.z = quat_z;
    target_pose_robot.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot);
  }
  void executeKinematicsPlan() {
    // move_group_robot_->setStartStateToCurrentState();
    MoveGroupInterface::Plan kinematics_trajectory_plan;
    bool plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan) ==
         moveit::core::MoveItErrorCode::SUCCESS);

    if (plan_success_robot_) {
      size_t num_points =
          kinematics_trajectory_plan.trajectory_.joint_trajectory.points.size();
      RCLCPP_INFO(LOGGER, "Trajectory has %zu points", num_points);
      if (num_points > 10) {
        move_group_robot_->execute(kinematics_trajectory_plan);
        RCLCPP_INFO(LOGGER, "Executed trajectory.");
      } else {
        gotoPredefined("quick_pick"); // Return to initial pose
        throw std::runtime_error("Trajectory too short — execution failed");
      }
    } else {
      gotoPredefined("quick_pick"); // Return to initial pose
      throw std::runtime_error("Kinematic Planning failed — execution failed");
    }
  }
  void executeCartesianPlan(float x_delta, float y_delta, float z_delta,
                            geometry_msgs::msg::Point target_position) {

    geometry_msgs::msg::Pose target_pose_robot;
    target_pose_robot.position = target_position;
    target_pose_robot.orientation.x = -1.0;
    target_pose_robot.orientation.y = 0.0;
    target_pose_robot.orientation.z = 0.0;
    target_pose_robot.orientation.w = 0.0;

    std::vector<geometry_msgs::msg::Pose> cartesian_waypoints;
    cartesian_waypoints.push_back(target_pose_robot);
    target_pose_robot.position.x += x_delta;
    target_pose_robot.position.y += y_delta;
    target_pose_robot.position.z += z_delta;
    cartesian_waypoints.push_back(target_pose_robot);

    RCLCPP_INFO(LOGGER, "Plan & Execute Cartesian Trajectory...");
    const double jump_threshold_ = 0.0;
    const double end_effector_step_ = 0.01;
    moveit_msgs::msg::RobotTrajectory cartesian_trajectory_plan_;
    double plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);

    // Check plan: 0.0 to 1.0 = success and -1.0 = failure
    if (plan_fraction_robot_ >= 0.0) {
      move_group_robot_->setStartStateToCurrentState();
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Cartesian Trajectory Success !");
    } else {
      gotoPredefined("quick_pick"); // Return to initial pose
      throw std::runtime_error("Cartesian planning failed !");
    }
    cartesian_waypoints.clear();
  }
  void executeGripperPlan(std::string pose_name) {
    MoveGroupInterface::Plan gripper_trajectory_plan;
    move_group_gripper_->setNamedTarget(pose_name);

    bool plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan) ==
         moveit::core::MoveItErrorCode::SUCCESS);

    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan);
      RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
    }
  }
  void closeGripperIncremental() {
    float gripper_value = 0.4;
    const float target_value = 0.0; // ~gripper_grasp
    const float step_large = 0.17;
    const float step_medium = 0.07;

    while (gripper_value > target_value) {
      // Dynamically reduce step size for precision
      if (gripper_value > 0.1)
        gripper_value -= step_large;
      else if (gripper_value > 0.05)
        gripper_value -= step_medium;

      joint_group_positions_gripper_[0] = gripper_value;
      move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);

      MoveGroupInterface::Plan plan;
      bool success = move_group_gripper_->plan(plan) ==
                     moveit::core::MoveItErrorCode::SUCCESS;

      if (success) {
        move_group_gripper_->execute(plan);
      } else {
        RCLCPP_ERROR(LOGGER, "Failed to plan gripper motion at value: %.3f",
                     gripper_value);
        break;
      }
    }
  }

  void attachObject(const std::string &object_id,
                    geometry_msgs::msg::Point pregrasp_pos) {
    // Add the cup to the planning scene
    geometry_msgs::msg::Pose cup_pose;
    cup_pose.position = pregrasp_pos;
    cup_pose.position.z -= 0.3;
    std::vector<double> cup_dimensions = {0.11, 0.04}; // {height, width}
    moveit_msgs::msg::CollisionObject coffee_cup =
        createCollisionObject(object_id, "cylinder", cup_dimensions, cup_pose);

    // Attach the cup to gripper
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = move_group_robot_->getEndEffectorLink();
    attached_object.object = coffee_cup;
    attached_object.touch_links = {"rg2_gripper_right_thumb",
                                   "rg2_gripper_left_thumb"};
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyAttachedCollisionObject(attached_object);

    RCLCPP_INFO(LOGGER, "Cup attached to gripper");
  }
  void detachObject(const std::string &object_id) {
    // Detach the cup from the gripper
    moveit_msgs::msg::AttachedCollisionObject detach_object;
    detach_object.link_name = move_group_robot_->getEndEffectorLink();
    detach_object.object.id = object_id;
    detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyAttachedCollisionObject(detach_object);

    // Remove the cup from the world
    planning_scene_interface.removeCollisionObjects({object_id});
    RCLCPP_INFO(LOGGER, "Removed cup from planning scene");

    RCLCPP_INFO(LOGGER, "Cup detached from gripper");
  }
  void createSceneObjects() {
    // Create scene objects arent visible in pointcloud
    geometry_msgs::msg::Pose counter_pose;
    counter_pose.orientation.w = 1.0;
    counter_pose.position.x = 0.25;
    counter_pose.position.y = 0.25;
    counter_pose.position.z = -0.04;
    std::vector<double> box_dimensions = {0.7, 1.5,
                                          0.08}; // {length, width, height}
    const moveit_msgs::msg::CollisionObject coffee_counter =
        createCollisionObject("coffee_counter", "box", box_dimensions,
                              counter_pose);

    geometry_msgs::msg::Pose wall_pose;
    wall_pose.orientation.w = 1.0;
    wall_pose.position.x = 0.0;
    wall_pose.position.y = -0.5;
    wall_pose.position.z = 0.0;
    std::vector<double> wall_dimensions = {1.5, 0.1,
                                           1.5}; // {length, width, height}
    const moveit_msgs::msg::CollisionObject wall =
        createCollisionObject("wall", "box", wall_dimensions, wall_pose);

    geometry_msgs::msg::Pose machine_pose;
    machine_pose.orientation.w = 1.0;
    machine_pose.position.x = 0.3;
    machine_pose.position.y = 0.87;
    machine_pose.position.z = 0.14;
    std::vector<double> machine_dimensions = {0.5, 0.22,
                                              0.25}; // {length, width, height}
    const moveit_msgs::msg::CollisionObject coffee_machine =
        createCollisionObject("coffee_machine", "box", machine_dimensions,
                              machine_pose);

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(wall);
    collision_objects.push_back(coffee_counter);
    collision_objects.push_back(coffee_machine);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.addCollisionObjects(collision_objects);
    // Wait for MoveGroup to recieve and process the collision object message.
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  moveit_msgs::msg::CollisionObject createCollisionObject(
      const std::string &object_id,
      const std::string &type,               // Object type: "box" or "cylinder"
      const std::vector<double> &dimensions, // Dimensions of the object
      const geometry_msgs::msg::Pose &pose)  // Pose of the object
  {
    moveit_msgs::msg::CollisionObject collision_object;
    shape_msgs::msg::SolidPrimitive primitive;

    // Ensure that dimensions are correctly converted
    if (type == "box") {
      primitive.type = primitive.BOX;
      if (dimensions.size() == 3) {
        // Assuming dimensions are {length, width, height}
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = dimensions[0]; // length
        primitive.dimensions[1] = dimensions[1]; // width
        primitive.dimensions[2] = dimensions[2]; // height
      } else {
        RCLCPP_ERROR(
            LOGGER,
            "Box requires exactly 3 dimensions: {length, width, height}");
        return collision_object; // Return empty object in case of error
      }
    } else if (type == "cylinder") {
      primitive.type = primitive.CYLINDER;
      if (dimensions.size() == 2) {
        // Assuming dimensions are {height, radius}
        primitive.dimensions.resize(2);
        primitive.dimensions[primitive.CYLINDER_HEIGHT] =
            dimensions[0]; // height
        primitive.dimensions[primitive.CYLINDER_RADIUS] =
            dimensions[1]; // radius
      } else {
        RCLCPP_ERROR(
            LOGGER, "Cylinder requires exactly 2 dimensions: {height, radius}");
        return collision_object; // Return empty object in case of error
      }
    } else {
      RCLCPP_ERROR(LOGGER, "Unsupported object type: %s", type.c_str());
      return collision_object; // Return an empty object if type is unsupported
    }

    // Set up collision object properties
    collision_object.header.frame_id = move_group_robot_->getPlanningFrame();
    collision_object.id = object_id;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    RCLCPP_INFO(LOGGER, "Added %s into the scene", object_id.c_str());
    return collision_object;
  }

  void createOrientationConstraint() {
    // "Gripper pointing down" orientation constraint
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = move_group_robot_->getEndEffectorLink();
    ocm.header.frame_id = move_group_robot_->getPlanningFrame();
    ocm.orientation.x = -1.0;
    ocm.orientation.y = 0.0;
    ocm.orientation.z = 0.0;
    ocm.orientation.w = 0.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = M_PI;
    ocm.weight = 1.0;
    path_constraints_.orientation_constraints.push_back(ocm);

    // Changed planner for better orientation constrained planning
    move_group_robot_->setPlannerId("KPIECEkConfigDefault");
    move_group_robot_->setPathConstraints(path_constraints_);
    move_group_robot_->setStartStateToCurrentState();

    RCLCPP_INFO(LOGGER, "Applied upright orientation constraint (Z down)");
  }
  void clearOrientationConstraints() {
    // move_group_robot_->clearPathConstraints();
    path_constraints_.orientation_constraints.clear();
    move_group_robot_->setPathConstraints(path_constraints_);
    move_group_robot_->setPlannerId("APSConfigDefault");
    RCLCPP_INFO(LOGGER, "Cleared Orientation constraints");
  }

  // declare class variables
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<MoveGroupInterface> move_group_robot_, move_group_gripper_;
  const moveit::core::JointModelGroup *joint_model_group_robot_,
      *joint_model_group_gripper_;
  moveit::core::RobotStatePtr robot_current_state_, gripper_current_state_;
  std::vector<double> joint_group_positions_robot_,
      joint_group_positions_gripper_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      constraint_marker_pub_;
  rclcpp::Subscription<starbots_detection_msgs::msg::DetectedCupholders>::
      SharedPtr holepose_sub_;
  rclcpp::Service<DeliverCup>::SharedPtr service_server_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octo_client_;
  moveit_msgs::msg::Constraints path_constraints_;

  // declare detection variables
  std::vector<geometry_msgs::msg::Point> goal_poses_;
  bool goal_poses_received_;
}; // class PickAndPlace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto pick_and_place_as = std::make_shared<PickAndPlace>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pick_and_place_as);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}