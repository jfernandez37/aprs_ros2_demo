#include <motoman_demo/motoman_kitting.hpp>

MotomanDemo::MotomanDemo()
    : Node("motoman_demo"),
      motoman_arm_(std::shared_ptr<rclcpp::Node>(std::move(this)), "motoman_arm"),
      planning_scene_()
{
  // Use upper joint velocity and acceleration limits
  motoman_arm_.setMaxAccelerationScalingFactor(1.0);
  motoman_arm_.setMaxVelocityScalingFactor(1.0);
  motoman_arm_.setPlanningTime(10.0);
  motoman_arm_.setNumPlanningAttempts(5);
  motoman_arm_.allowReplanning(true);
  motoman_arm_.setReplanAttempts(5);

  open_gripper_client_ = this->create_client<example_interfaces::srv::Trigger>("gripper_open");
  close_gripper_client_ = this->create_client<example_interfaces::srv::Trigger>("gripper_close");
  update_antvision_data_client_ = this->create_client<example_interfaces::srv::Trigger>("update_antvision_data");

  motoman_position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

  trays_subscriber_ = this->create_subscription<aprs_interfaces::msg::Trays>("/trays_info", 10, bind(&MotomanDemo::TraysInfoCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

MotomanDemo::~MotomanDemo()
{
  motoman_arm_.~MoveGroupInterface();
}

void MotomanDemo::TraysInfoCallback(const aprs_interfaces::msg::Trays::SharedPtr msg)
{
  kit_trays_ = msg->kit_trays;
  part_trays_ = msg->part_trays;
}

void MotomanDemo::MotomanSendHome()
{
  motoman_arm_.setNamedTarget("home");
  auto plan = MotomanPlantoTarget();
  if (plan.first)
  {
    MotomanSendTrajectory(plan.second);
  }

}

std::pair<bool, moveit_msgs::msg::RobotTrajectory>  MotomanDemo::MotomanPlantoTarget()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(motoman_arm_.plan(plan));
  moveit_msgs::msg::RobotTrajectory trajectory = plan.trajectory;

  if (success)
  {
    return std::make_pair(true, trajectory);
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return std::make_pair(false, trajectory);
  }
}

std::pair<bool,moveit_msgs::msg::RobotTrajectory> MotomanDemo::MotomanPlanCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = motoman_arm_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

  if (path_fraction < 0.9)
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return std::make_pair(false, trajectory);
  }

  // Retime trajectory
  robot_trajectory::RobotTrajectory rt(motoman_arm_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*motoman_arm_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return std::make_pair(true, trajectory);
}

bool MotomanDemo::MotomanSendTrajectory(moveit_msgs::msg::RobotTrajectory trajectory)
{
  // Send Entire Trajectory
  /*
  for (auto point : trajectory.joint_trajectory.points)
  {
    std_msgs::msg::Float64MultiArray joint_positions;
    joint_positions.data = point.positions;
    motoman_position_publisher_->publish(joint_positions);
    usleep(100000);
    std::stringstream joint_position_ss;
    for (auto position : point.positions){
      joint_position_ss << position << " ";
    }
    RCLCPP_INFO(get_logger(),"Joint Positions: %s",joint_position_ss.str().c_str());
  }
  */

  // Send Last Point of Trajectory

  trajectory.joint_trajectory.points.back().positions[2] -= trajectory.joint_trajectory.points.back().positions[1];

  std_msgs::msg::Float64MultiArray joint_positions;
  trajectory.joint_trajectory.points.back().positions;
  joint_positions.data = trajectory.joint_trajectory.points.back().positions;
  motoman_position_publisher_->publish(joint_positions);
  usleep(100000);
  std::stringstream joint_position_ss;
  for (auto position : trajectory.joint_trajectory.points.back().positions){
    joint_position_ss << position << " ";
  }
  RCLCPP_INFO(get_logger(),"Joint Positions: %s",joint_position_ss.str().c_str());

  return true;
}

bool MotomanDemo::MoveToJoints(std::vector<double> joint_values)
{
  std_msgs::msg::Float64MultiArray joint_positions;
  joint_positions.data = joint_values;
  motoman_position_publisher_->publish(joint_positions);
 
  return true;
}

bool MotomanDemo::MotomanOpenGripper()
{
  auto request = std::make_shared<example_interfaces::srv::Trigger::Request>();
  auto result = open_gripper_client_->async_send_request(request);
  return result.get()->success;
}

bool MotomanDemo::MotomanCloseGripper()
{
  auto request = std::make_shared<example_interfaces::srv::Trigger::Request>();
  auto result = close_gripper_client_->async_send_request(request);
  return result.get()->success;
}

double MotomanDemo::calculateDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2) {
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  double dz = pose1.position.z - pose2.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool MotomanDemo::BuildTarget(){
  int joint_num = 1;
  for (auto joint_value: motoman_arm_.getCurrentJointValues()) {
     RCLCPP_INFO_STREAM(get_logger(), "Joint " << joint_num << " : " << joint_value);
     joint_num++;
  }

  // Perform Joint Space Planning

  geometry_msgs::msg::Pose robot_pose = motoman_arm_.getCurrentPose().pose;

  RCLCPP_INFO_STREAM(get_logger(),"The current Robot Pose is X: " << robot_pose.position.x << " Y: " << robot_pose.position.y << " Z: " << robot_pose.position.z);

  robot_pose.position.z = 0.06;
  robot_pose.position.x = -0.088;
  robot_pose.position.y = 0.651;

  RCLCPP_INFO_STREAM(get_logger(), "The New Target Robot Pose is X: "
                                           << robot_pose.position.x
                                           << " Y: " << robot_pose.position.y
                                           << " Z: " << robot_pose.position.z);

  motoman_arm_.setPoseTarget(robot_pose);
  auto plan = MotomanPlantoTarget();
  if (plan.first)
  {
    MotomanSendTrajectory(plan.second);
  }

  // Perform Cartesian Path Planning

  /*
  geometry_msgs::msg::Pose robot_pose = motoman_arm_.getCurrentPose().pose;

  RCLCPP_INFO_STREAM(get_logger(),"The current Robot Pose is X: " << robot_pose.position.x << " Y: " << robot_pose.position.y << " Z: " << robot_pose.position.z);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  robot_pose.position.x += 0.1;
  waypoints.push_back(robot_pose);

  auto plan = MotomanPlanCartesian(waypoints, 0.1, 0.1, true);
  if (plan.first)
  {
    MotomanSendTrajectory(plan.second);
  } 
  */
 
  return true;
}

bool MotomanDemo::PickPart(const std::string& gear_name)
{
  geometry_msgs::msg::TransformStamped pick_transform = tf_buffer->lookupTransform("motoman_base_link", gear_name, tf2::TimePointZero);

  RCLCPP_INFO_STREAM(get_logger(), "Picking part " << gear_name << " at X: " << pick_transform.transform.translation.x
                                                      << " Y: " << pick_transform.transform.translation.y
                                                      << " Z: " << pick_transform.transform.translation.z);

  geometry_msgs::msg::Pose robot_pose = motoman_arm_.getCurrentPose().pose;
  
  geometry_msgs::msg::Pose pick_pose;
  pick_pose.position.x = pick_transform.transform.translation.x;
  pick_pose.position.y = pick_transform.transform.translation.y;
  pick_pose.position.z = pick_transform.transform.translation.z + 0.05;
  pick_pose.orientation = robot_pose.orientation;

  motoman_arm_.setPoseTarget(pick_pose);
  auto plan = MotomanPlantoTarget();
  if (plan.first)
  {
    MotomanSendTrajectory(plan.second);
  }
  else
  {
    return false;
  }

  pick_pose.position.z -= 0.05;

  motoman_arm_.setPoseTarget(pick_pose);
  plan = MotomanPlantoTarget();
  if (plan.first)
  {
    MotomanSendTrajectory(plan.second);
  }
  else
  {
    return false;
  }



  robot_pose = motoman_arm_.getCurrentPose().pose;
  while (calculateDistance(robot_pose,pick_pose) > 0.003) {
    robot_pose = motoman_arm_.getCurrentPose().pose;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Moving to pick pose");
  }

  MotomanCloseGripper();

  pick_pose.position.z += 0.1;
  motoman_arm_.setPoseTarget(pick_pose);
  plan = MotomanPlantoTarget();
  if (plan.first)
  {
    MotomanSendTrajectory(plan.second);
  }
  

  return true;
}

bool MotomanDemo::PlacePart(const std::string& slot_name)
{
  geometry_msgs::msg::TransformStamped place_transform = tf_buffer->lookupTransform("motoman_base_link", slot_name, tf2::TimePointZero);

  RCLCPP_INFO_STREAM(get_logger(), "Placing part in slot " << slot_name << " at X: " << place_transform.transform.translation.x
                                                            << " Y: " << place_transform.transform.translation.y
                                                            << " Z: " << place_transform.transform.translation.z);

  geometry_msgs::msg::Pose robot_pose = motoman_arm_.getCurrentPose().pose;
  
  geometry_msgs::msg::Pose place_pose;
  place_pose.position.x = place_transform.transform.translation.x;
  place_pose.position.y = place_transform.transform.translation.y;
  place_pose.position.z = place_transform.transform.translation.z + 0.1;
  place_pose.orientation = robot_pose.orientation;

  motoman_arm_.setPoseTarget(place_pose);
  auto plan = MotomanPlantoTarget();
  if (plan.first)
  {
    MotomanSendTrajectory(plan.second);
  }
    else
  {
    return false;
  }

  place_pose.position.z -= 0.06;

  motoman_arm_.setPoseTarget(place_pose);
  plan = MotomanPlantoTarget();
  if (plan.first)
  {
    MotomanSendTrajectory(plan.second);
  }
  else
  {
    return false;
  }

  robot_pose = motoman_arm_.getCurrentPose().pose;
  while (calculateDistance(robot_pose,place_pose) > 0.003) {
    robot_pose = motoman_arm_.getCurrentPose().pose;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Moving to place pose");
  }

  MotomanOpenGripper();

  place_pose.position.z += 0.1;
  motoman_arm_.setPoseTarget(place_pose);
  plan = MotomanPlantoTarget();
  if (plan.first)
  {
    MotomanSendTrajectory(plan.second);
  }
  
  
  return true;
}

void MotomanDemo::FillKitSlots(std::vector<aprs_interfaces::msg::SlotInfo>& kit_tray_slots, const uint8_t part_type)
{
  for (auto& kit_tray_slot : kit_tray_slots)
  {
    if (!kit_tray_slot.occupied)
    {
      bool slot_filled = false;
      for (auto& part_tray : part_trays_)
      {
        if (part_tray.identifier == part_type)
        {
          for (auto& part_tray_slot : part_tray.slots)
          {
            if (part_tray_slot.occupied)
            {
              PickPart(part_tray_slot.name);
              PlacePart(kit_tray_slot.name);
              kit_tray_slot.occupied = true;
              part_tray_slot.occupied = false;
              slot_filled = true;
              break;
            }
          }
        }
        if (slot_filled)
        {
          break;
        }
      }
    }
  }
}

void MotomanDemo::EmptyKitSlots(std::vector<aprs_interfaces::msg::SlotInfo>& kit_tray_slots, const uint8_t part_type)
{
  for (auto& kit_tray_slot : kit_tray_slots)
  {
    if (kit_tray_slot.occupied)
    {
      bool slot_empty = false;
      for (auto& part_tray : part_trays_)
      {
        if (part_tray.identifier == part_type)
        {
          for (auto& part_tray_slot : part_tray.slots)
          {
            if (!part_tray_slot.occupied)
            {
              PickPart(kit_tray_slot.name);
              PlacePart(part_tray_slot.name);
              kit_tray_slot.occupied = false;
              part_tray_slot.occupied = true;
              slot_empty = true;
              break;
            }
          }
        }
        if (slot_empty)
        {
          break;
        }
      }
    }
  }
}

bool MotomanDemo::FillKitTray()
{
  for (auto& kit_tray : kit_trays_)
  {
    FillKitSlots(kit_tray.large_gear_slots, aprs_interfaces::msg::Object::LARGE_GEAR_TRAY);
    FillKitSlots(kit_tray.medium_gear_slots, aprs_interfaces::msg::Object::MEDIUM_GEAR_TRAY);
    FillKitSlots(kit_tray.small_gear_slots, aprs_interfaces::msg::Object::SMALL_GEAR_TRAY);
  }
  return true;
}

bool MotomanDemo::EmptyKitTray()
{
  for (auto& kit_tray : kit_trays_)
  {
    EmptyKitSlots(kit_tray.large_gear_slots, aprs_interfaces::msg::Object::LARGE_GEAR_TRAY);
    EmptyKitSlots(kit_tray.medium_gear_slots, aprs_interfaces::msg::Object::MEDIUM_GEAR_TRAY);
    EmptyKitSlots(kit_tray.small_gear_slots, aprs_interfaces::msg::Object::SMALL_GEAR_TRAY);
  }
  return true;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto motoman_demo = std::make_shared<MotomanDemo>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(motoman_demo);
  std::thread([&executor]() { executor.spin(); }).detach();

  sleep(3);

  // std::vector<double> joint_values{2.79,0.0,0.0,0.0,-1.54,0.0};
  
  // motoman_demo->MoveToJoints(joint_values);
  motoman_demo->MotomanSendHome();
  motoman_demo->FillKitTray();
  // motoman_demo->BuildTarget();

  rclcpp::shutdown();
}
