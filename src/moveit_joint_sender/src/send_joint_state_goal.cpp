#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <optional>
#include <moveit/robot_state/robot_state.hpp>

namespace fs = std::filesystem;

class JointStateExecutor : public rclcpp::Node
{
public:
  JointStateExecutor()
    : Node("joint_state_executor")
  {
    RCLCPP_INFO(this->get_logger(), "JointStateExecutor constructed.");
  }

  bool is_gripper_joint(const std::string& name) const
  {
    return name == "robotiq_85_left_knuckle_joint";
  }

  void init_move_groups()
  {
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "manipulator");

    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "gripper");

    arm_group_->startStateMonitor();
    gripper_group_->startStateMonitor();

    arm_group_->setPlanningTime(5.0);
    arm_group_->setNumPlanningAttempts(5);
    arm_group_->setMaxVelocityScalingFactor(0.2);
    arm_group_->setMaxAccelerationScalingFactor(0.2);

    gripper_group_->setPlanningTime(2.0);
    gripper_group_->setMaxVelocityScalingFactor(0.5);
    gripper_group_->setMaxAccelerationScalingFactor(0.5);

    RCLCPP_INFO(this->get_logger(),
                "MoveGroups initialized: manipulator + gripper");
  }

  void execute_from_yaml(const std::string& file_path)
  {
    RCLCPP_INFO(this->get_logger(), "Loading YAML: %s", file_path.c_str());
    YAML::Node data = YAML::LoadFile(file_path);

    if (!data["joint_names"] || !data["positions"])
    {
      RCLCPP_ERROR(this->get_logger(), "Missing fields in YAML");
      return;
    }

    auto names = data["joint_names"].as<std::vector<std::string>>();
    auto positions = data["positions"].as<std::vector<double>>();
    double wait_after = 0.0;
    if (data["wait_after"])
    {
      wait_after = data["wait_after"].as<double>();
    }

    if (names.size() != positions.size())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "joint_names and positions sizes differ");
      return;
    }

    std::map<std::string, double> arm_targets;
    std::map<std::string, double> gripper_targets;

    for (size_t i = 0; i < names.size(); ++i)
    {
      const auto& joint = names[i];
      double value = positions[i];

      if (is_gripper_joint(joint))
      {
        gripper_targets[joint] = std::clamp(value, 0.002, 0.082);
      }
      else
      {
        arm_targets[joint] = value;
      }
    }

    // ------------------------------
    //   EXECUTE GRIPPER FIRST
    // ------------------------------
    if (!gripper_targets.empty())
    {
      gripper_group_->setStartStateToCurrentState();
      gripper_group_->setJointValueTarget(gripper_targets);
      gripper_group_->move();   // blocking, no OMPL
      rclcpp::sleep_for(std::chrono::milliseconds(800));
    }

    // ------------------------------
    //   REFRESH STATE AFTER GRIPPER
    // ------------------------------
    // Give PlanningSceneMonitor time to sync gripper motion
    rclcpp::sleep_for(std::chrono::milliseconds(500));


    // ------------------------------
    //   EXECUTE ARM
    // ------------------------------
    if (arm_targets.empty())
    {
      RCLCPP_INFO(this->get_logger(), "No arm joints in this YAML, skipping arm plan");
      return;
    }

    if (!arm_targets.empty())
    {
      rclcpp::sleep_for(std::chrono::milliseconds(500));

      arm_group_->setStartStateToCurrentState();
      arm_group_->setJointValueTarget(arm_targets);

      moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
      auto arm_result = arm_group_->plan(arm_plan);

      if (arm_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Executing arm motion");
        arm_group_->execute(arm_plan);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Arm planning failed");
      }

    if (wait_after > 0.0)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting %.2f seconds after execution", wait_after);
      rclcpp::sleep_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(wait_after)
        )
      );

    }

    }
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
};

// ---------------------- Helper ----------------------
std::vector<std::string> get_yaml_files_for_task(const std::string& task_path,
                                                 int task_number)
{
  std::vector<std::string> files;

  for (int i = 0;; ++i)
  {
    std::string file = task_path + "/task" +
                       std::to_string(task_number) +
                       "_position" +
                       std::to_string(i) +
                       ".yaml";

    if (!fs::exists(file))
      break;

    files.push_back(file);
  }

  return files;
}

// ---------------------- main ----------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2)
  {
    std::cerr << "Usage: send_joint_state_goal <task_number>" << std::endl;
    return 1;
  }

  int task_number = std::stoi(argv[1]);
  std::string base_path = "/home/pascal/kinova_joint_data/";
  std::string task_dir = base_path + "task" + std::to_string(task_number);

  if (!fs::exists(task_dir))
  {
    std::cerr << "Task folder does not exist: " << task_dir << std::endl;
    return 1;
  }

  auto yaml_files = get_yaml_files_for_task(task_dir, task_number);
  if (yaml_files.empty())
  {
    std::cerr << "No YAML files found in " << task_dir << std::endl;
    return 1;
  }

  auto node = std::make_shared<JointStateExecutor>();
  node->init_move_groups();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);

  std::thread spinner([&exec]() { exec.spin(); });

  // Let joint_state + MoveIt sync
  rclcpp::sleep_for(std::chrono::seconds(2));

  for (const auto& file : yaml_files)
  {
    RCLCPP_INFO(node->get_logger(), "Executing: %s", file.c_str());
    node->execute_from_yaml(file);
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
