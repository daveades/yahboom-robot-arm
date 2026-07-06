#include "dofbot_ros2_control/dofbot_topic_hardware.hpp"

#include <algorithm>

#include <hardware_interface/handle.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace dofbot_ros2_control {

hardware_interface::CallbackReturn DofbotTopicHardware::on_init(
    const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("command_topic") > 0) {
    command_topic_ = info_.hardware_parameters.at("command_topic");
  }
  if (info_.hardware_parameters.count("state_topic") > 0) {
    state_topic_ = info_.hardware_parameters.at("state_topic");
  }

  joint_names_.clear();
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  hw_states_.assign(joint_names_.size(), 0.0);
  hw_commands_.assign(joint_names_.size(), 0.0);

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  node_ = std::make_shared<rclcpp::Node>("dofbot_topic_hardware");
  command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      command_topic_, rclcpp::SystemDefaultsQoS());
  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      state_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&DofbotTopicHardware::state_callback, this, std::placeholders::_1));

  executor_.add_node(node_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DofbotTopicHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_names_[i],
                                            hardware_interface::HW_IF_POSITION,
                                            &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DofbotTopicHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_names_[i],
                                              hardware_interface::HW_IF_POSITION,
                                              &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn DofbotTopicHardware::on_configure(
    const rclcpp_lifecycle::State &) {
  hw_states_ = hw_commands_;
  have_state_ = false;
  commands_initialized_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DofbotTopicHardware::on_activate(
    const rclcpp_lifecycle::State &) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DofbotTopicHardware::on_deactivate(
    const rclcpp_lifecycle::State &) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DofbotTopicHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &) {
  executor_.spin_some();

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (last_state_) {
    have_state_ = true;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(last_state_->name.begin(), last_state_->name.end(), joint_names_[i]);
      if (it != last_state_->name.end()) {
        size_t idx = static_cast<size_t>(std::distance(last_state_->name.begin(), it));
        if (idx < last_state_->position.size()) {
          hw_states_[i] = last_state_->position[idx];
        }
      }
    }
    if (!commands_initialized_) {
      // Initialize commands to current state to avoid sudden jumps on startup.
      hw_commands_ = hw_states_;
      commands_initialized_ = true;
    }
  } else {
    hw_states_ = hw_commands_;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DofbotTopicHardware::write(
    const rclcpp::Time & time, const rclcpp::Duration &) {
  if (!have_state_ || !commands_initialized_) {
    return hardware_interface::return_type::OK;
  }
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = time;
  msg.name = joint_names_;
  msg.position = hw_commands_;
  command_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

void DofbotTopicHardware::state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_state_ = msg;
}

}  // namespace dofbot_ros2_control

PLUGINLIB_EXPORT_CLASS(dofbot_ros2_control::DofbotTopicHardware,
                       hardware_interface::SystemInterface)
