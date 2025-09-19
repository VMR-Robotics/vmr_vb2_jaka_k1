#include "yaobu_joint.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace yaobu_control {

#define DEV_INFO(...) \
  RCLCPP_INFO(rclcpp::get_logger("YaoBuSystemHardware"), __VA_ARGS__)

#define DEV_FATAL(...) \
  RCLCPP_INFO(rclcpp::get_logger("YaoBuSystemHardware"), __VA_ARGS__)

hardware_interface::CallbackReturn YaoBuSystemHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (read_configure(info) == hardware_interface::CallbackReturn::ERROR) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() != 1) {
    DEV_FATAL(
        "The number of crawlers is set not correactly,please check "
        "your urdf. 1 is expected,but now it is %zu.",
        info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  } else {
    DEV_INFO("Found %zu crawlers sucessfully!", info_.joints.size());
  }

  hw_positions_.resize(info_.joints.size(),
                       std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    DEV_INFO("joint name:%s", joint.name.c_str());
    // DiffBotSystem has exactly two states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1) {
      DEV_FATAL("Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      DEV_FATAL("Joint '%s' have '%s' as first state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn YaoBuSystemHardware::read_configure(
    const hardware_interface::HardwareInfo& info) {
  // Option 2: Load parameters in on_configure (Recommended)

  DEV_INFO("Configuring ...");

  try {
    // Instantiate the generated parameter class using the provided node pointer
    {
      auto it_param1 = info.hardware_parameters.find("state_topic");
      if (it_param1 != info.hardware_parameters.end()) {
        params_.state_topic = it_param1->second;
      } else {
        params_.state_topic = "";
      }
      DEV_INFO("Got state_topic: '%s'", params_.state_topic.c_str());
    }
    {
      auto it_param1 = info.hardware_parameters.find("v_control_topic");
      if (it_param1 != info.hardware_parameters.end()) {
        params_.v_control_topic = it_param1->second;
      } else {
        params_.v_control_topic = "";
      }
      DEV_INFO("Got v_control_topic: '%s'", params_.v_control_topic.c_str());
    }
    {
      auto it_param1 = info.hardware_parameters.find("p_control_topic");
      if (it_param1 != info.hardware_parameters.end()) {
        params_.p_control_topic = it_param1->second;
      } else {
        params_.p_control_topic = "";
      }
      DEV_INFO("Got p_control_topic: '%s'", params_.p_control_topic.c_str());
    }

    // Access loaded parameters for configuration logic
    DEV_INFO("Loaded param 'state_topic': %s", params_.state_topic.c_str());
    DEV_INFO("Loaded param 'v_control_topic': %s",
             params_.v_control_topic.c_str());
    DEV_INFO("Loaded param 'p_control_topic': %s",
             params_.p_control_topic.c_str());
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
      if (joint.state_interfaces.size() != 2) {
        DEV_FATAL("Joint '%s' has %zu state interface. 1 expected.",
                  joint.name.c_str(), joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (params_.v_control_topic != "" &&
          joint.command_interfaces[0].name !=
              hardware_interface::HW_IF_VELOCITY) {
        DEV_FATAL("Joint '%s' have %s command interfaces found. '%s' expected.",
                  joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                  hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (params_.p_control_topic != "" &&
          joint.command_interfaces[0].name !=
              hardware_interface::HW_IF_POSITION) {
        DEV_FATAL("Joint '%s' have %s command interfaces found. '%s' expected.",
                  joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                  hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  } catch (const std::exception& e) {
    DEV_FATAL("An unexpected error occurred during parameter loading: %s",
              e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  DEV_INFO("Configuration successful.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
YaoBuSystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
YaoBuSystemHardware::export_command_interfaces() {
  DEV_INFO("公布cmd:%ld", info_.joints.size());
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto ci = 0u; ci < info_.joints.size(); ci++) {
    auto cmd_type = hardware_interface::HW_IF_POSITION;
    if (params_.p_control_topic == "") {
      cmd_type = hardware_interface::HW_IF_VELOCITY;
    }
    DEV_INFO("公布cmd cmd_type:%s name:%s", cmd_type,
             info_.joints[ci].name.c_str());
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[ci].name, cmd_type, &hw_commands_[ci]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn YaoBuSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  DEV_INFO("Activating ...please wait...");
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }
  subscriber_is_active_ = true;

  this->node_ = std::make_shared<rclcpp::Node>("hardware_node");


  received_fb_msg_ptr_.set(
      std::make_shared<lx_motor_interfaces::msg::MotorStatus>());

  fb_subscriber_ =
      this->node_->create_subscription<lx_motor_interfaces::msg::MotorStatus>(
          params_.state_topic, rclcpp::SystemDefaultsQoS(),
          [this](const std::shared_ptr<lx_motor_interfaces::msg::MotorStatus> msg)
              -> void {
            //DEV_INFO("recv data:%lf", msg->data[6]);
            if (!subscriber_is_active_) {
              RCLCPP_WARN(this->node_->get_logger(),
                          "Can't accept new commands. subscriber is inactive");
              return;
            }
            received_fb_msg_ptr_.set(std::move(msg));
          });

  // 创建实时Publisher
  auto create_cmd = [this](const std::string& topic) -> CmdPublisher {
    CmdPublisher ret;
    ret.cmd_publisher =
        this->node_->create_publisher<lx_motor_interfaces::msg::MotorStatus>(
            topic, rclcpp::SystemDefaultsQoS());
    ret.realtime_cmd_publisher_ = std::make_shared<
        realtime_tools::RealtimePublisher<lx_motor_interfaces::msg::MotorStatus>>(
        ret.cmd_publisher);
    return ret;
  };
  if (params_.p_control_topic != "") {
    cmd_ = create_cmd(params_.p_control_topic);
  } else {
    cmd_ = create_cmd(params_.v_control_topic);
  }

  DEV_INFO("Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn YaoBuSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  DEV_INFO("Deactivating ...please wait...");

  subscriber_is_active_ = false;
  fb_subscriber_.reset();
  received_fb_msg_ptr_.set(nullptr);
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  DEV_INFO("Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type yaobu_control::YaoBuSystemHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  std::shared_ptr<lx_motor_interfaces::msg::MotorStatus> fb_msg;
  received_fb_msg_ptr_.get(fb_msg);
  rclcpp::spin_some(this->node_);
  for (std::size_t pi = 0; pi < hw_positions_.size(); pi++) {
    hw_positions_[pi] = fb_msg->data[pi];
    hw_velocities_[pi] = 0; // fb_msg->data[pi * 2 + 1];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type yaobu_control::YaoBuSystemHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (cmd_.realtime_cmd_publisher_->trylock()) {
    auto& cmd_msg = cmd_.realtime_cmd_publisher_->msg_;
    // cmd_msg.data.resize(hw_commands_.size());
    for (auto i = 0u; i < hw_commands_.size(); i++) {
      cmd_msg.data[i] = hw_commands_[i];
    }
    // DEV_INFO("topic:%s recv cmd:%lf", params_.p_control_topic.c_str(), cmd_msg.data[hw_commands_.size() - 1]);
    cmd_.realtime_cmd_publisher_->unlockAndPublish();
  }

  return hardware_interface::return_type::OK;
}

}  // namespace yaobu_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(yaobu_control::YaoBuSystemHardware,
                       hardware_interface::SystemInterface)
