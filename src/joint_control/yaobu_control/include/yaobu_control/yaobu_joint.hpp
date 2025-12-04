#pragma once

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "lx_motor_interfaces/msg/motor_status.hpp"
#include "visibility_control.h"
#include "yaobu_control_parameters.hpp"

namespace yaobu_control {
class YaoBuSystemHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(YaoBuSystemHardware)

  YAOBU_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  YAOBU_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  YAOBU_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  YAOBU_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  YAOBU_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  YAOBU_CONTROL_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  YAOBU_CONTROL_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::CallbackReturn read_configure(
    const hardware_interface::HardwareInfo& info);

 private:
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  std::vector<double> hw_positions_;
  std::shared_ptr<rclcpp::Node> node_;
  // subscribe crawler feed back
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<lx_motor_interfaces::msg::MotorStatus>::SharedPtr
      fb_subscriber_ = nullptr;
  realtime_tools::RealtimeBox<std::shared_ptr<lx_motor_interfaces::msg::MotorStatus>>
      received_fb_msg_ptr_{nullptr};
  // publishe crawler conmand
  struct CmdPublisher {
    std::shared_ptr<
        realtime_tools::RealtimePublisher<lx_motor_interfaces::msg::MotorStatus>>
        realtime_cmd_publisher_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<lx_motor_interfaces::msg::MotorStatus>>
        cmd_publisher = nullptr;
  };
  CmdPublisher cmd_;


  // Parameters from ROS for diff_test_controller
  std::shared_ptr<yaobu_control::ParamListener> param_listener_;
  yaobu_control::Params params_;
};

}  // namespace yaobu_control