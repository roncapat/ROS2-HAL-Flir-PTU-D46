#ifndef HAL_FLIR_D46
#define HAL_FLIR_D46

#include <iostream>
#include <chrono>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>

#include "ptu_interfaces/msg/ptu.hpp"
#include "ptu_interfaces/srv/set_pan.hpp"
#include "ptu_interfaces/srv/set_tilt.hpp"
#include "ptu_interfaces/srv/set_pan_tilt.hpp"
#include "ptu_interfaces/srv/set_pan_tilt_speed.hpp"
#include "ptu_interfaces/srv/get_limits.hpp"

#include "ptu_interfaces/action/set_pan.hpp"
#include "ptu_interfaces/action/set_tilt.hpp"
#include "ptu_interfaces/action/set_pan_tilt.hpp"

#include "hal_ptu_flir_d46/driver.h"
#include <serial/serial.h>

#include <chrono>
#include <cstdlib>
#include <memory>

namespace ph = std::placeholders;

namespace hal {

class FlirD46 : public rclcpp::Node {
 public:
  using SetPanAction = ptu_interfaces::action::SetPan;
  using GoalHandlePanAction = rclcpp_action::ServerGoalHandle<SetPanAction>;

  using SetTiltAction = ptu_interfaces::action::SetTilt;
  using GoalHandleTiltAction = rclcpp_action::ServerGoalHandle<SetTiltAction>;

  using SetPanTiltAction = ptu_interfaces::action::SetPanTilt;
  using GoalHandlePanTiltAction = rclcpp_action::ServerGoalHandle<SetPanTiltAction>;

  FlirD46(const rclcpp::NodeOptions & options);
  ~FlirD46();  

  void disconnect();

 private:
  std::shared_ptr<flir_ptu_driver::PTU> m_pantilt = nullptr;
  std::shared_ptr<serial::Serial> m_ser = nullptr;

  double default_velocity_;
  double pan_min, pan_max, tilt_min, tilt_max;
  double pan_speed_min, pan_speed_max, tilt_speed_min, tilt_speed_max;
  double pan_resolution, tilt_resolution;
  double min_threshold_to_move_pan, min_threshold_to_move_tilt;
  int hz;

  rclcpp::Publisher<ptu_interfaces::msg::PTU>::SharedPtr ptu_state_pub;

  rclcpp::Service<ptu_interfaces::srv::SetPan>::SharedPtr set_pan_srv;
  rclcpp::Service<ptu_interfaces::srv::SetTilt>::SharedPtr set_tilt_srv;
  rclcpp::Service<ptu_interfaces::srv::SetPanTilt>::SharedPtr set_pantilt_srv;
  rclcpp::Service<ptu_interfaces::srv::SetPanTiltSpeed>::SharedPtr set_pantilt_speed_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;
  rclcpp::Service<ptu_interfaces::srv::GetLimits>::SharedPtr get_limits_srv;

  rclcpp_action::Server<SetPanAction>::SharedPtr action_server_set_pan;
  rclcpp_action::Server<SetTiltAction>::SharedPtr action_server_set_tilt;
  rclcpp_action::Server<SetPanTiltAction>::SharedPtr action_server_set_pantilt;

  rclcpp::TimerBase::SharedPtr timer_;

  bool ok();
  void get_limits_callback(const std::shared_ptr<ptu_interfaces::srv::GetLimits::Request>,
          std::shared_ptr<ptu_interfaces::srv::GetLimits::Response> response);


	void resetCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
          std::shared_ptr<std_srvs::srv::Empty::Response>);

  void set_pan_callback(const std::shared_ptr<ptu_interfaces::srv::SetPan::Request>,
          std::shared_ptr<ptu_interfaces::srv::SetPan::Response>);


  void set_tilt_callback(const std::shared_ptr<ptu_interfaces::srv::SetTilt::Request>,
          std::shared_ptr<ptu_interfaces::srv::SetTilt::Response>);

  void set_pantilt_callback(const std::shared_ptr<ptu_interfaces::srv::SetPanTilt::Request>,
          std::shared_ptr<ptu_interfaces::srv::SetPanTilt::Response>);


  void set_pantilt_speed_callback(const std::shared_ptr<ptu_interfaces::srv::SetPanTiltSpeed::Request>,
        std::shared_ptr<ptu_interfaces::srv::SetPanTiltSpeed::Response>);

  void spinCallback();

  rclcpp_action::GoalResponse handle_goal_pan(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SetPanAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel_pan(
    const std::shared_ptr<GoalHandlePanAction> goal_handle);

  void handle_accepted_pan(const std::shared_ptr<GoalHandlePanAction> goal_handle);

  void execute_pan_action(const std::shared_ptr<GoalHandlePanAction> goal_handle);

  rclcpp_action::GoalResponse handle_goal_tilt(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SetTiltAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel_tilt(
    const std::shared_ptr<GoalHandleTiltAction> goal_handle);

  void handle_accepted_tilt(const std::shared_ptr<GoalHandleTiltAction> goal_handle);

  void execute_tilt_action(const std::shared_ptr<GoalHandleTiltAction> goal_handle);

  rclcpp_action::GoalResponse handle_goal_pantilt(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SetPanTiltAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel_pantilt(
    const std::shared_ptr<GoalHandlePanTiltAction> goal_handle);

  void handle_accepted_pantilt(const std::shared_ptr<GoalHandlePanTiltAction> goal_handle);
  
  void execute_pantilt_action(const std::shared_ptr<GoalHandlePanTiltAction> goal_handle);

};

} // namespace hal

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(hal::FlirD46)


#endif // HAL_FLIR_D46