#include <iostream>
#include <chrono>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include <ros/console.h>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
//#include "hal_tof_mesa_sr4xxx/srv/move_ptu.hpp"
#include "hal_ptu_flir_d46/driver.h"
#include <serial/serial.h>

/* TODO
 * 
 * publish pan/tilt
 * subscribe pan/tilt
 * service set pan/tilt
 * service set pan/tilt blocking (flag?)
 * service get pan/tilt
 * */

using namespace std::chrono_literals;
namespace ph = std::placeholders;

class HALPTUFlirD46 : public rclcpp::Node {
 public:

  HALPTUFlirD46() : Node("HALPTUFlirD46") {
	  m_joint_name_prefix = declare_parameter("~joint_name_prefix", "ptu_");
  }
  
  bool init() {
	disconnect();

  // Query for serial configuration
  std::string port;
  int32_t baud;
  bool limit;
  port = declare_parameter("~port", PTU_DEFAULT_PORT);
  limit = declare_parameter("~limits_enabled", true);
  baud = declare_parameter("~baud", PTU_DEFAULT_BAUD);
  default_velocity_ = declare_parameter("~default_velocity", PTU_DEFAULT_VEL);

  // Connect to the PTU
  RCLCPP_INFO_STREAM(get_logger(), "Attempting to connect to FLIR PTU on " << port);

  try{
    m_ser.setPort(port);
    m_ser.setBaudrate(baud);
    serial::Timeout to = serial::Timeout(200, 200, 0, 200, 0);
    m_ser.setTimeout(to);
    m_ser.open();
  }
  catch (serial::IOException& e){
    RCLCPP_ERROR_STREAM(get_logger(), "Unable to open port " << port);
    return false;
  }

  RCLCPP_INFO_STREAM(get_logger(), "FLIR PTU serial port opened, now initializing.");

  m_pantilt = new flir_ptu_driver::PTU(&m_ser);

  if (!m_pantilt->initialize()){
    RCLCPP_ERROR_STREAM(get_logger(), "Could not initialize FLIR PTU on " << port);
    disconnect();
    return false;
  }

  if (!limit){
    m_pantilt->disableLimits();
    RCLCPP_INFO_STREAM(get_logger(), "FLIR PTU limits disabled.");
  }

  RCLCPP_INFO_STREAM(get_logger(), "FLIR PTU initialized.");

  declare_parameter("min_tilt", m_pantilt->getMin(PTU_TILT));
  declare_parameter("max_tilt", m_pantilt->getMax(PTU_TILT));
  declare_parameter("min_tilt_speed", m_pantilt->getMinSpeed(PTU_TILT));
  declare_parameter("max_tilt_speed", m_pantilt->getMaxSpeed(PTU_TILT));
  declare_parameter("tilt_step", m_pantilt->getResolution(PTU_TILT));

  declare_parameter("min_pan", m_pantilt->getMin(PTU_PAN));
  declare_parameter("max_pan", m_pantilt->getMax(PTU_PAN));
  declare_parameter("min_pan_speed", m_pantilt->getMinSpeed(PTU_PAN));
  declare_parameter("max_pan_speed", m_pantilt->getMaxSpeed(PTU_PAN));
  declare_parameter("pan_step", m_pantilt->getResolution(PTU_PAN));

  set_parameter({"min_tilt", m_pantilt->getMin(PTU_TILT)});
  set_parameter({"max_tilt", m_pantilt->getMax(PTU_TILT)});
  set_parameter({"min_tilt_speed", m_pantilt->getMinSpeed(PTU_TILT)});
  set_parameter({"max_tilt_speed", m_pantilt->getMaxSpeed(PTU_TILT)});
  set_parameter({"tilt_step", m_pantilt->getResolution(PTU_TILT)});

  set_parameter({"min_pan", m_pantilt->getMin(PTU_PAN)});
  set_parameter({"max_pan", m_pantilt->getMax(PTU_PAN)});
  set_parameter({"min_pan_speed", m_pantilt->getMinSpeed(PTU_PAN)});
  set_parameter({"max_pan_speed", m_pantilt->getMaxSpeed(PTU_PAN)});
  set_parameter({"pan_step", m_pantilt->getResolution(PTU_PAN)});

  m_joint_pub = create_publisher
                <sensor_msgs::msg::JointState>("state", 1);

  m_joint_sub = create_subscription
                <sensor_msgs::msg::JointState>("cmd", 1, 
                std::bind(&HALPTUFlirD46::cmdCallback, this, ph::_1));

  m_reset_sub = create_subscription
                <std_msgs::msg::Bool>("reset", 1, 
                std::bind(&HALPTUFlirD46::resetCallback, this, ph::_1));
  
  int hz;
  hz = declare_parameter("~hz", PTU_DEFAULT_HZ);
  timer_ = this->create_wall_timer(1000ms / hz, std::bind(&HALPTUFlirD46::spinCallback, this));
      
  return true;
  }

  ~HALPTUFlirD46() {
	disconnect();
  }
  
    void disconnect()
{
  if (m_pantilt != NULL)
  {
    delete m_pantilt;   // Closes the connection
    m_pantilt = NULL;   // Marks the service as disconnected
  }
}



 private:
  flir_ptu_driver::PTU* m_pantilt;
  serial::Serial m_ser;
  std::string m_joint_name_prefix;
  double default_velocity_;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_reset_sub;
  rclcpp::TimerBase::SharedPtr timer_;

bool ok()
  {
    return m_pantilt != NULL;
  }

	void resetCallback(const std::shared_ptr<std_msgs::msg::Bool>){
	  RCLCPP_INFO_STREAM(get_logger(), "Resetting the PTU");
	  if (!ok()) return;
	  m_pantilt->home();
	}

	/** Callback for getting new Goal JointState */
	void cmdCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg){
	  RCLCPP_DEBUG_STREAM(get_logger(), "PTU command callback.");
	  if (!ok()) return;

	  if (msg->position.size() != 2){
		RCLCPP_ERROR_STREAM(get_logger(), "JointState command to PTU has wrong number of position elements.");
		return;
	  }

	  double pan = msg->position[0];
	  double tilt = msg->position[1];
	  double panspeed, tiltspeed;

	  if (msg->velocity.size() == 2){
		panspeed = msg->velocity[0];
		tiltspeed = msg->velocity[1];
	  } else {
		RCLCPP_WARN_STREAM(get_logger(), "JointState command to PTU has wrong number of velocity elements; using default velocity.");
		panspeed = default_velocity_;
		tiltspeed = default_velocity_;
	  }

	  m_pantilt->setPosition(PTU_PAN, pan);
	  m_pantilt->setPosition(PTU_TILT, tilt);
	  m_pantilt->setSpeed(PTU_PAN, panspeed);
	  m_pantilt->setSpeed(PTU_TILT, tiltspeed);
	}
	
	void spinCallback(){
		  if (!ok()) return;

  // Read Position & Speed
  double pan  = m_pantilt->getPosition(PTU_PAN);
  double tilt = m_pantilt->getPosition(PTU_TILT);

  double panspeed  = m_pantilt->getSpeed(PTU_PAN);
  double tiltspeed = m_pantilt->getSpeed(PTU_TILT);

  // Publish Position & Speed
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = now();
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.velocity.resize(2);
  joint_state.name[0] = m_joint_name_prefix + "pan";
  joint_state.position[0] = pan;
  joint_state.velocity[0] = panspeed;
  joint_state.name[1] = m_joint_name_prefix + "tilt";
  joint_state.position[1] = tilt;
  joint_state.velocity[1] = tiltspeed;
  m_joint_pub->publish(joint_state);
		}
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<HALPTUFlirD46>();
    if (not node->init()) rclcpp::shutdown();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
