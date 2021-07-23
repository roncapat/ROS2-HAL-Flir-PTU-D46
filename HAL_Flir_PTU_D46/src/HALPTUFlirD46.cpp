#include <iostream>
#include <chrono>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "flir_ptu_d46_interfaces/msg/ptu.hpp"
#include "flir_ptu_d46_interfaces/srv/set_pan.hpp"
#include "flir_ptu_d46_interfaces/srv/set_tilt.hpp"
#include "flir_ptu_d46_interfaces/srv/set_pan_tilt.hpp"
#include "flir_ptu_d46_interfaces/srv/set_pan_tilt_speed.hpp"
#include "flir_ptu_d46_interfaces/srv/get_limits.hpp"

#include "hal_ptu_flir_d46/driver.h"
#include <serial/serial.h>

#include <chrono>
#include <cstdlib>
#include <memory>


using namespace std::chrono_literals;
namespace ph = std::placeholders;

class HALPTUFlirD46 : public rclcpp::Node {
 public:

  HALPTUFlirD46() : Node("hal_ptu_flir_d46") {
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

  this->pan_min = m_pantilt->getMin(PTU_PAN);
  this->pan_max = m_pantilt->getMax(PTU_PAN);

  this->tilt_min = m_pantilt->getMin(PTU_TILT);
  this->tilt_max = m_pantilt->getMax(PTU_TILT);

  ptu_state_pub = create_publisher
                <flir_ptu_d46_interfaces::msg::PTU>("/PTU/state", 1);

  set_pan_srv = create_service<flir_ptu_d46_interfaces::srv::SetPan>("/PTU/set_pan", std::bind(&HALPTUFlirD46::set_pan_callback, this, std::placeholders::_1, std::placeholders::_2));    

  set_tilt_srv = create_service<flir_ptu_d46_interfaces::srv::SetTilt>("/PTU/set_tilt", std::bind(&HALPTUFlirD46::set_tilt_callback, this, std::placeholders::_1, std::placeholders::_2));    

  set_pantilt_srv = create_service<flir_ptu_d46_interfaces::srv::SetPanTilt>("/PTU/set_pan_tilt", std::bind(&HALPTUFlirD46::set_pantilt_callback, this, std::placeholders::_1, std::placeholders::_2));    

  set_pantilt_speed_srv = create_service<flir_ptu_d46_interfaces::srv::SetPanTiltSpeed>("/PTU/set_pan_tilt_speed", std::bind(&HALPTUFlirD46::set_pantilt_speed_callback, this, std::placeholders::_1, std::placeholders::_2));    

  reset_srv = create_service<std_srvs::srv::Empty>("/PTU/reset", std::bind(&HALPTUFlirD46::resetCallback, this, std::placeholders::_1, std::placeholders::_2));

  get_limits_srv = create_service<flir_ptu_d46_interfaces::srv::GetLimits>("/PTU/get_limits", std::bind(&HALPTUFlirD46::get_limits_callback, this, std::placeholders::_1, std::placeholders::_2));

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
  
  rclcpp::Publisher<flir_ptu_d46_interfaces::msg::PTU>::SharedPtr ptu_state_pub;
  rclcpp::Service<flir_ptu_d46_interfaces::srv::SetPan>::SharedPtr set_pan_srv;
  rclcpp::Service<flir_ptu_d46_interfaces::srv::SetTilt>::SharedPtr set_tilt_srv;
  rclcpp::Service<flir_ptu_d46_interfaces::srv::SetPanTilt>::SharedPtr set_pantilt_srv;
  rclcpp::Service<flir_ptu_d46_interfaces::srv::SetPanTiltSpeed>::SharedPtr set_pantilt_speed_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;
  rclcpp::Service<flir_ptu_d46_interfaces::srv::GetLimits>::SharedPtr get_limits_srv;

  rclcpp::TimerBase::SharedPtr timer_;

  bool ok()
  {
    return m_pantilt != NULL;
  }


  void get_limits_callback(const std::shared_ptr<flir_ptu_d46_interfaces::srv::GetLimits::Request> request,
          std::shared_ptr<flir_ptu_d46_interfaces::srv::GetLimits::Response>      response){
    RCLCPP_INFO_STREAM(get_logger(), "Getting PTU limits");
    if (!ok()) return;
    m_pantilt->home();
  }

	void resetCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
          std::shared_ptr<std_srvs::srv::Empty::Response>){
	  RCLCPP_INFO_STREAM(get_logger(), "Resetting the PTU");
	  response->pan_min= this->pan_min;
    response->tilt_min = this->tilt_min;
    response->pan_max = this->pan_max;
    response->tilt_max = this->tilt_max;
	}


  void set_pan_callback(const std::shared_ptr<flir_ptu_d46_interfaces::srv::SetPan::Request> request,
          std::shared_ptr<flir_ptu_d46_interfaces::srv::SetPan::Response>      response){
    if (!ok())
    {
      response->ret = false;
      return;
    } 

    m_pantilt->setPosition(PTU_PAN, request->pan);
    response->ret = true;
  }

  void set_tilt_callback(const std::shared_ptr<flir_ptu_d46_interfaces::srv::SetTilt::Request> request,
          std::shared_ptr<flir_ptu_d46_interfaces::srv::SetTilt::Response>      response){
    if (!ok())
    {
      response->ret = false;
      return;
    } 

    m_pantilt->setPosition(PTU_TILT, request->tilt);
    response->ret = true;
  }


  void set_pantilt_callback(const std::shared_ptr<flir_ptu_d46_interfaces::srv::SetPanTilt::Request> request,
          std::shared_ptr<flir_ptu_d46_interfaces::srv::SetPanTilt::Response>      response){
    if (!ok())
    {
      response->ret = false;
      return;
    } 

    m_pantilt->setPosition(PTU_PAN, request->pan);
    m_pantilt->setPosition(PTU_TILT, request->tilt);
    response->ret = true;
  }


  void set_pantilt_speed_callback(const std::shared_ptr<flir_ptu_d46_interfaces::srv::SetPanTiltSpeed::Request> request,
          std::shared_ptr<flir_ptu_d46_interfaces::srv::SetPanTiltSpeed::Response>      response){
    if (!ok())
    {
      response->ret = false;
      return;
    } 

    m_pantilt->setSpeed(PTU_PAN, request->pan_speed);
    m_pantilt->setSpeed(PTU_TILT, request->tilt_speed);
    response->ret = true;
  }
	
	void spinCallback(){
		  if (!ok()) return;

		// Read Position & Speed
		double pan  = m_pantilt->getPosition(PTU_PAN);
		double tilt = m_pantilt->getPosition(PTU_TILT);

		double panspeed  = m_pantilt->getSpeed(PTU_PAN);
		double tiltspeed = m_pantilt->getSpeed(PTU_TILT);

		// Publish Position & Speed
		flir_ptu_d46_interfaces::msg::PTU ptu_msg;
		ptu_msg.header.stamp = now();
		ptu_msg.pan = pan;
		ptu_msg.tilt = tilt;
		ptu_msg.pan_speed = panspeed;
		ptu_msg.tilt_speed = tiltspeed;
		ptu_state_pub->publish(ptu_msg);
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
