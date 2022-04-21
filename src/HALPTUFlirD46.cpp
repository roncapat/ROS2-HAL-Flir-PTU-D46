#include "hal_ptu_flir_d46/HALPTUFlirD46.hpp"

using namespace std::chrono_literals;
namespace ph = std::placeholders;

namespace hal {

FlirD46::FlirD46(const rclcpp::NodeOptions & options) : Node("hal_flir_d46", options) {
  std::cerr << "Ciao" << std::endl;
  disconnect();

  // Query for serial configuration
  std::string port;
  int32_t baud;
  bool limit;
  port = declare_parameter("port", PTU_DEFAULT_PORT);
  limit = declare_parameter("limits_enabled", true);
  baud = declare_parameter("baud", PTU_DEFAULT_BAUD);
  default_velocity_ = declare_parameter("default_velocity", PTU_DEFAULT_VEL);

  // Connect to the PTU
  RCLCPP_INFO_STREAM(get_logger(), "Attempting to connect to FLIR PTU on " << port);

  try{
    m_ser = std::make_shared<serial::Serial>();
    m_ser->setPort(port);
    m_ser->setBaudrate(baud);
    serial::Timeout to = serial::Timeout(200, 200, 0, 200, 0);
    m_ser->setTimeout(to);
    m_ser->open();
  }
  catch (serial::IOException& e){
    RCLCPP_ERROR_STREAM(get_logger(), "Unable to open port " << port);
    std::abort();
  }
  catch(...){

    RCLCPP_ERROR_STREAM(get_logger(), "Unable to open port " << port);
    std::abort();
  }

  RCLCPP_INFO_STREAM(get_logger(), "FLIR PTU serial port opened, now initializing.");

  try{
    
    m_pantilt = std::make_shared<flir_ptu_driver::PTU>(m_ser);
    if (!m_pantilt->initialize()){
      RCLCPP_ERROR_STREAM(get_logger(), "Could not initialize FLIR PTU on " << port);
      disconnect();
      std::abort();
    }

    if (!limit){
      m_pantilt->disableLimits();
      RCLCPP_INFO_STREAM(get_logger(), "FLIR PTU limits disabled.");
    }
  } catch (...){
    RCLCPP_INFO_STREAM(get_logger(), "FLIR PTU Failed to initialize.");
    std::abort();
  }

  RCLCPP_INFO_STREAM(get_logger(), "FLIR PTU initialized.");

  tilt_min = declare_parameter("limits.min_tilt", m_pantilt->getMin(PTU_TILT));
  tilt_max = declare_parameter("limits.max_tilt", m_pantilt->getMax(PTU_TILT));
  tilt_speed_min = declare_parameter("limits.min_tilt_speed", m_pantilt->getMinSpeed(PTU_TILT));
  tilt_speed_max = declare_parameter("limits.max_tilt_speed", m_pantilt->getMaxSpeed(PTU_TILT));
  tilt_resolution = declare_parameter("configuration.tilt_step", m_pantilt->getResolution(PTU_TILT));
  pan_min = declare_parameter("limits.min_pan", m_pantilt->getMin(PTU_PAN));
  pan_max = declare_parameter("limits.max_pan", m_pantilt->getMax(PTU_PAN));
  pan_speed_min = declare_parameter("limits.min_pan_speed", m_pantilt->getMinSpeed(PTU_PAN));
  pan_speed_max = declare_parameter("limits.max_pan_speed", m_pantilt->getMaxSpeed(PTU_PAN));
  pan_resolution = declare_parameter("configuration.pan_step", m_pantilt->getResolution(PTU_PAN));    
  min_threshold_to_move_pan = declare_parameter("min_thresold_command_input_pan", 0.001);
  min_threshold_to_move_tilt = declare_parameter("min_thresold_command_input_tilt", 0.001);
  hz = declare_parameter("hz", PTU_DEFAULT_HZ);

  std::string ptu_state_publisher = declare_parameter<std::string>("publishers.state", "/ptu/state");
  std::string set_pan_srv_name = declare_parameter<std::string>("services.set_pan", "/ptu/set_pan");
  std::string set_tilt_srv_name = declare_parameter<std::string>("services.set_tilt", "/ptu/set_tilt");
  std::string set_pantilt_srv_name = declare_parameter<std::string>("services.set_pantilt", "/ptu/set_pan_tilt");
  std::string set_pantilt_speed_srv_name = declare_parameter<std::string>("services.set_pantilt_speed", "/ptu/set_pan_tilt_speed");
  std::string set_reset_srv_name = declare_parameter<std::string>("services.reset", "/ptu/reset");
  std::string set_get_limits_srv_name = declare_parameter<std::string>("services.get_limits", "/ptu/get_limits");
  std::string set_pan_action_name = declare_parameter<std::string>("actions.set_pan", "/ptu/set_pan");
  std::string set_tilt_action_name = declare_parameter<std::string>("actions.set_tilt", "/ptu/set_tilt");
  std::string set_pantilt_action_name = declare_parameter<std::string>("actions.set_pantilt", "/ptu/set_pan_tilt");

  ptu_state_pub = create_publisher<ptu_interfaces::msg::PTU>(ptu_state_publisher, 1);
  set_pan_srv = create_service<ptu_interfaces::srv::SetPan>(set_pan_srv_name, std::bind(&FlirD46::set_pan_callback, this, ph::_1, ph::_2));    
  set_tilt_srv = create_service<ptu_interfaces::srv::SetTilt>(set_tilt_srv_name, std::bind(&FlirD46::set_tilt_callback, this, ph::_1, ph::_2));    
  set_pantilt_srv = create_service<ptu_interfaces::srv::SetPanTilt>(set_pantilt_srv_name, std::bind(&FlirD46::set_pantilt_callback, this, ph::_1, ph::_2));    
  set_pantilt_speed_srv = create_service<ptu_interfaces::srv::SetPanTiltSpeed>(set_pantilt_speed_srv_name, std::bind(&FlirD46::set_pantilt_speed_callback, this, ph::_1, ph::_2));    
  reset_srv = create_service<std_srvs::srv::Empty>(set_reset_srv_name, std::bind(&FlirD46::resetCallback, this, ph::_1, ph::_2));
  get_limits_srv = create_service<ptu_interfaces::srv::GetLimits>(set_get_limits_srv_name, std::bind(&FlirD46::get_limits_callback, this, ph::_1, ph::_2));

  action_server_set_pan = rclcpp_action::create_server<SetPanAction>(
    this,
    set_pan_action_name,
    std::bind(&FlirD46::handle_goal_pan, this, ph::_1, ph::_2),
    std::bind(&FlirD46::handle_cancel_pan, this, ph::_1),
    std::bind(&FlirD46::handle_accepted_pan, this, ph::_1));


  action_server_set_tilt = rclcpp_action::create_server<SetTiltAction>(
    this,
    set_tilt_action_name,
    std::bind(&FlirD46::handle_goal_tilt, this, ph::_1, ph::_2),
    std::bind(&FlirD46::handle_cancel_tilt, this, ph::_1),
    std::bind(&FlirD46::handle_accepted_tilt, this, ph::_1));


  action_server_set_pantilt = rclcpp_action::create_server<SetPanTiltAction>(
    this,
    set_pantilt_action_name,
    std::bind(&FlirD46::handle_goal_pantilt, this, ph::_1, ph::_2),
    std::bind(&FlirD46::handle_cancel_pantilt, this, ph::_1),
    std::bind(&FlirD46::handle_accepted_pantilt, this, ph::_1));

  timer_ = this->create_wall_timer(1000ms / hz, std::bind(&FlirD46::spinCallback, this));
}

FlirD46::~FlirD46(){
  disconnect();
}

void FlirD46::disconnect(){
  if (m_pantilt != nullptr){
    delete m_pantilt.get();   // Closes the connection
    m_pantilt = nullptr;   // Marks the service as disconnected
  }
}

bool FlirD46::ok(){
  return m_pantilt != nullptr;
}

void FlirD46::get_limits_callback(const std::shared_ptr<ptu_interfaces::srv::GetLimits::Request>,
        std::shared_ptr<ptu_interfaces::srv::GetLimits::Response> response){
  response->pan_min= this->pan_min;
  response->tilt_min = this->tilt_min;
  response->pan_max = this->pan_max;
  response->tilt_max = this->tilt_max;
}


void FlirD46::resetCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>){
  if (!ok()) return;
  m_pantilt->home();
}


void FlirD46::set_pan_callback(const std::shared_ptr<ptu_interfaces::srv::SetPan::Request> request,
        std::shared_ptr<ptu_interfaces::srv::SetPan::Response>      response){
  if (!ok())
  {
    response->ret = false;
    return;
  } 

  // Read Position & Speed
  double pan  = m_pantilt->getPosition(PTU_PAN);

  if(abs(request->pan - pan) > min_threshold_to_move_pan)
  {
      m_pantilt->setPosition(PTU_PAN, request->pan);
  }
  response->ret = true;
}


void FlirD46::set_tilt_callback(const std::shared_ptr<ptu_interfaces::srv::SetTilt::Request> request,
        std::shared_ptr<ptu_interfaces::srv::SetTilt::Response>      response){
  if (!ok())
  {
    response->ret = false;
    return;
  } 

  // Read Position & Speed
  double tilt = m_pantilt->getPosition(PTU_TILT);

  if(abs(request->tilt - tilt) > min_threshold_to_move_tilt)
  {
      m_pantilt->setPosition(PTU_TILT, request->tilt);
  }
  
  response->ret = true;
}


void FlirD46::set_pantilt_callback(const std::shared_ptr<ptu_interfaces::srv::SetPanTilt::Request> request,
        std::shared_ptr<ptu_interfaces::srv::SetPanTilt::Response>      response){
  if (!ok())
  {
    response->ret = false;
    return;
  } 

  // Read Position & Speed
  double pan  = m_pantilt->getPosition(PTU_PAN);
  double tilt = m_pantilt->getPosition(PTU_TILT);

  if(abs(request->pan - pan) > min_threshold_to_move_pan)
  {
      m_pantilt->setPosition(PTU_PAN, request->pan);
  }
  if(abs(request->tilt - tilt) > min_threshold_to_move_tilt)
  {
      m_pantilt->setPosition(PTU_TILT, request->tilt);
  }
  
  response->ret = true;
}


void FlirD46::set_pantilt_speed_callback(const std::shared_ptr<ptu_interfaces::srv::SetPanTiltSpeed::Request> request,
        std::shared_ptr<ptu_interfaces::srv::SetPanTiltSpeed::Response>      response){
  if (!ok())
  {
    response->ret = false;
    return;
  } 

  m_pantilt->setSpeed(PTU_PAN, request->pan_speed);
  m_pantilt->setSpeed(PTU_TILT, request->tilt_speed);
  response->ret = true;
}


void FlirD46::spinCallback(){
  if (!ok()) return;

  // Read Position & Speed
  double pan  = m_pantilt->getPosition(PTU_PAN);
  double tilt = m_pantilt->getPosition(PTU_TILT);

  double panspeed  = m_pantilt->getSpeed(PTU_PAN);
  double tiltspeed = m_pantilt->getSpeed(PTU_TILT);

  // Publish Position & Speed
  ptu_interfaces::msg::PTU ptu_msg;
  ptu_msg.header.stamp = now();
  ptu_msg.pan = pan;
  ptu_msg.tilt = tilt;
  ptu_msg.pan_speed = panspeed;
  ptu_msg.tilt_speed = tiltspeed;
  ptu_state_pub->publish(ptu_msg);
}


rclcpp_action::GoalResponse FlirD46::handle_goal_pan(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const SetPanAction::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse FlirD46::handle_cancel_pan(
  const std::shared_ptr<GoalHandlePanAction> goal_handle)
{
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void FlirD46::handle_accepted_pan(const std::shared_ptr<GoalHandlePanAction> goal_handle)
{
  using namespace ph;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&FlirD46::execute_pan_action, this, _1), goal_handle}.detach();
}


void FlirD46::execute_pan_action(const std::shared_ptr<GoalHandlePanAction> goal_handle)
{
  rclcpp::Rate loop_rate(100.0);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<SetPanAction::Feedback>();
  double perc_of_compl = 0.0;

  auto result = std::make_shared<SetPanAction::Result>();

  if (!ok())
  {
    result->ret = false;
    return;
  } 

  // Read Position & Speed
  double pan  = m_pantilt->getPosition(PTU_PAN);

  double excursion = abs(goal->pan - pan);
  if(excursion > min_threshold_to_move_pan)
  {
      m_pantilt->setPosition(PTU_PAN, goal->pan);

      while(1){

          if (goal_handle->is_canceling()) {
            result->ret = false;
            goal_handle->canceled(result);
            return;
          }

          pan  = m_pantilt->getPosition(PTU_PAN);

          if(abs(goal->pan - pan) < min_threshold_to_move_pan)
          {
              break;
          }

          if (abs(goal->pan - pan) < min_threshold_to_move_pan)
              perc_of_compl = 100.0;
          else
              perc_of_compl = 100.0 - (abs(goal->pan - pan) / excursion * 100.0);
          
          feedback->percentage_of_completing = perc_of_compl;
          goal_handle->publish_feedback(feedback);
          loop_rate.sleep();


      }

  }

  if (rclcpp::ok()) {
    result->ret = true;
    goal_handle->succeed(result);
  }
}


rclcpp_action::GoalResponse FlirD46::handle_goal_tilt(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const SetTiltAction::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse FlirD46::handle_cancel_tilt(
  const std::shared_ptr<GoalHandleTiltAction> goal_handle)
{
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void FlirD46::handle_accepted_tilt(const std::shared_ptr<GoalHandleTiltAction> goal_handle)
{
  using namespace ph;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&FlirD46::execute_tilt_action, this, _1), goal_handle}.detach();
}


void FlirD46::execute_tilt_action(const std::shared_ptr<GoalHandleTiltAction> goal_handle)
{
  rclcpp::Rate loop_rate(100);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<SetTiltAction::Feedback>();
  double perc_of_compl = 0.0;

  auto result = std::make_shared<SetTiltAction::Result>();

  if (!ok())
  {
    result->ret = false;
    return;
  } 

  // Read Position & Speed
  double tilt  = m_pantilt->getPosition(PTU_TILT);

  double excursion = abs(goal->tilt - tilt);
  if(excursion > min_threshold_to_move_tilt)
  {
      m_pantilt->setPosition(PTU_TILT, goal->tilt);

      while(1){

          if (goal_handle->is_canceling()) {
            result->ret = false;
            goal_handle->canceled(result);
            return;
          }

          tilt = m_pantilt->getPosition(PTU_TILT);

          if(abs(goal->tilt - tilt) < min_threshold_to_move_tilt)
          {
              break;
          }

          if (abs(goal->tilt - tilt) < min_threshold_to_move_tilt)
              perc_of_compl = 100.0;
          else
              perc_of_compl = 100.0 - (abs(goal->tilt - tilt) / excursion * 100.0);
            
          feedback->percentage_of_completing = perc_of_compl;
          goal_handle->publish_feedback(feedback);
          loop_rate.sleep();


      }

  }

  // Check if goal is done
  if (rclcpp::ok()) {
    result->ret = true;
    goal_handle->succeed(result);
  }
}


rclcpp_action::GoalResponse FlirD46::handle_goal_pantilt(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const SetPanTiltAction::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse FlirD46::handle_cancel_pantilt(
  const std::shared_ptr<GoalHandlePanTiltAction> goal_handle)
{
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void FlirD46::handle_accepted_pantilt(const std::shared_ptr<GoalHandlePanTiltAction> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&FlirD46::execute_pantilt_action, this, ph::_1), goal_handle}.detach();
}


void FlirD46::execute_pantilt_action(const std::shared_ptr<GoalHandlePanTiltAction> goal_handle)
{
  rclcpp::Rate loop_rate(100);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<SetPanTiltAction::Feedback>();
  double perc_of_compl_pan = 0.0;
  double perc_of_compl_tilt = 0.0;

  auto result = std::make_shared<SetPanTiltAction::Result>();

  if (!ok())
  {
    result->ret = false;
    return;
  } 

  // Read Position & Speed
  
  // Read Position & Speed
  double pan  = m_pantilt->getPosition(PTU_PAN);
  double tilt = m_pantilt->getPosition(PTU_TILT);


  double excursion_pan = abs(goal->pan - pan);
  double excursion_tilt = abs(goal->tilt - tilt);
  if(excursion_pan > min_threshold_to_move_pan or excursion_tilt > min_threshold_to_move_tilt)
  {
      
      if(excursion_pan > min_threshold_to_move_pan)
      {
          m_pantilt->setPosition(PTU_PAN, goal->pan);
      }
      if(excursion_tilt > min_threshold_to_move_tilt)
      {
          m_pantilt->setPosition(PTU_TILT, goal->tilt);
      }

      while(1){

          if (goal_handle->is_canceling()) {
            result->ret = false;
            goal_handle->canceled(result);
            return;
          }

          pan  = m_pantilt->getPosition(PTU_PAN);
          tilt = m_pantilt->getPosition(PTU_TILT);

          if(abs(goal->pan - pan) < min_threshold_to_move_pan and abs(goal->tilt - tilt) < min_threshold_to_move_tilt)
          {
              break;
          }

          if (abs(goal->pan - pan) < min_threshold_to_move_pan)
              perc_of_compl_pan = 100.0;
          else
              perc_of_compl_pan = 100.0 - (abs(goal->pan - pan) / excursion_pan * 100.0);
          
          if (abs(goal->tilt - tilt) < min_threshold_to_move_tilt)
              perc_of_compl_tilt = 100.0;
          else
          
              perc_of_compl_tilt = 100.0 - (abs(goal->tilt - tilt) / excursion_tilt * 100.0);
              
          feedback->percentage_of_completing_pan = perc_of_compl_pan;
          feedback->percentage_of_completing_tilt = perc_of_compl_tilt;

          goal_handle->publish_feedback(feedback);
          loop_rate.sleep();

      }

  }

  // Check if goal is done
  if (rclcpp::ok()) {
    result->ret = true;
    goal_handle->succeed(result);
  }
}

} // namespace hal