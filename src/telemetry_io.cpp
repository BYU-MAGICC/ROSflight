#include <telemetry/mavtelemetry/serial_exception.h>
#include <string>
#include <stdint.h>

#include <telemetry/telemetry_io.h>

namespace telemetry_io
{
telemetryIO::telemetryIO()
{
  mav_state_sub_ = nh_.subscribe("state", 1, &telemetryIO::stateCallback, this);
  mav_current_path_sub_ = nh_.subscribe("current_path", 1, &telemetryIO::currentPathCallback, this);
  mav_waypoint_sub_ = nh_.subscribe("waypoints", 1, &telemetryIO::waypointCallback, this);
  rc_raw_telem_sub_ = nh_.subscribe("rc_raw", 1, &telemetryIO::rcRawCallback, this);
  gps_data_sub_ = nh_.subscribe("gps/data", 1, &telemetryIO::gpsDataCallback, this);
  mav_controller_internals_sub_ = nh_.subscribe("controller_inners", 1, &telemetryIO::controllerInternalsCallback, this);
  mav_controller_commands_sub_ = nh_.subscribe("controller_commands", 1, &telemetryIO::controllerCommandsCallback, this);

  ros::NodeHandle nh_private("~");
  std::string port = nh_private.param<std::string>("port", "/dev/ttyUSB0");
  int baud_rate = nh_private.param<int>("baud_rate", 57600);

  ROS_INFO("Connecting to %s, at %d baud", port.c_str(), baud_rate);

  try
  {
    mavtelemetry_ = new mavtelemetry::Mavtelemetry(port, baud_rate);
  }
  catch (mavtelemetry::SerialException e)
  {
    ROS_FATAL("%s", e.what());
    ros::shutdown();
  }

  mavtelemetry_->serial.register_mavlink_listener(this);
}

telemetryIO::~telemetryIO()
{
  delete mavtelemetry_;
}

void telemetryIO::handle_mavlink_message(const mavlink_message_t &msg)
{
  switch (msg.msgid)
  {
    case MAVLINK_MSG_ID_HEARTBEAT:
      handle_heartbeat_msg(msg);
      break;
    case MAVLINK_MSG_ID_STATUSTEXT:
      handle_statustext_msg(msg);
      break;
    case MAVLINK_MSG_ID_MAV_STATE_SMALL:
      handle_mav_state_small_msg(msg);
      break;
    case MAVLINK_MSG_ID_MAV_CURRENT_PATH:
      handle_mav_current_path_msg(msg);
      break;
    case MAVLINK_MSG_ID_MAV_WAYPOINT:
      handle_mav_waypoint_msg(msg);
      break;
    case MAVLINK_MSG_ID_RC_RAW:
      handle_rc_raw_msg(msg);
      break;
    case MAVLINK_MSG_ID_GPS_DATA:
      handle_gps_data_msg(msg);
      break;
    case MAVLINK_MSG_ID_MAV_CONTROLLER_INTERNALS:
      handle_mav_controller_internals_msg(msg);
      break;
    case MAVLINK_MSG_ID_MAV_CONTROLLER_COMMANDS:
      handle_mav_controller_commands_msg(msg);
      break;

    case MAVLINK_MSG_ID_PARAM_VALUE:
    case MAVLINK_MSG_ID_TIMESYNC:
      // silently ignore (handled elsewhere)
      break;
    default:
      ROS_DEBUG("telemetry_io: Got unhandled mavlink message ID %d", msg.msgid);
      break;
  }
}

void telemetryIO::handle_heartbeat_msg(const mavlink_message_t &msg)
{
  ROS_INFO_ONCE("Got HEARTBEAT, connected.");
}

void telemetryIO::handle_statustext_msg(const mavlink_message_t &msg)
{
  mavlink_statustext_t status;
  mavlink_msg_statustext_decode(&msg, &status);

  // ensure null termination
  char c_str[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
  memcpy(c_str, status.text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
  c_str[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = '\0';

  switch (status.severity)
  {
    case MAV_SEVERITY_EMERGENCY:
    case MAV_SEVERITY_ALERT:
    case MAV_SEVERITY_CRITICAL:
    case MAV_SEVERITY_ERROR:
      ROS_ERROR("[Autopilot]: %s", c_str);
      break;
    case MAV_SEVERITY_WARNING:
      ROS_WARN("[Autopilot]: %s", c_str);
      break;
    case MAV_SEVERITY_NOTICE:
    case MAV_SEVERITY_INFO:
      ROS_INFO("[Autopilot]: %s", c_str);
      break;
    case MAV_SEVERITY_DEBUG:
      ROS_DEBUG("[Autopilot]: %s", c_str);
      break;
  }
}

void telemetryIO::handle_mav_state_small_msg(const mavlink_message_t &msg)
{
  mavlink_mav_state_small_t state;
  mavlink_msg_mav_state_small_decode(&msg, &state);

  // fill out the ROS State message
  rosplane_msgs::State state_msg;
  state_msg.header.stamp = ros::Time::now();
  state_msg.position[0] = state.position[0];
  state_msg.position[1] = state.position[1];
  state_msg.position[2] = state.position[2];
  state_msg.Va = state.Va;
  state_msg.alpha = 0.0;
  state_msg.beta = 0.0;
  state_msg.phi = state.phi;
  state_msg.theta = state.theta;
  state_msg.psi = state.psi;
  state_msg.chi = state.chi;
  state_msg.p = 0.0;
  state_msg.q = 0.0;
  state_msg.r = 0.0;
  state_msg.Vg = 0.0;
  state_msg.wn = 0.0;
  state_msg.we = 0.0;
  state_msg.quat[0] = 0.0;  // x
  state_msg.quat[1] = 0.0;  // y
  state_msg.quat[2] = 0.0;  // z
  state_msg.quat[3] = 1.0;  // w
  state_msg.quat_valid = false;
  state_msg.chi_deg = 0.0;
  state_msg.psi_deg = 0.0;
  state_msg.initial_lat = state.initial_lat;
  state_msg.initial_lon = state.initial_lon;
  state_msg.initial_alt = state.initial_alt;

  // publish the message
  if (mav_state_pub_.getTopic().empty())
  {
    mav_state_pub_ = nh_.advertise<rosplane_msgs::State>("state_out", 1);
  }
  mav_state_pub_.publish(state_msg);
}

void telemetryIO::handle_mav_current_path_msg(const mavlink_message_t &msg)
{
  mavlink_mav_current_path_t current_path;
  mavlink_msg_mav_current_path_decode(&msg, &current_path);

  // fill out the ROS Current_Path message
  rosplane_msgs::Current_Path current_path_msg;
  current_path_msg.path_type = current_path.path_type;

  current_path_msg.Va_d = current_path.Va_d;
  current_path_msg.r[0] = current_path.r[0];
  current_path_msg.r[1] = current_path.r[1];
  current_path_msg.r[2] = current_path.r[2];
  current_path_msg.q[0] = current_path.q[0];
  current_path_msg.q[1] = current_path.q[1];
  current_path_msg.q[2] = current_path.q[2];
  current_path_msg.c[0] = current_path.c[0];
  current_path_msg.c[1] = current_path.c[1];
  current_path_msg.c[2] = current_path.c[2];
  current_path_msg.rho = current_path.rho;
  current_path_msg.lambda = current_path.lambda;

  // publish the message
  if (mav_current_path_pub_.getTopic().empty())
  {
    mav_current_path_pub_ = nh_.advertise<rosplane_msgs::Current_Path>("current_path_out", 1);
  }
  mav_current_path_pub_.publish(current_path_msg);
}

void telemetryIO::handle_mav_waypoint_msg(const mavlink_message_t &msg)
{
  mavlink_mav_waypoint_t waypoint;
  mavlink_msg_mav_waypoint_decode(&msg, &waypoint);

  // fill out the ROS Waypoint message
  rosplane_msgs::Waypoint waypoint_msg;
  waypoint_msg.w[0] = waypoint.w[0];
  waypoint_msg.w[1] = waypoint.w[1];
  waypoint_msg.w[2] = waypoint.w[2];
  waypoint_msg.chi_d = waypoint.chi_d;
  if (waypoint.chi_valid == 1)
  {
     waypoint_msg.chi_valid = true;
  }
  else
  {
    waypoint_msg.chi_valid = false;
  }
  waypoint_msg.Va_d = waypoint.Va_d;
  if (waypoint.set_current == 1)
  {
     waypoint_msg.set_current = true;
  }
  else
  {
    waypoint_msg.set_current = false;
  }
  if (waypoint.clear_wp_list == 1)
  {
    waypoint_msg.clear_wp_list = true;
  }
  else
  {
    waypoint_msg.clear_wp_list = false;
  }

  // publish the message
  if (mav_waypoint_pub_.getTopic().empty())
  {
    mav_waypoint_pub_ = nh_.advertise<rosplane_msgs::Waypoint>("waypoint_out", 1);
  }
  mav_waypoint_pub_.publish(waypoint_msg);
}

void telemetryIO::handle_rc_raw_msg(const mavlink_message_t &msg)
{
  mavlink_rc_raw_t rc_raw;
  mavlink_msg_rc_raw_decode(&msg, &rc_raw);

  // fill out the ROS Data
  rosflight_msgs::RCRaw rc_raw_msg;
  rc_raw_msg.values[0] = rc_raw.values[0];
  rc_raw_msg.values[1] = rc_raw.values[1];
  rc_raw_msg.values[2] = rc_raw.values[2];
  rc_raw_msg.values[3] = rc_raw.values[3];
  rc_raw_msg.values[4] = rc_raw.values[4];
  rc_raw_msg.values[5] = rc_raw.values[5];
  rc_raw_msg.values[6] = rc_raw.values[6];
  rc_raw_msg.values[7] = rc_raw.values[7];

  // publish the message
  if (rc_raw_telem_pub_.getTopic().empty())
  {
    rc_raw_telem_pub_ = nh_.advertise<rosflight_msgs::RCRaw>("rc_raw_out", 1);
  }
  rc_raw_telem_pub_.publish(rc_raw_msg);
}

void telemetryIO::handle_gps_data_msg(const mavlink_message_t &msg)
{
  mavlink_gps_data_t gps_data;
  mavlink_msg_gps_data_decode(&msg, &gps_data);

  // fill out the ROS Data
  rosflight_msgs::GPS gps_msg;
  if (gps_data.fix == 1)
  {
    gps_msg.fix = true;
  }
  else
  {
    gps_msg.fix = false;
  }
  gps_msg.NumSat = gps_data.NumSat;
  gps_msg.latitude = gps_data.latitude;
  gps_msg.longitude = gps_data.longitude;
  gps_msg.altitude = gps_data.altitude;
  gps_msg.speed = gps_data.speed;
  gps_msg.ground_course = gps_data.ground_course;
  gps_msg.covariance = gps_data.covariance;

  // publish the message
  if (gps_data_pub_.getTopic().empty())
  {
    gps_data_pub_ = nh_.advertise<rosflight_msgs::GPS>("gps/data_out", 1);
  }
  gps_data_pub_.publish(gps_msg);
}

void telemetryIO::handle_mav_controller_internals_msg(const mavlink_message_t &msg)
{
  mavlink_mav_controller_internals_t controller_internals;
  mavlink_msg_mav_controller_internals_decode(&msg, &controller_internals);

  // fill out the ROS Data
  rosplane_msgs::Controller_Internals controller_internals_msg;
  controller_internals_msg.theta_c = controller_internals.theta_c;
  controller_internals_msg.phi_c = controller_internals.phi_c;
  controller_internals_msg.alt_zone = controller_internals.alt_zone;
  controller_internals_msg.aux[0] = controller_internals.aux[0];
  controller_internals_msg.aux[1] = controller_internals.aux[1];
  controller_internals_msg.aux[2] = controller_internals.aux[2];
  controller_internals_msg.aux[3] = controller_internals.aux[3];
  if (controller_internals.aux_valid == 1)
  {
    controller_internals_msg.aux_valid = true;
  }
  else
  {
    controller_internals_msg.aux_valid = false;
  }

  // publish the message
  if (mav_controller_internals_pub_.getTopic().empty())
  {
    mav_controller_internals_pub_ = nh_.advertise<rosplane_msgs::Controller_Internals>("controller_inners_out", 1);
  }
  mav_controller_internals_pub_.publish(controller_internals_msg);
}

void telemetryIO::handle_mav_controller_commands_msg(const mavlink_message_t &msg)
{
  mavlink_mav_controller_commands_t controller_commands;
  mavlink_msg_mav_controller_commands_decode(&msg, &controller_commands);

  // fill out the ROS Data
  rosplane_msgs::Controller_Commands controller_commands_msg;
  controller_commands_msg.Va_c = controller_commands.Va_c;
  controller_commands_msg.h_c = controller_commands.h_c;
  controller_commands_msg.chi_c = controller_commands.chi_c;
  controller_commands_msg.phi_ff = controller_commands.phi_ff;
  controller_commands_msg.aux[0] = controller_commands.aux[0];
  controller_commands_msg.aux[1] = controller_commands.aux[1];
  controller_commands_msg.aux[2] = controller_commands.aux[2];
  controller_commands_msg.aux[3] = controller_commands.aux[3];
  if (controller_commands.aux_valid == 1)
  {
    controller_commands_msg.aux_valid = true;
  }
  else
  {
    controller_commands_msg.aux_valid = false;
  }

  // publish the message
  if (mav_controller_commands_pub_.getTopic().empty())
  {
    mav_controller_commands_pub_ = nh_.advertise<rosplane_msgs::Controller_Commands>("controller_commands_out", 1);
  }
  mav_controller_commands_pub_.publish(controller_commands_msg);
}

void telemetryIO::stateCallback(rosplane_msgs::State::ConstPtr msg)
{
  float position[3];
  position[0] = msg->position[0];
  position[1] = msg->position[1];
  position[2] = msg->position[2];
  float Va = msg->Va;
  float phi = msg->phi;
  float theta = msg->theta;
  float psi = msg->psi;
  float chi = msg->chi;
  float initial_lat = msg->initial_lat;
  float initial_lon = msg->initial_lon;
  float initial_alt = msg->initial_alt;

  mavlink_message_t mavlink_msg;
  mavlink_msg_mav_state_small_pack(1, 250, &mavlink_msg, position, Va, phi, theta, psi, chi, initial_lat, initial_lon, initial_alt);
  mavtelemetry_->serial.send_message(mavlink_msg);
}

void telemetryIO::currentPathCallback(rosplane_msgs::Current_Path::ConstPtr msg)
{
  uint8_t path_type = msg->path_type;

  float Va_d = msg->Va_d;
  float r[3];
  r[0] = msg->r[0];
  r[1] = msg->r[1];
  r[2] = msg->r[2];
  float q[3];
  q[0] = msg->q[0];
  q[1] = msg->q[1];
  q[2] = msg->q[2];
  float c[3];
  c[0] = msg->c[0];
  c[1] = msg->c[1];
  c[2] = msg->c[2];
  float rho = msg->rho;
  int8_t lambda = msg->lambda;

  mavlink_message_t mavlink_msg;
  mavlink_msg_mav_current_path_pack(1, 250, &mavlink_msg, path_type, Va_d, r, q, c, rho, lambda);
  mavtelemetry_->serial.send_message(mavlink_msg);
}

void telemetryIO::waypointCallback(rosplane_msgs::Waypoint::ConstPtr msg)
{
  float w[3];
  w[0] = msg->w[0];
  w[1] = msg->w[1];
  w[2] = msg->w[2];
  float chi_d = msg->chi_d;
  uint8_t chi_valid;
  if (msg->chi_valid)
  {
    chi_valid = 1;
  }
  else
  {
    chi_valid = 0;
  }
  float Va_d = msg->Va_d;
  uint8_t set_current;
  if (msg->set_current)
  {
    set_current = 1;
  }
  else
  {
    set_current = 0;
  }
  uint8_t clear_wp_list;
  if (msg->clear_wp_list)
  {
    clear_wp_list = 1;
  }
  else
  {
    clear_wp_list = 0;
  }

  mavlink_message_t mavlink_msg;
  mavlink_msg_mav_waypoint_pack(1, 250, &mavlink_msg, w, chi_d, chi_valid, Va_d, set_current, clear_wp_list);
  mavtelemetry_->serial.send_message(mavlink_msg);
}

void telemetryIO::rcRawCallback(rosflight_msgs::RCRaw::ConstPtr msg)
{
  uint16_t values[8];
  for (int i = 0; i < 8; i++)
  {
    values[i] = msg->values[i];
  }

  mavlink_message_t mavlink_msg;
  mavlink_msg_rc_raw_pack(1, 250, &mavlink_msg, values);
  mavtelemetry_->serial.send_message(mavlink_msg);
}

void telemetryIO::gpsDataCallback(rosflight_msgs::GPS::ConstPtr msg)
{
  uint8_t fix;
  if (msg->fix)
  {
    fix = 1;
  }
  else
  {
    fix = 0;
  }
  uint16_t NumSat = msg->NumSat;
  float latitude = msg->latitude;
  float longitude = msg->longitude;
  float altitude = msg->altitude;
  float speed = msg->speed;
  float ground_course = msg->ground_course;
  float covariance = msg->covariance;

  mavlink_message_t mavlink_msg;
  mavlink_msg_gps_data_pack(1, 250, &mavlink_msg, fix, NumSat, latitude, longitude, altitude, speed, ground_course, covariance);
  mavtelemetry_->serial.send_message(mavlink_msg);
}

void telemetryIO::controllerInternalsCallback(rosplane_msgs::Controller_Internals::ConstPtr msg)
{
  float theta_c = msg->theta_c;
  float phi_c = msg->phi_c;
  uint8_t alt_zone = msg->alt_zone;
  float aux[4];
  aux[0] = msg->aux[0];
  aux[1] = msg->aux[1];
  aux[2] = msg->aux[2];
  aux[3] = msg->aux[3];
  uint8_t aux_valid;
  if (msg->aux_valid)
  {
    aux_valid = 1;
  }
  else
  {
    aux_valid = 0;
  }

  mavlink_message_t mavlink_msg;
  mavlink_msg_mav_controller_internals_pack(1, 250, &mavlink_msg, theta_c, phi_c, alt_zone, aux, aux_valid);
  mavtelemetry_->serial.send_message(mavlink_msg);
}

void telemetryIO::controllerCommandsCallback(rosplane_msgs::Controller_Commands::ConstPtr msg)
{
  float Va_c = msg->Va_c;
  float h_c = msg->h_c;
  float chi_c = msg->chi_c;
  float phi_ff = msg->phi_ff;
  float aux[4];
  aux[0] = msg->aux[0];
  aux[1] = msg->aux[1];
  aux[2] = msg->aux[2];
  aux[3] = msg->aux[3];
  uint8_t aux_valid;
  if (msg->aux_valid)
  {
    aux_valid = 1;
  }
  else
  {
    aux_valid = 0;
  }

  mavlink_message_t mavlink_msg;
  mavlink_msg_mav_controller_commands_pack(1, 250, &mavlink_msg, Va_c, h_c, chi_c, phi_ff, aux, aux_valid);
  mavtelemetry_->serial.send_message(mavlink_msg);
}

} // namespace telemetry_io
