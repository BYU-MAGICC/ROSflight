/*
 * Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file telemetry_io.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <telemetry/mavtelemetry/serial_exception.h>
#include <string>
#include <stdint.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>

#include <telemetry/telemetry_io.h>

namespace telemetry_io
{
telemetryIO::telemetryIO() :
  prev_status_(0),
  prev_error_code_(0),
  prev_control_mode_(0)
{
  command_sub_ = nh_.subscribe("command", 1, &telemetryIO::commandCallback, this);

  unsaved_params_pub_ = nh_.advertise<std_msgs::Bool>("unsaved_params", 1, true);

  param_get_srv_ = nh_.advertiseService("param_get", &telemetryIO::paramGetSrvCallback, this);
  param_set_srv_ = nh_.advertiseService("param_set", &telemetryIO::paramSetSrvCallback, this);
  param_write_srv_ = nh_.advertiseService("param_write", &telemetryIO::paramWriteSrvCallback, this);
  param_save_to_file_srv_ = nh_.advertiseService("param_save_to_file", &telemetryIO::paramSaveToFileCallback, this);
  param_load_from_file_srv_ = nh_.advertiseService("param_load_from_file", &telemetryIO::paramLoadFromFileCallback, this);
  imu_calibrate_bias_srv_ = nh_.advertiseService("calibrate_imu", &telemetryIO::calibrateImuBiasSrvCallback, this);
//  imu_calibrate_temp_srv_ = nh_.advertiseService("calibrate_imu_temp", &telemetryIO::calibrateImuTempSrvCallback, this);
  calibrate_rc_srv_ = nh_.advertiseService("calibrate_rc_trim", &telemetryIO::calibrateRCTrimSrvCallback, this);
  reboot_srv_ = nh_.advertiseService("reboot", &telemetryIO::rebootSrvCallback, this);

  ros::NodeHandle nh_private("~");
  std::string port = nh_private.param<std::string>("port", "/dev/ttyUSB0");
  int baud_rate = nh_private.param<int>("baud_rate", 921600);

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
  mavtelemetry_->param.register_param_listener(this);

  // request the param list
  mavtelemetry_->param.request_params();
  param_timer_ = nh_.createTimer(ros::Duration(1.0), &telemetryIO::paramTimerCallback, this);

  // request version information
  request_version();
  version_timer_ = nh_.createTimer(ros::Duration(1.0), &telemetryIO::versionTimerCallback, this);

  // initialize latched "unsaved parameters" message value
  std_msgs::Bool unsaved_msg;
  unsaved_msg.data = false;
  unsaved_params_pub_.publish(unsaved_msg);

  // Set up a few other random things
  frame_id_ = nh_private.param<std::string>("frame_id", "world");
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
    case MAVLINK_MSG_ID_telemetry_STATUS:
      handle_status_msg(msg);
      break;
    case MAVLINK_MSG_ID_telemetry_CMD_ACK:
      handle_command_ack_msg(msg);
      break;
    case MAVLINK_MSG_ID_STATUSTEXT:
      handle_statustext_msg(msg);
      break;
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
      handle_attitude_quaternion_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_IMU:
      handle_small_imu_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_MAG:
      handle_small_mag_msg(msg);
      break;
    case MAVLINK_MSG_ID_telemetry_OUTPUT_RAW:
      handle_telemetry_output_raw_msg(msg);
      break;
    case MAVLINK_MSG_ID_RC_CHANNELS:
      handle_rc_channels_raw_msg(msg);
      break;
    case MAVLINK_MSG_ID_DIFF_PRESSURE:
      handle_diff_pressure_msg(msg);
      break;
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
      handle_named_value_int_msg(msg);
      break;
    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
      handle_named_value_float_msg(msg);
      break;
    case MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT:
      handle_named_command_struct_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_BARO:
      handle_small_baro_msg(msg);
      break;
    case MAVLINK_MSG_ID_SMALL_SONAR:
      handle_small_sonar(msg);
      break;
    case MAVLINK_MSG_ID_telemetry_VERSION:
      handle_version_msg(msg);
      break;

    // begin jesse insertions

    // i think this is how these should be added?
    case MAVLINK_MSG_ID_MAV_STATE_SMALL:
      handle_mav_state_small_msg(msg);
      break;
    case MAVLINK_MSG_ID_MAV_CURRENT_PATH:
      handle_mav_current_path_msg(msg);
      break;
    case MAVLINK_MSG_ID_MAV_WAYPOINT:
      handle_mav_waypoint_msg(msg);
      break;

    //end jesse insertions

    case MAVLINK_MSG_ID_PARAM_VALUE:
    case MAVLINK_MSG_ID_TIMESYNC:
      // silently ignore (handled elsewhere)
      break;
    default:
      ROS_DEBUG("telemetry_io: Got unhandled mavlink message ID %d", msg.msgid);
      break;
  }
}

void telemetryIO::on_new_param_received(std::string name, double value)
{
  ROS_INFO("Got parameter %s with value %g", name.c_str(), value);
}

void telemetryIO::on_param_value_updated(std::string name, double value)
{
  ROS_INFO("Parameter %s has new value %g", name.c_str(), value);
}

void telemetryIO::on_params_saved_change(bool unsaved_changes)
{
  std_msgs::Bool msg;
  msg.data = unsaved_changes;
  unsaved_params_pub_.publish(msg);

  if (unsaved_changes)
  {
    ROS_WARN_THROTTLE(1,"There are unsaved changes to onboard parameters");
  }
  else
  {
    ROS_INFO("Onboard parameters have been saved");
  }
}

void telemetryIO::handle_heartbeat_msg(const mavlink_message_t &msg)
{
  ROS_INFO_ONCE("Got HEARTBEAT, connected.");
}

void telemetryIO::handle_status_msg(const mavlink_message_t &msg)
{
  mavlink_telemetry_status_t status_msg;
  mavlink_msg_telemetry_status_decode(&msg, &status_msg);

  // Print if change to status
  if (prev_status_ != status_msg.status)
  {
    // armed state check
    if ((prev_status_ & telemetry_STATUS_ARMED) != (status_msg.status & telemetry_STATUS_ARMED))
    {
      if (status_msg.status & telemetry_STATUS_ARMED)
        ROS_WARN("Autopilot ARMED");
      else
        ROS_WARN("Autopilot DISARMED");
    }

    // failsafe check
    if ((prev_status_ & telemetry_STATUS_IN_FAILSAFE) != (status_msg.status & telemetry_STATUS_IN_FAILSAFE))
    {
      if (status_msg.status & telemetry_STATUS_IN_FAILSAFE)
        ROS_ERROR("Autopilot FAILSAFE");
      else
        ROS_INFO("Autopilot FAILSAFE RECOVERED");
    }

    // rc override check
    if ((prev_status_ & telemetry_STATUS_RC_OVERRIDE) != (status_msg.status & telemetry_STATUS_RC_OVERRIDE))
    {
      if (status_msg.status & telemetry_STATUS_RC_OVERRIDE)
        ROS_WARN("RC override active");
      else
        ROS_WARN("Returned to computer control");
    }

    // offboard control check
    if ((prev_status_ & telemetry_STATUS_OFFBOARD_CONTROL_ACTIVE) != (status_msg.status & telemetry_STATUS_OFFBOARD_CONTROL_ACTIVE))
    {
      if (status_msg.status & telemetry_STATUS_OFFBOARD_CONTROL_ACTIVE)
        ROS_WARN("Computer control active");
      else
        ROS_WARN("Computer control lost");
    }
    prev_status_ = status_msg.status;
  }

  // Print if got error code
  if (prev_error_code_ != status_msg.error_code)
  {
    ROS_ERROR("Autopilot ERROR 0x%02x", status_msg.error_code);
    prev_error_code_ = status_msg.error_code;
  }

  // Print if change in control mode
  if (prev_control_mode_ != status_msg.control_mode)
  {
    std::string mode_string;
    switch (status_msg.control_mode)
    {
      case MODE_PASS_THROUGH:
        mode_string = "PASS_THROUGH";
        break;
      case MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
        mode_string = "RATE";
        break;
      case MODE_ROLL_PITCH_YAWRATE_THROTTLE:
        mode_string = "ANGLE";
        break;
      default:
        mode_string = "UNKNOWN";
    }
    ROS_WARN_STREAM("Autopilot now in " << mode_string << " mode");
    prev_control_mode_ = status_msg.control_mode;
  }

  // Build the status message and send it
  rosflight_msgs::Status out_status;
  out_status.header.stamp = ros::Time::now();
  out_status.armed = status_msg.status & telemetry_STATUS_ARMED;
  out_status.failsafe = status_msg.status & telemetry_STATUS_IN_FAILSAFE;
  out_status.rc_override = status_msg.status & telemetry_STATUS_RC_OVERRIDE;
  out_status.num_errors = status_msg.num_errors;
  out_status.loop_time_us = status_msg.loop_time_us;
  if (status_pub_.getTopic().empty())
  {
    status_pub_ = nh_.advertise<rosflight_msgs::Status>("status", 1);
  }
  status_pub_.publish(out_status);
}

void telemetryIO::handle_command_ack_msg(const mavlink_message_t &msg)
{
  mavlink_telemetry_cmd_ack_t ack;
  mavlink_msg_telemetry_cmd_ack_decode(&msg, &ack);

  if (ack.success == telemetry_CMD_SUCCESS)
  {
    ROS_DEBUG("MAVLink command %d Acknowledged", ack.command);
  }
  else
  {
    ROS_ERROR("MAVLink command %d Failed", ack.command);
  }
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

void telemetryIO::handle_attitude_quaternion_msg(const mavlink_message_t &msg)
{
  mavlink_attitude_quaternion_t attitude;
  mavlink_msg_attitude_quaternion_decode(&msg, &attitude);

  rosflight_msgs::Attitude attitude_msg;
  attitude_msg.header.stamp = mavtelemetry_->time.get_ros_time_ms(attitude.time_boot_ms);
  attitude_msg.attitude.w = attitude.q1;
  attitude_msg.attitude.x = attitude.q2;
  attitude_msg.attitude.y = attitude.q3;
  attitude_msg.attitude.z = attitude.q4;
  attitude_msg.angular_velocity.x = attitude.rollspeed;
  attitude_msg.angular_velocity.y = attitude.pitchspeed;
  attitude_msg.angular_velocity.z = attitude.yawspeed;

  geometry_msgs::Vector3Stamped euler_msg;

  tf::Quaternion quat(attitude.q2, attitude.q3, attitude.q4, attitude.q1);
  tf::Matrix3x3(quat).getEulerYPR(euler_msg.vector.z, euler_msg.vector.y, euler_msg.vector.x);

  // save off the quaternion for use with the IMU callback
  tf::quaternionTFToMsg(quat, attitude_quat_);

  if (attitude_pub_.getTopic().empty())
  {
    attitude_pub_ = nh_.advertise<rosflight_msgs::Attitude>("attitude", 1);
  }
  if (euler_pub_.getTopic().empty())
  {
    euler_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("attitude/euler", 1);
  }
  attitude_pub_.publish(attitude_msg);
  euler_pub_.publish(euler_msg);
}

void telemetryIO::handle_small_imu_msg(const mavlink_message_t &msg)
{
  mavlink_small_imu_t imu;
  mavlink_msg_small_imu_decode(&msg, &imu);

  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = mavtelemetry_->time.get_ros_time_us(imu.time_boot_us);
  imu_msg.header.frame_id = frame_id_;

  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = imu_msg.header.stamp;
  temp_msg.header.frame_id = frame_id_;

  // This is so we can eventually make calibrating the IMU an external service
  if (imu_.is_calibrating())
  {
    if (imu_.calibrate_temp(imu))
    {
      ROS_INFO("IMU temperature calibration complete:\n xm = %f, ym = %f, zm = %f xb = %f yb = %f, zb = %f", imu_.xm(),
               imu_.ym(), imu_.zm(), imu_.xb(), imu_.yb(), imu_.zb());

      // calibration is done, send params to the param server
      mavtelemetry_->param.set_param_value("ACC_X_TEMP_COMP", imu_.xm());
      mavtelemetry_->param.set_param_value("ACC_Y_TEMP_COMP", imu_.ym());
      mavtelemetry_->param.set_param_value("ACC_Z_TEMP_COMP", imu_.zm());
      mavtelemetry_->param.set_param_value("ACC_X_BIAS", imu_.xb());
      mavtelemetry_->param.set_param_value("ACC_Y_BIAS", imu_.yb());
      mavtelemetry_->param.set_param_value("ACC_Z_BIAS", imu_.zb());

      ROS_WARN("Write params to save new temperature calibration!");
    }
  }

  bool valid = imu_.correct(imu, &imu_msg.linear_acceleration.x, &imu_msg.linear_acceleration.y,
                            &imu_msg.linear_acceleration.z, &imu_msg.angular_velocity.x, &imu_msg.angular_velocity.y,
                            &imu_msg.angular_velocity.z, &temp_msg.temperature);

  imu_msg.orientation = attitude_quat_;

  if (valid)
  {
    if (imu_pub_.getTopic().empty())
    {
      imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
    }
    imu_pub_.publish(imu_msg);

    if (imu_temp_pub_.getTopic().empty())
    {
      imu_temp_pub_ = nh_.advertise<sensor_msgs::Temperature>("imu/temperature", 1);
    }
    imu_temp_pub_.publish(temp_msg);
  }
}

void telemetryIO::handle_telemetry_output_raw_msg(const mavlink_message_t &msg)
{
  mavlink_telemetry_output_raw_t servo;
  mavlink_msg_telemetry_output_raw_decode(&msg, &servo);

  rosflight_msgs::OutputRaw out_msg;
  out_msg.header.stamp = mavtelemetry_->time.get_ros_time_us(servo.stamp);
  for (int i = 0; i < 8; i++)
  {
    out_msg.values[i] = servo.values[i];
  }

  if (output_raw_pub_.getTopic().empty())
  {
    output_raw_pub_ = nh_.advertise<rosflight_msgs::OutputRaw>("output_raw", 1);
  }
  output_raw_pub_.publish(out_msg);
}

void telemetryIO::handle_rc_channels_raw_msg(const mavlink_message_t &msg)
{
  mavlink_rc_channels_raw_t rc;
  mavlink_msg_rc_channels_raw_decode(&msg, &rc);

  rosflight_msgs::RCRaw out_msg;
  out_msg.header.stamp = mavtelemetry_->time.get_ros_time_ms(rc.time_boot_ms);

  out_msg.values[0] = rc.chan1_raw;
  out_msg.values[1] = rc.chan2_raw;
  out_msg.values[2] = rc.chan3_raw;
  out_msg.values[3] = rc.chan4_raw;
  out_msg.values[4] = rc.chan5_raw;
  out_msg.values[5] = rc.chan6_raw;
  out_msg.values[6] = rc.chan7_raw;
  out_msg.values[7] = rc.chan8_raw;

  if (rc_raw_pub_.getTopic().empty())
  {
    rc_raw_pub_ = nh_.advertise<rosflight_msgs::RCRaw>("rc_raw", 1);
  }
  rc_raw_pub_.publish(out_msg);
}

void telemetryIO::handle_diff_pressure_msg(const mavlink_message_t &msg)
{
  mavlink_diff_pressure_t diff;
  mavlink_msg_diff_pressure_decode(&msg, &diff);

  rosflight_msgs::Airspeed airspeed_msg;
  airspeed_msg.header.stamp = ros::Time::now();
  airspeed_msg.velocity = diff.velocity;
  airspeed_msg.differential_pressure = diff.diff_pressure;
  airspeed_msg.temperature = diff.temperature;

  if(calibrate_airspeed_srv_.getService().empty())
  {
    calibrate_airspeed_srv_ = nh_.advertiseService("calibrate_airspeed", &telemetryIO::calibrateAirspeedSrvCallback, this);
  }

  if (diff_pressure_pub_.getTopic().empty())
  {
    diff_pressure_pub_ = nh_.advertise<rosflight_msgs::Airspeed>("airspeed", 1);
  }
  diff_pressure_pub_.publish(airspeed_msg);
}

void telemetryIO::handle_named_value_int_msg(const mavlink_message_t &msg)
{
  mavlink_named_value_int_t val;
  mavlink_msg_named_value_int_decode(&msg, &val);

  // ensure null termination of name
  char c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN + 1];
  memcpy(c_name, val.name, MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN);
  c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN] = '\0';
  std::string name(c_name);

  if (named_value_int_pubs_.find(name) == named_value_int_pubs_.end())
  {
    ros::NodeHandle nh;
    named_value_int_pubs_[name] = nh.advertise<std_msgs::Int32>("named_value/int/" + name, 1);
  }

  std_msgs::Int32 out_msg;
  out_msg.data = val.value;

  named_value_int_pubs_[name].publish(out_msg);
}

void telemetryIO::handle_named_value_float_msg(const mavlink_message_t &msg)
{
  mavlink_named_value_float_t val;
  mavlink_msg_named_value_float_decode(&msg, &val);

  // ensure null termination of name
  char c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN + 1];
  memcpy(c_name, val.name, MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN);
  c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN] = '\0';
  std::string name(c_name);

  if (named_value_float_pubs_.find(name) == named_value_float_pubs_.end())
  {
    ros::NodeHandle nh;
    named_value_float_pubs_[name] = nh.advertise<std_msgs::Float32>("named_value/float/" + name, 1);
  }

  std_msgs::Float32 out_msg;
  out_msg.data = val.value;

  named_value_float_pubs_[name].publish(out_msg);
}

void telemetryIO::handle_named_command_struct_msg(const mavlink_message_t &msg)
{
  mavlink_named_command_struct_t command;
  mavlink_msg_named_command_struct_decode(&msg, &command);

  // ensure null termination of name
  char c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN + 1];
  memcpy(c_name, command.name, MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN);
  c_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN] = '\0';
  std::string name(c_name);

  if (named_command_struct_pubs_.find(name) == named_command_struct_pubs_.end())
  {
    ros::NodeHandle nh;
    named_command_struct_pubs_[name] = nh.advertise<rosflight_msgs::Command>("named_value/command_struct/" + name, 1);
  }

  rosflight_msgs::Command command_msg;
  if (command.type == MODE_PASS_THROUGH)
    command_msg.mode = rosflight_msgs::Command::MODE_PASS_THROUGH;
  else if (command.type == MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE)
    command_msg.mode = rosflight_msgs::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
  else if (command.type == MODE_ROLL_PITCH_YAWRATE_THROTTLE)
    command_msg.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  else if (command.type == MODE_ROLL_PITCH_YAWRATE_ALTITUDE)
    command_msg.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_ALTITUDE;

  command_msg.ignore = command.ignore;
  command_msg.x = command.x;
  command_msg.y = command.y;
  command_msg.z = command.z;
  command_msg.F = command.F;
  named_command_struct_pubs_[name].publish(command_msg);
}

void telemetryIO::handle_small_baro_msg(const mavlink_message_t &msg)
{
  mavlink_small_baro_t baro;
  mavlink_msg_small_baro_decode(&msg, &baro);

  rosflight_msgs::Barometer baro_msg;
  baro_msg.header.stamp = ros::Time::now();
  baro_msg.altitude = baro.altitude;
  baro_msg.pressure = baro.pressure;
  baro_msg.temperature = baro.temperature;

  // If we are getting barometer messages, then we should publish the barometer calibration service
  if(calibrate_baro_srv_.getService().empty())
  {
    calibrate_baro_srv_ = nh_.advertiseService("calibrate_baro", &telemetryIO::calibrateBaroSrvCallback, this);
  }

  if (baro_pub_.getTopic().empty())
  {
    baro_pub_ = nh_.advertise<rosflight_msgs::Barometer>("baro", 1);
  }
  baro_pub_.publish(baro_msg);
}

void telemetryIO::handle_small_mag_msg(const mavlink_message_t &msg)
{
  mavlink_small_mag_t mag;
  mavlink_msg_small_mag_decode(&msg, &mag);

  //! \todo calibration, correct units, floating point message type
  sensor_msgs::MagneticField mag_msg;
  mag_msg.header.stamp = ros::Time::now();//mavtelemetry_->time.get_ros_time_us(mag.time_boot_us);
  mag_msg.header.frame_id = frame_id_;

  mag_msg.magnetic_field.x = mag.xmag;
  mag_msg.magnetic_field.y = mag.ymag;
  mag_msg.magnetic_field.z = mag.zmag;

  if (mag_pub_.getTopic().empty())
  {
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("magnetometer", 1);
  }
  mag_pub_.publish(mag_msg);
}

void telemetryIO::handle_small_sonar(const mavlink_message_t &msg)
{
  mavlink_small_sonar_t sonar;
  mavlink_msg_small_sonar_decode(&msg, &sonar);

  sensor_msgs::Range alt_msg;
  alt_msg.header.stamp = ros::Time::now();
  alt_msg.max_range = sonar.max_range;
  alt_msg.min_range = sonar.min_range;
  alt_msg.range = sonar.range;

  alt_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  alt_msg.field_of_view = 1.0472;  // approx 60 deg

  if (sonar_pub_.getTopic().empty())
  {
    sonar_pub_ = nh_.advertise<sensor_msgs::Range>("sonar", 1);
  }
  sonar_pub_.publish(alt_msg);
}

void telemetryIO::handle_version_msg(const mavlink_message_t &msg)
{
  version_timer_.stop();

  mavlink_telemetry_version_t version;
  mavlink_msg_telemetry_version_decode(&msg, &version);

  std_msgs::String version_msg;
  version_msg.data = version.version;

  if (version_pub_.getTopic().empty())
  {
    version_pub_ = nh_.advertise<std_msgs::String>("version", 1, true);
  }
  version_pub_.publish(version_msg);

  ROS_INFO("Firmware version: %s", version.version);
}

// begin jesse insertions

void telemetryIO::handle_mav_state_small_msg(const mavlink_message_t &msg)
{
  mavlink_msg_mav_state_small_t state;
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
    mav_state_pub_ = nh_.advertise<rosplane_msgs::State>("state", 1);
  }
  mav_state_pub_.publish(state_msg);
}

void telemetryIO::handle_mav_current_path_msg(const mavlink_message_t &msg)
{
  mavlink_msg_mav_current_path_t current_path;
  mavlink_msg_mav_current_path_decode(&msg, &current_path);

  // fill out the ROS Current_Path message
  rosplane_msgs::Current_Path current_path_msg;
  if (current_path.flag == 1)
  {
    current_path_msg.flag = true;
  }
  else
  {
    current_path_msg.flag = false;
  }
  current_path_msg.Va_d = current_path.Va_d
  current_path_msg.r[0] = current_path.r[0]
  current_path_msg.r[1] = current_path.r[1]
  current_path_msg.r[2] = current_path.r[2]
  current_path_msg.q[0] = current_path.q[0]
  current_path_msg.q[1] = current_path.q[1]
  current_path_msg.q[2] = current_path.q[2]
  current_path_msg.c[0] = current_path.c[0]
  current_path_msg.c[1] = current_path.c[1]
  current_path_msg.c[2] = current_path.c[2]
  current_path_msg.rho = current_path.rho
  current_path_msg.lambda = current_path.lambda

  // publish the message
  if (mav_current_path_pub_.getTopic().empty())
  {
    mav_current_path_pub_ = nh_.advertise<rosplane_msgs::Current_Path>("current_path", 1);
  }
  mav_current_path_pub_.publish(current_path_msg);
}

void telemetryIO::handle_mav_waypoint_msg(const mavlink_message_t &msg)
{
  mavlink_msg_mav_waypoint_t waypoint;
  mavlink_msg_mav_waypoint_decode(&msg, &waypoint);

  // fill out the ROS Waypoint message
  rosplane_msgs::Waypoint waypoint_msg;
  waypoint_msg.w[0] = waypoint[0];
  waypoint_msg.w[1] = waypoint[1];
  waypoint_msg.w[2] = waypoint[2];
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

  // publish the message
  if (mav_waypoint_pub_.getTopic().empty())
  {
    mav_waypoint_pub_ = nh_.advertise<rosplane_msgs::Waypoint>("waypoint", 1);
  }
  mav_current_path_pub_.publish(waypoint_msg);
}

// end jesse insertions

void telemetryIO::commandCallback(rosflight_msgs::Command::ConstPtr msg)
{
  //! \todo these are hard-coded to match right now; may want to replace with something more robust
  OFFBOARD_CONTROL_MODE mode = (OFFBOARD_CONTROL_MODE)msg->mode;
  OFFBOARD_CONTROL_IGNORE ignore = (OFFBOARD_CONTROL_IGNORE)msg->ignore;

  float x = msg->x;
  float y = msg->y;
  float z = msg->z;
  float F = msg->F;

  switch (mode)
  {
    case MODE_PASS_THROUGH:
      x = saturate(x, -1.0f, 1.0f);
      y = saturate(y, -1.0f, 1.0f);
      z = saturate(z, -1.0f, 1.0f);
      F = saturate(F, 0.0f, 1.0f);
      break;
    case MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
    case MODE_ROLL_PITCH_YAWRATE_THROTTLE:
      F = saturate(F, 0.0f, 1.0f);
      break;
    case MODE_ROLL_PITCH_YAWRATE_ALTITUDE:
      break;
  }

  mavlink_message_t mavlink_msg;
  mavlink_msg_offboard_control_pack(1, 50, &mavlink_msg, mode, ignore, x, y, z, F);
  mavtelemetry_->serial.send_message(mavlink_msg);
}

bool telemetryIO::paramGetSrvCallback(rosflight_msgs::ParamGet::Request &req, rosflight_msgs::ParamGet::Response &res)
{
  res.exists = mavtelemetry_->param.get_param_value(req.name, &res.value);
  return true;
}

bool telemetryIO::paramSetSrvCallback(rosflight_msgs::ParamSet::Request &req, rosflight_msgs::ParamSet::Response &res)
{
  res.exists = mavtelemetry_->param.set_param_value(req.name, req.value);
  return true;
}

bool telemetryIO::paramWriteSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = mavtelemetry_->param.write_params();
  if (!res.success)
  {
    res.message = "Request rejected: write already in progress";
  }

  return true;
}

bool telemetryIO::paramSaveToFileCallback(rosflight_msgs::ParamFile::Request &req, rosflight_msgs::ParamFile::Response &res)
{
  res.success = mavtelemetry_->param.save_to_file(req.filename);
  return true;
}

bool telemetryIO::paramLoadFromFileCallback(rosflight_msgs::ParamFile::Request &req, rosflight_msgs::ParamFile::Response &res)
{
  res.success = mavtelemetry_->param.load_from_file(req.filename);
  return true;
}

bool telemetryIO::calibrateImuBiasSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_telemetry_cmd_pack(1, 50, &msg, telemetry_CMD_ACCEL_CALIBRATION);
  mavtelemetry_->serial.send_message(msg);

  res.success = true;
  return true;
}

bool telemetryIO::calibrateRCTrimSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_telemetry_cmd_pack(1, 50, &msg, telemetry_CMD_RC_CALIBRATION);
  mavtelemetry_->serial.send_message(msg);
  res.success = true;
  return true;
}

void telemetryIO::paramTimerCallback(const ros::TimerEvent &e)
{
  if (mavtelemetry_->param.got_all_params())
  {
    param_timer_.stop();
    ROS_INFO("Received all parameters");
  }
  else
  {
    mavtelemetry_->param.request_params();
    ROS_INFO("Received %d of %d parameters. Requesting missing parameters...",
             mavtelemetry_->param.get_params_received(), mavtelemetry_->param.get_num_params());
  }
}

void telemetryIO::versionTimerCallback(const ros::TimerEvent &e)
{
  request_version();
}

void telemetryIO::request_version()
{
  mavlink_message_t msg;
  mavlink_msg_telemetry_cmd_pack(1, 50, &msg, telemetry_CMD_SEND_VERSION);
  mavtelemetry_->serial.send_message(msg);
}

bool telemetryIO::calibrateImuTempSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  // First, reset the previous calibration
  mavtelemetry_->param.set_param_value("ACC_X_TEMP_COMP", 0);
  mavtelemetry_->param.set_param_value("ACC_Y_TEMP_COMP", 0);
  mavtelemetry_->param.set_param_value("ACC_Z_TEMP_COMP", 0);
  mavtelemetry_->param.set_param_value("ACC_X_BIAS", 0);
  mavtelemetry_->param.set_param_value("ACC_Y_BIAS", 0);
  mavtelemetry_->param.set_param_value("ACC_Z_BIAS", 0);

  // tell the IMU to start a temperature calibration
  imu_.start_temp_calibration();
  ROS_WARN("IMU temperature calibration started");

  res.success = true;
  return true;
}

bool telemetryIO::calibrateAirspeedSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_telemetry_cmd_pack(1, 50, &msg, telemetry_CMD_AIRSPEED_CALIBRATION);
  mavtelemetry_->serial.send_message(msg);
  res.success = true;
  return true;
}

bool telemetryIO::calibrateBaroSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_telemetry_cmd_pack(1, 50, &msg, telemetry_CMD_BARO_CALIBRATION);
  mavtelemetry_->serial.send_message(msg);
  res.success = true;
  return true;
}

bool telemetryIO::rebootSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_telemetry_cmd_pack(1, 50, &msg, telemetry_CMD_REBOOT);
  mavtelemetry_->serial.send_message(msg);
  res.success = true;
  return true;
}

} // namespace telemetry_io
