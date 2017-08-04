#ifndef telemetry_IO_MAVtelemetry_ROS_H
#define telemetry_IO_MAVtelemetry_ROS_H

#include <string>

#include <ros/ros.h>

#include <rosflight_msgs/RCRaw.h>
#include <rosflight_msgs/GPS.h>

#include <rosplane_msgs/State.h>
#include <rosplane_msgs/Current_Path.h>
#include <rosplane_msgs/Waypoint.h>
#include <rosplane_msgs/Controller_Internals.h>
#include <rosplane_msgs/Controller_Commands.h>

#include <telemetry/mavtelemetry/mavtelemetry.h>
#include <telemetry/mavtelemetry/mavlink_listener_interface.h>

namespace telemetry_io
{

class telemetryIO :
  public mavtelemetry::MavlinkListenerInterface//, public mavtelemetry::ParamListenerInterface
{
public:
  telemetryIO();
  ~telemetryIO();

  virtual void handle_mavlink_message(const mavlink_message_t &msg);

private:

  // handle mavlink messages
  void handle_heartbeat_msg(const mavlink_message_t &msg);
  void handle_statustext_msg(const mavlink_message_t &msg);
  void handle_mav_state_small_msg(const mavlink_message_t &msg);
  void handle_mav_current_path_msg(const mavlink_message_t &msg);
  void handle_mav_waypoint_msg(const mavlink_message_t &msg);
  void handle_rc_raw_msg(const mavlink_message_t &msg);
  void handle_gps_data_msg(const mavlink_message_t &msg);
  void handle_mav_controller_internals_msg(const mavlink_message_t &msg);
  void handle_mav_controller_commands_msg(const mavlink_message_t &msg);

  // ROS publishers
  ros::NodeHandle nh_;
  ros::Publisher mav_state_pub_;
  ros::Publisher mav_current_path_pub_;
  ros::Publisher mav_waypoint_pub_;
  ros::Publisher rc_raw_telem_pub_;
  ros::Publisher gps_data_pub_;
  ros::Publisher mav_controller_internals_pub_;
  ros::Publisher mav_controller_commands_pub_;

  // ROS subscribers
  ros::Subscriber mav_state_sub_;
  ros::Subscriber mav_current_path_sub_;
  ros::Subscriber mav_waypoint_sub_;
  ros::Subscriber rc_raw_telem_sub_;
  ros::Subscriber gps_data_sub_;
  ros::Subscriber mav_controller_internals_sub_;
  ros::Subscriber mav_controller_commands_sub_;

  // ROS message callbacks
  void stateCallback(rosplane_msgs::State::ConstPtr msg);
  void currentPathCallback(rosplane_msgs::Current_Path::ConstPtr msg);
  void waypointCallback(rosplane_msgs::Waypoint::ConstPtr msg);
  void rcRawCallback(rosflight_msgs::RCRaw::ConstPtr msg);
  void gpsDataCallback(rosflight_msgs::GPS::ConstPtr msg);
  void controllerInternalsCallback(rosplane_msgs::Controller_Internals::ConstPtr msg);
  void controllerCommandsCallback(rosplane_msgs::Controller_Commands::ConstPtr msg);

  mavtelemetry::Mavtelemetry *mavtelemetry_;
};

} // namespace telemetry_io

#endif // telemetry_IO_MAVtelemetry_ROS_H
