/**
 * \file fcu_io.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef FCU_IO_MAVROSFLIGHT_ROS_H
#define FCU_IO_MAVROSFLIGHT_ROS_H

#include <map>
#include <string>
#include <queue>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Time.h>

#include <fcu_common/Attitude.h>
#include <fcu_common/ExtendedCommand.h>
#include <fcu_common/ServoOutputRaw.h>

#include <fcu_io/ParamGet.h>
#include <fcu_io/ParamSet.h>

#include <mavrosflight/mavrosflight.h>
#include <mavrosflight/mavlink_listener_interface.h>
#include <mavrosflight/param_listener_interface.h>

namespace fcu_io
{

class fcuIO :
  public mavrosflight::MavlinkListenerInterface,
  public mavrosflight::ParamListenerInterface
{
public:
  fcuIO();
  ~fcuIO();

  virtual void handle_mavlink_message(const mavlink_message_t &msg);

  virtual void on_new_param_received(std::string name, double value);
  virtual void on_param_value_updated(std::string name, double value);
  virtual void on_params_saved_change(bool unsaved_changes);

private:

  // handle mavlink messages
  void handle_heartbeat_msg();
  void handle_command_ack_msg(const mavlink_message_t &msg);
  void handle_statustext_msg(const mavlink_message_t &msg);
  void handle_attitude_msg(const mavlink_message_t &msg);
//  void handle_small_imu_msg(const mavlink_message_t &msg);
  void handle_camera_stamped_small_imu_msg(const mavlink_message_t &msg);
  void handle_servo_output_raw_msg(const mavlink_message_t &msg);
  void handle_rc_channels_raw_msg(const mavlink_message_t &msg);
  void handle_diff_pressure_msg(const mavlink_message_t &msg);
  void handle_small_baro_msg(const mavlink_message_t &msg);
  void handle_named_value_int_msg(const mavlink_message_t &msg);
  void handle_named_value_float_msg(const mavlink_message_t &msg);
  void handle_distance_sensor(const mavlink_message_t &msg);

  // ROS message callbacks
  void commandCallback(fcu_common::ExtendedCommand::ConstPtr msg);
  void cameraCallback(const sensor_msgs::Image msg);

  // ROS service callbacks
  bool paramGetSrvCallback(fcu_io::ParamGet::Request &req, fcu_io::ParamGet::Response &res);
  bool paramSetSrvCallback(fcu_io::ParamSet::Request &req, fcu_io::ParamSet::Response &res);
  bool paramWriteSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool calibrateImuBiasSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool calibrateImuTempSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  // Function to match up images and stamps
  void stampMatch();

  // helpers
  template<class T> inline T saturate(T value, T min, T max)
  {
    return value < min ? min : (value > max ? max : value);
  }


  ros::NodeHandle nh_;

  ros::Subscriber command_sub_;
  ros::Subscriber image_sub_;

  ros::Publisher unsaved_params_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher image_pub_;
  ros::Publisher imu_temp_pub_;
  ros::Publisher servo_output_raw_pub_;
  ros::Publisher rc_raw_pub_;
  ros::Publisher diff_pressure_pub_;
  ros::Publisher temperature_pub_;
  ros::Publisher baro_pub_;
  ros::Publisher sonar_pub_;
  ros::Publisher attitude_pub_;
  std::map<std::string, ros::Publisher> named_value_int_pubs_;
  std::map<std::string, ros::Publisher> named_value_float_pubs_;\

  ros::Publisher imu_time_;
  ros::Publisher image_time_;

  std::queue<ros::Time> stamp_queue;
  std::queue<sensor_msgs::Image> image_queue;
  std::queue<ros::Time> stamp_time_queue;
  std::queue<ros::Time> image_time_queue;
  bool missed_stamp;

  ros::ServiceServer param_get_srv_;
  ros::ServiceServer param_set_srv_;
  ros::ServiceServer param_write_srv_;
  ros::ServiceServer imu_calibrate_bias_srv_;
  ros::ServiceServer imu_calibrate_temp_srv_;

  mavrosflight::MavROSflight *mavrosflight_;
  mavrosflight::sensors::DifferentialPressure diff_pressure_;
  mavrosflight::sensors::Imu imu_;
  mavrosflight::sensors::Baro baro_;

  int time_offset;
  double min_image_lag;
  double max_image_lag;
};


} // namespace fcu_io

#endif // FCU_IO_MAVROSFLIGHT_ROS_H
