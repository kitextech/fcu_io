/**
 * \file fcu_io.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <mavrosflight/serial_exception.h>
#include <string>
#include <stdint.h>

#include "fcu_io.h"

namespace fcu_io
{

fcuIO::fcuIO()
{
  ros::NodeHandle nh;

  command_sub_ = nh.subscribe("extended_command", 1, &fcuIO::commandCallback, this);

  imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
  servo_output_raw_pub_ = nh.advertise<fcu_common::ServoOutputRaw>("servo_output_raw", 1);
  rc_raw_pub_ = nh.advertise<fcu_common::ServoOutputRaw>("rc_raw", 1);
  diff_pressure_pub_ = nh.advertise<sensor_msgs::FluidPressure>("diff_pressure", 1);
  temperature_pub_ = nh.advertise<sensor_msgs::Temperature>("temperature", 1);
  baro_pub_ = nh.advertise<std_msgs::Float32>("baro/alt", 1);

  param_request_list_srv_ = nh.advertiseService("param_request_list", &fcuIO::paramRequestListSrvCallback, this);
  param_request_read_srv_ = nh.advertiseService("param_request_read", &fcuIO::paramRequestReadSrvCallback, this);
  param_set_srv_ = nh.advertiseService("param_set", &fcuIO::paramSetSrvCallback, this);
  param_write_srv_ = nh.advertiseService("param_write", &fcuIO::paramWriteSrvCallback, this);

  ros::NodeHandle nh_private("~");
  std::string port = nh_private.param<std::string>("port", "/dev/ttyUSB0");
  int baud_rate = nh_private.param<int>("baud_rate", 115200);

  try
  {
    mavrosflight_ = new mavrosflight::MavROSflight(port, baud_rate);
  }
  catch (mavrosflight::SerialException e)
  {
    ROS_FATAL("%s", e.what());
    ros::shutdown();
  }

  mavrosflight_->register_param_value_callback(boost::bind(&fcuIO::paramCallback, this, _1, _2, _3));
  mavrosflight_->register_heartbeat_callback(boost::bind(&fcuIO::heartbeatCallback, this));
  mavrosflight_->register_imu_callback(boost::bind(&fcuIO::imuCallback, this, _1, _2, _3, _4, _5, _6));
  mavrosflight_->register_servo_output_raw_callback(boost::bind(&fcuIO::servoOutputRawCallback, this, _1, _2, _3));
  mavrosflight_->register_rc_raw_callback(boost::bind(&fcuIO::rcRawCallback, this, _1, _2, _3));
  mavrosflight_->register_diff_press_callback(boost::bind(&fcuIO::diffPressCallback, this, _1, _2));
  mavrosflight_->register_baro_callback(boost::bind(&fcuIO::baroCallback, this, _1, _2));
  mavrosflight_->register_command_ack_callback(boost::bind(&fcuIO::commandAckCallback, this, _1, _2));
  mavrosflight_->register_named_value_int_callback(boost::bind(&fcuIO::namedValueIntCallback, this, _1, _2, _3));
  mavrosflight_->register_named_value_float_callback(boost::bind(&fcuIO::namedValueFloatCallback, this, _1, _2, _3));
  mavrosflight_->send_param_request_list(1);
}

fcuIO::~fcuIO()
{
  delete mavrosflight_;
}

bool fcuIO::paramRequestListSrvCallback(fcu_io::ParamRequestList::Request &req, fcu_io::ParamRequestList::Response &res)
{
  mavrosflight_->send_param_request_list(1);
  return true;
}

bool fcuIO::paramRequestReadSrvCallback(ParamRequestRead::Request &req, ParamRequestRead::Response &res)
{
  mavrosflight_->send_param_request_read(1, MAV_COMP_ID_ALL, req.param_id);
  return true;
}

bool fcuIO::paramSetSrvCallback(fcu_io::ParamSet::Request &req, fcu_io::ParamSet::Response &res)
{
  switch (req.param_type)
  {
  case MAV_PARAM_TYPE_INT32:
    mavrosflight_->send_param_set(1, MAV_COMP_ID_ALL, req.param_id, req.integer_value);
    return true;
  default:
    ROS_ERROR("Currently only params of type int32 are supported");
    return false;
  }

}

bool fcuIO::paramWriteSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  mavrosflight_->send_param_write(1);
  return true;
}

void fcuIO::paramCallback(std::string param_id, float param_value, MAV_PARAM_TYPE param_type)
{
  bool have_uint(false);
  uint32_t uint_value;

  bool have_int(false);
  int32_t int_value;

  bool have_float(false);

  switch (param_type)
  {
  case MAV_PARAM_TYPE_UINT8:
    uint_value = (uint32_t) (*(uint8_t*) &param_value);
    have_uint = true;
    break;
  case MAV_PARAM_TYPE_UINT16:
    uint_value = (uint32_t) (*(uint16_t*) &param_value);
    have_uint = true;
    break;
  case MAV_PARAM_TYPE_UINT32:
    uint_value = *(uint32_t*) &param_value;
    have_uint = true;
    break;
  case MAV_PARAM_TYPE_INT8:
    int_value = (int32_t) (*(uint8_t*) &param_value);
    have_int = true;
    break;
  case MAV_PARAM_TYPE_INT16:
    int_value = (int32_t) (*(uint16_t*) &param_value);
    have_int = true;
    break;
  case MAV_PARAM_TYPE_INT32:
    int_value = *(uint32_t*) &param_value;
    have_int = true;
    break;
  case MAV_PARAM_TYPE_REAL32:
    have_float = true;
    break;
  default:
    break;
  }

  if (have_uint)
    ROS_INFO("Got parameter %s with value %u", param_id.c_str(), uint_value);
  else if (have_int)
    ROS_INFO("Got parameter %s with value %d", param_id.c_str(), int_value);
  else if (have_float)
    ROS_INFO("Got parameter %s with value %f", param_id.c_str(), param_value);
}

void fcuIO::heartbeatCallback()
{
  ROS_INFO_ONCE("Got HEARTBEAT, connected.");
}

void fcuIO::imuCallback(double xacc, double yacc, double zacc, double xgyro, double ygyro, double zgyro)
{
  sensor_msgs::Imu msg;

  msg.header.stamp = ros::Time::now();

  msg.linear_acceleration.x = xacc*.002349;
  msg.linear_acceleration.y = yacc*.002349;
  msg.linear_acceleration.z = zacc*.002349;

  msg.angular_velocity.x = xgyro*.004256;
  msg.angular_velocity.y = ygyro*.004256;
  msg.angular_velocity.z = zgyro*.004256;

  imu_pub_.publish(msg);
}

void fcuIO::servoOutputRawCallback(uint32_t time_usec, uint8_t port, uint16_t values[8])
{
  fcu_common::ServoOutputRaw msg;

  msg.port = port;
  for (int i = 0; i < 8; i++)
  {
    msg.values[i] = values[i];
  }

  servo_output_raw_pub_.publish(msg);
}

void fcuIO::rcRawCallback(uint32_t time_usec, uint8_t port, uint16_t values[8])
{
  fcu_common::ServoOutputRaw msg;

  msg.port = port;
  for (int i = 0; i < 8; i++)
  {
    msg.values[i] = values[i];
  }

  rc_raw_pub_.publish(msg);
}

void fcuIO::diffPressCallback(int16_t diff_pressure, int16_t temperature)
{
  const double P_min = -1.0f;
  const double P_max = 1.0f;
  const double PSI_to_Pa = 6894.757f;

  static int calibration_counter = 0;
  static int calibration_count = 100;
  static double _diff_pres_offset = 0.0;

  // conversion from pixhawk source code
  double temp = ((200.0f * temperature) / 2047) - 50;
  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = ros::Time::now();
  temp_msg.temperature = temp;
  temperature_pub_.publish(temp_msg);

  /*
   * this equation is an inversion of the equation in the
   * pressure transfer function figure on page 4 of the datasheet
   * We negate the result so that positive differential pressures
   * are generated when the bottom port is used as the static
   * port on the pitot and top port is used as the dynamic port
   */
  double diff_press_PSI = -((diff_pressure - 0.1f*16383) * (P_max-P_min)/(0.8f*16383) + P_min);
  double diff_press_pa_raw = diff_press_PSI * PSI_to_Pa;
  if (calibration_counter > calibration_count)
  {
    diff_press_pa_raw -= _diff_pres_offset;
    sensor_msgs::FluidPressure pressure_msg;
    pressure_msg.header.stamp = ros::Time::now();
    pressure_msg.fluid_pressure = diff_press_pa_raw;
    diff_pressure_pub_.publish(pressure_msg);
  }
  else if (calibration_counter == calibration_count)
  {
    _diff_pres_offset = _diff_pres_offset/calibration_count;
    calibration_counter++;
  }
  else
  {
    _diff_pres_offset += diff_press_pa_raw;
    calibration_counter++;
  }
}

void fcuIO::baroCallback(int16_t pressure, int16_t temperature)
{
  // calibration variables
  static int calibration_counter = 0;
  static double calibration_sum = 0;
  static int settling_count = 20; // settle for a second or so
  static int calibration_count = 20;

  // offsets and filters
  static double prev_alt = 0.0;
  static double alt_alpha = 0.01; // really slow
  static double alt_ground = 0;

  if( calibration_counter > calibration_count + settling_count)
  {
    double alt_tmp = (1.0f - pow(pressure/101325.0f, 0.190295f)) * 4430.0f; // in meters

    // offset calculated ground altitude
    alt_tmp -= alt_ground;

    // LPF measurements
    double altitude = alt_alpha*alt_tmp + (1.0 - alt_alpha)*prev_alt;
    prev_alt = altitude;

    // publish measurement
    std_msgs::Float32 alt_msg;
    alt_msg.data = altitude;
    baro_pub_.publish(alt_msg);
  }
  if (calibration_counter < settling_count)
  {
    calibration_counter++;
  }
  else if (calibration_counter < settling_count + calibration_count)
  {
    double measurement = (1.0f - pow(pressure/101325.0f, 0.190295f)) * 4430.0f;
    calibration_sum += measurement;
    calibration_counter++;
  }
  else if(calibration_counter == settling_count + calibration_count)
  {
    alt_ground = calibration_sum/calibration_count;
    ROS_INFO_STREAM("BARO CALIBRATED " << alt_ground << " meters above sea level");
    calibration_counter++;
  }
}

void fcuIO::commandAckCallback(uint16_t command, uint8_t result)
{
  ROS_INFO("Acknowledged command %d with result %d", command, result);
}

void fcuIO::namedValueIntCallback(uint32_t time, std::string name, int32_t value)
{
  if (named_value_int_pubs_.find(name) == named_value_int_pubs_.end())
  {
    ros::NodeHandle nh;
    named_value_int_pubs_[name] = nh.advertise<std_msgs::Int32>("named_value/int/" + name, 1);
  }

  std_msgs::Int32 msg;
  msg.data = value;

  named_value_int_pubs_[name].publish(msg);
}

void fcuIO::namedValueFloatCallback(uint32_t time, std::string name, float value)
{
  if (named_value_float_pubs_.find(name) == named_value_float_pubs_.end())
  {
    ros::NodeHandle nh;
    named_value_float_pubs_[name] = nh.advertise<std_msgs::Float32>("named_value/float/" + name, 1);
  }

  std_msgs::Float32 msg;
  msg.data = value;

  named_value_float_pubs_[name].publish(msg);
}

void fcuIO::commandCallback(fcu_common::ExtendedCommand::ConstPtr msg)
{
  //! \todo these are hard-coded to match right now; may want to replace with something more robust
  OFFBOARD_CONTROL_MODE mode = (OFFBOARD_CONTROL_MODE) msg->mode;
  OFFBOARD_CONTROL_IGNORE ignore = (OFFBOARD_CONTROL_IGNORE) msg->ignore;

  mavrosflight_->send_command(mode, ignore, msg->value1, msg->value2, msg->value3, msg->value4);
}

} // namespace fcu_io
