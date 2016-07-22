/**
 * \file fcu_io.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <mavrosflight/serial_exception.h>
#include <string>
#include <stdint.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "fcu_io.h"

namespace fcu_io
{

fcuIO::fcuIO()
{
  command_sub_ = nh_.subscribe("extended_command", 1, &fcuIO::commandCallback, this);

  std::string port = nh_.param<std::string>("port", "/dev/ttyUSB0");
  int baud_rate = nh_.param<int>("baud_rate", 921600);
  std::string image_sub_name = nh_.param<std::string>("image_sub_name", "image_raw");
  std::string image_pub_name = nh_.param<std::string>("image_pub_name", "image_stamped");
  time_offset = nh_.param<int>("time_offset",0);//This is the time shift to be added to the camera stamp in nanoseconds
  min_image_lag = nh_.param<double>("min_image_lag",0);//in seconds
  max_image_lag = nh_.param<double>("max_image_lag",.02);//in seconds

  command_sub_ = nh_.subscribe("extended_command", 1, &fcuIO::commandCallback, this);
  image_sub_ = nh_.subscribe(image_sub_name, 1, &fcuIO::cameraCallback, this);

  unsaved_params_pub_ = nh_.advertise<std_msgs::Bool>("unsaved_params", 1, true);
//  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
  image_pub_ = nh_.advertise<sensor_msgs::Image>(image_pub_name, 1, true);
//  imu_temp_pub_ = nh_.advertise<sensor_msgs::Temperature>("imu/temperature", 1);
//  servo_output_raw_pub_ = nh_.advertise<fcu_common::ServoOutputRaw>("servo_output_raw", 1);
//  rc_raw_pub_ = nh_.advertise<fcu_common::ServoOutputRaw>("rc_raw", 1);
//  diff_pressure_pub_ = nh_.advertise<sensor_msgs::FluidPressure>("diff_pressure", 1);
//  temperature_pub_ = nh_.advertise<sensor_msgs::Temperature>("temperature", 1);

  imu_time_ = nh_.advertise<std_msgs::Time>("imu_time", 1);
  image_time_ = nh_.advertise<std_msgs::Time>("image_time", 1);

  param_get_srv_ = nh_.advertiseService("param_get", &fcuIO::paramGetSrvCallback, this);
  param_set_srv_ = nh_.advertiseService("param_set", &fcuIO::paramSetSrvCallback, this);
  param_write_srv_ = nh_.advertiseService("param_write", &fcuIO::paramWriteSrvCallback, this);

  unsaved_params_pub_ = nh_.advertise<std_msgs::Bool>("unsaved_params", 1, true);
  imu_calibrate_bias_srv_ = nh_.advertiseService("calibrate_imu_bias", &fcuIO::calibrateImuBiasSrvCallback, this);
  imu_calibrate_temp_srv_ = nh_.advertiseService("calibrate_imu_temp", &fcuIO::calibrateImuTempSrvCallback, this);

  try
  {
    mavrosflight_ = new mavrosflight::MavROSflight(port, baud_rate);
  }
  catch (mavrosflight::SerialException e)
  {
    ROS_FATAL("%s", e.what());
    ros::shutdown();
  }

  mavrosflight_->serial.register_mavlink_listener(this);
  mavrosflight_->param.register_param_listener(this);

  std_msgs::Bool unsaved_msg;
  unsaved_msg.data = false;
  unsaved_params_pub_.publish(unsaved_msg);
}

fcuIO::~fcuIO()
{
  delete mavrosflight_;
}

void fcuIO::handle_mavlink_message(const mavlink_message_t &msg)
{
  switch (msg.msgid)
  {
  case MAVLINK_MSG_ID_HEARTBEAT:
    handle_heartbeat_msg();
    break;
  case MAVLINK_MSG_ID_COMMAND_ACK:
    handle_command_ack_msg(msg);
    break;
  case MAVLINK_MSG_ID_STATUSTEXT:
    handle_statustext_msg(msg);
    break;
  case MAVLINK_MSG_ID_ATTITUDE:
    handle_attitude_msg(msg);
    break;
//  case MAVLINK_MSG_ID_SMALL_IMU:
//    handle_small_imu_msg(msg);
//    break;
  case MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU:
    handle_camera_stamped_small_imu_msg(msg);
    break;
  case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
    handle_servo_output_raw_msg(msg);
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
  case MAVLINK_MSG_ID_SMALL_BARO:
    handle_small_baro_msg(msg);
    break;
  case MAVLINK_MSG_ID_DISTANCE_SENSOR:
    handle_distance_sensor(msg);
    break;
  default:
    ROS_DEBUG("fcu_io: Got unhandled mavlink message ID %d", msg.msgid);
    break;
  }
}

void fcuIO::on_new_param_received(std::string name, double value)
{
  ROS_INFO("Got parameter %s with value %g", name.c_str(), value);
}

void fcuIO::on_param_value_updated(std::string name, double value)
{
  ROS_INFO("Parameter %s has new value %g", name.c_str(), value);
}

void fcuIO::on_params_saved_change(bool unsaved_changes)
{
  std_msgs::Bool msg;
  msg.data = unsaved_changes;
  unsaved_params_pub_.publish(msg);

  if (unsaved_changes)
  {
    ROS_WARN("There are unsaved changes to onboard parameters");
  }
  else
  {
    ROS_INFO("Onboard parameters have been saved");
  }
}

void fcuIO::handle_heartbeat_msg()
{
  ROS_INFO_ONCE("Got HEARTBEAT, connected.");
}

void fcuIO::handle_command_ack_msg(const mavlink_message_t &msg)
{
  mavlink_command_ack_t ack;
  mavlink_msg_command_ack_decode(&msg, &ack);

  switch (ack.command)
  {
  case MAV_CMD_PREFLIGHT_CALIBRATION:
    if (ack.result == MAV_RESULT_ACCEPTED)
    {
      ROS_INFO("IMU bias calibration complete!");
    }
    else
    {
      ROS_ERROR("IMU bias calibration failed");
    }
    break;
  }
}

void fcuIO::handle_statustext_msg(const mavlink_message_t &msg)
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
    ROS_ERROR("[FCU]: %s", c_str);
    break;
  case MAV_SEVERITY_WARNING:
    ROS_WARN("[FCU]: %s", c_str);
    break;
  case MAV_SEVERITY_NOTICE:
  case MAV_SEVERITY_INFO:
    ROS_INFO("[FCU]: %s", c_str);
    break;
  case MAV_SEVERITY_DEBUG:
    ROS_DEBUG("[FCU]: %s", c_str);
    break;
  }
}

void fcuIO::handle_attitude_msg(const mavlink_message_t &msg)
{
  mavlink_attitude_t attitude;
  mavlink_msg_attitude_decode(&msg, &attitude);

  fcu_common::Attitude attitude_msg;
  attitude_msg.header.stamp = mavrosflight_->time.get_ros_time_ms(attitude.time_boot_ms);
  attitude_msg.roll = attitude.roll;
  attitude_msg.pitch = attitude.pitch;
  attitude_msg.yaw = attitude.yaw;
  attitude_msg.p = attitude.rollspeed;
  attitude_msg.q = attitude.pitchspeed;
  attitude_msg.r = attitude.yawspeed;

  if(attitude_pub_.getTopic().empty())
  {
    attitude_pub_ = nh_.advertise<fcu_common::Attitude>("attitude", 1);
  }
  attitude_pub_.publish(attitude_msg);
}

//void fcuIO::handle_small_imu_msg(const mavlink_message_t &msg)
//{
//  mavlink_small_imu_t imu;
//  mavlink_msg_small_imu_decode(&msg, &imu);

//  sensor_msgs::Imu imu_msg;
//  imu_msg.header.stamp = mavrosflight_->time.get_ros_time_us(imu.time_boot_us);

//  sensor_msgs::Temperature temp_msg;
//  temp_msg.header.stamp = imu_msg.header.stamp;

//  // This is so we can eventually make calibrating the IMU an external service
//  if (imu_.is_calibrating())
//  {
//    if(imu_.calibrate_temp(imu))
//    {
//      ROS_INFO("IMU temperature calibration complete:\n xm = %f, ym = %f, zm = %f xb = %f yb = %f, zb = %f",
//               imu_.xm(),imu_.ym(),imu_.zm(),imu_.xb(),imu_.yb(),imu_.zb());

//      // calibration is done, send params to the param server
//      mavrosflight_->param.set_param_value("ACC_X_TEMP_COMP", imu_.xm());
//      mavrosflight_->param.set_param_value("ACC_Y_TEMP_COMP", imu_.ym());
//      mavrosflight_->param.set_param_value("ACC_Z_TEMP_COMP", imu_.zm());
//      mavrosflight_->param.set_param_value("ACC_X_BIAS", imu_.xb());
//      mavrosflight_->param.set_param_value("ACC_Y_BIAS", imu_.yb());
//      mavrosflight_->param.set_param_value("ACC_Z_BIAS", imu_.zb());

//      ROS_WARN("Write params to save new temperature calibration!");
//    }
//  }

//  bool valid = imu_.correct(imu,
//                            &imu_msg.linear_acceleration.x,
//                            &imu_msg.linear_acceleration.y,
//                            &imu_msg.linear_acceleration.z,
//                            &imu_msg.angular_velocity.x,
//                            &imu_msg.angular_velocity.y,
//                            &imu_msg.angular_velocity.z,
//                            &temp_msg.temperature);

//  if (valid)
//  {
//    if(imu_pub_.getTopic().empty())
//    {
//      imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
//    }
//    imu_pub_.publish(imu_msg);

//    if(imu_temp_pub_.getTopic().empty())
//    {
//      imu_temp_pub_ = nh_.advertise<sensor_msgs::Temperature>("imu/temperature", 1);
//    }
//    imu_temp_pub_.publish(temp_msg);
//  }
//}

void fcuIO::handle_camera_stamped_small_imu_msg(const mavlink_message_t &msg)
{
  mavlink_camera_stamped_small_imu_t imu;
  mavlink_msg_camera_stamped_small_imu_decode(&msg, &imu);

  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = mavrosflight_->time.get_ros_time_us(imu.time_boot_us);

  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = imu_msg.header.stamp;

  //  Figure out Camera Stamp and data association
  if (imu.image == true)
  {
    //imu_time_.publish(ros::Time::now());
    stamp_time_queue.push(ros::Time::now());
    stamp_queue.push(imu_msg.header.stamp);
  }
  stampMatch();

  // This is so we can eventually make calibrating the IMU an external service
  if (imu_.is_calibrating())
  {
    if(imu_.calibrate_temp(imu))
    {
      ROS_INFO("IMU temperature calibration complete:\n xm = %f, ym = %f, zm = %f xb = %f yb = %f, zb = %f",
               imu_.xm(),imu_.ym(),imu_.zm(),imu_.xb(),imu_.yb(),imu_.zb());

      // calibration is done, send params to the param server
      mavrosflight_->param.set_param_value("ACC_X_TEMP_COMP", imu_.xm());
      mavrosflight_->param.set_param_value("ACC_Y_TEMP_COMP", imu_.ym());
      mavrosflight_->param.set_param_value("ACC_Z_TEMP_COMP", imu_.zm());
      mavrosflight_->param.set_param_value("ACC_X_BIAS", imu_.xb());
      mavrosflight_->param.set_param_value("ACC_Y_BIAS", imu_.yb());
      mavrosflight_->param.set_param_value("ACC_Z_BIAS", imu_.zb());

      ROS_WARN("Write params to save new temperature calibration!");
    }
  }

  bool valid = imu_.correct(imu,
                            &imu_msg.linear_acceleration.x,
                            &imu_msg.linear_acceleration.y,
                            &imu_msg.linear_acceleration.z,
                            &imu_msg.angular_velocity.x,
                            &imu_msg.angular_velocity.y,
                            &imu_msg.angular_velocity.z,
                            &temp_msg.temperature);

  if (valid)
  {
    if(imu_pub_.getTopic().empty())
    {
      imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
    }
    imu_pub_.publish(imu_msg);

    if(imu_temp_pub_.getTopic().empty())
    {
      imu_temp_pub_ = nh_.advertise<sensor_msgs::Temperature>("imu/temperature", 1);
    }
    imu_temp_pub_.publish(temp_msg);
  }
}

void fcuIO::handle_servo_output_raw_msg(const mavlink_message_t &msg)
{
  mavlink_servo_output_raw_t servo;
  mavlink_msg_servo_output_raw_decode(&msg, &servo);

  fcu_common::ServoOutputRaw out_msg;
  out_msg.header.stamp = mavrosflight_->time.get_ros_time_us(servo.time_usec);
  out_msg.port = servo.port;

  out_msg.values[0] = servo.servo1_raw;
  out_msg.values[1] = servo.servo2_raw;
  out_msg.values[2] = servo.servo3_raw;
  out_msg.values[3] = servo.servo4_raw;
  out_msg.values[4] = servo.servo5_raw;
  out_msg.values[5] = servo.servo6_raw;
  out_msg.values[6] = servo.servo7_raw;
  out_msg.values[7] = servo.servo8_raw;

  if(servo_output_raw_pub_.getTopic().empty())
  {
    servo_output_raw_pub_ = nh_.advertise<fcu_common::ServoOutputRaw>("servo_output_raw", 1);
  }
  servo_output_raw_pub_.publish(out_msg);
}

void fcuIO::handle_rc_channels_raw_msg(const mavlink_message_t &msg)
{
  mavlink_rc_channels_raw_t rc;
  mavlink_msg_rc_channels_raw_decode(&msg, &rc);

  fcu_common::ServoOutputRaw out_msg;
  out_msg.header.stamp = mavrosflight_->time.get_ros_time_ms(rc.time_boot_ms);
  out_msg.port = rc.port;

  out_msg.values[0] = rc.chan1_raw;
  out_msg.values[1] = rc.chan2_raw;
  out_msg.values[2] = rc.chan3_raw;
  out_msg.values[3] = rc.chan4_raw;
  out_msg.values[4] = rc.chan5_raw;
  out_msg.values[5] = rc.chan6_raw;
  out_msg.values[6] = rc.chan7_raw;
  out_msg.values[7] = rc.chan8_raw;

  if(rc_raw_pub_.getTopic().empty())
  {
    rc_raw_pub_ = nh_.advertise<fcu_common::ServoOutputRaw>("rc_raw", 1);
  }
  rc_raw_pub_.publish(out_msg);
}

void fcuIO::handle_diff_pressure_msg(const mavlink_message_t &msg)
{
  mavlink_diff_pressure_t diff;
  mavlink_msg_diff_pressure_decode(&msg, &diff);

  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = ros::Time::now(); //! \todo time synchronization

  sensor_msgs::FluidPressure pressure_msg;
  pressure_msg.header.stamp = ros::Time::now(); //! \todo time synchronization

  bool valid = diff_pressure_.correct(diff,
                                      &pressure_msg.fluid_pressure,
                                      &temp_msg.temperature);

  if (valid)
  {
    if(diff_pressure_pub_.getTopic().empty())
    {
      diff_pressure_pub_ = nh_.advertise<sensor_msgs::FluidPressure>("diff_pressure", 1);
    }
    if(temperature_pub_.getTopic().empty())
    {
      temperature_pub_ = nh_.advertise<sensor_msgs::Temperature>("temperature", 1);
    }
    temperature_pub_.publish(temp_msg);
    diff_pressure_pub_.publish(pressure_msg);
  }
}

void fcuIO::handle_named_value_int_msg(const mavlink_message_t &msg)
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

void fcuIO::handle_named_value_float_msg(const mavlink_message_t &msg)
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

void fcuIO::handle_small_baro_msg(const mavlink_message_t &msg)
{
  mavlink_small_baro_t baro;
  mavlink_msg_small_baro_decode(&msg, &baro);

  double alt = 0;

  if (baro_.correct(baro, &alt))
  {
    std_msgs::Float32 alt_msg;
    alt_msg.data = alt;
    if(baro_pub_.getTopic().empty())
    {
      baro_pub_ = nh_.advertise<std_msgs::Float32>("baro/alt", 1);
    }
    baro_pub_.publish(alt_msg);
  }
}

void fcuIO::handle_distance_sensor(const mavlink_message_t &msg)
{
  mavlink_distance_sensor_t distance;
  mavlink_msg_distance_sensor_decode(&msg, &distance);

  sensor_msgs::Range alt_msg;
  alt_msg.header.stamp = mavrosflight_->time.get_ros_time_us(distance.time_boot_ms);

  // MB1242 returns in cm
  alt_msg.max_range = distance.max_distance/100.0;
  alt_msg.min_range = distance.min_distance/100.0;
  alt_msg.range = distance.current_distance/100.0;

  alt_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  alt_msg.field_of_view = 1.0472; // approx 60 deg

  if(sonar_pub_.getTopic().empty())
  {
    sonar_pub_ = nh_.advertise<sensor_msgs::Range>("sonar/data", 1);
  }
  sonar_pub_.publish(alt_msg);

}


void fcuIO::commandCallback(fcu_common::ExtendedCommand::ConstPtr msg)
{
  //! \todo these are hard-coded to match right now; may want to replace with something more robust
  OFFBOARD_CONTROL_MODE mode = (OFFBOARD_CONTROL_MODE) msg->mode;
  OFFBOARD_CONTROL_IGNORE ignore = (OFFBOARD_CONTROL_IGNORE) msg->ignore;

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
  mavlink_msg_offboard_control_pack(1, 50, &mavlink_msg,
                                    mode, ignore, x, y, z, F);
  mavrosflight_->serial.send_message(mavlink_msg);
}

void fcuIO::cameraCallback(const sensor_msgs::Image msg)
{
  //image_time_.publish(ros::Time::now());
  image_time_queue.push(ros::Time::now());
  sensor_msgs::Image cam_msg = msg;
  image_queue.push(cam_msg);
}

void fcuIO::stampMatch()
{
  if (!stamp_queue.empty() && !image_queue.empty()){
    if (stamp_time_queue.empty() || image_time_queue.empty()){
      ROS_ERROR_STREAM("Queue and Time_Queue are not synced.");
      return;
    }
    ros::Time image_time = image_time_queue.front();
    ros::Time stamp_time = stamp_time_queue.front();
    double time_diff = image_time.toSec() - stamp_time.toSec();

//    if (time_diff < min_image_lag){
//      image_time_queue.pop();
//      image_queue.pop();
//      ROS_INFO("toss image, too little lag");
//    }
//    else if (time_diff > max_image_lag){
//      stamp_time_queue.pop();
//      stamp_queue.pop();
//      ROS_INFO("toss image, too much lag");
//    }
//    else{
      //ROS_INFO_STREAM("stamp_queue: " << stamp_queue.size() << " image_queue: " << image_queue.size());

      sensor_msgs::Image cam_msg = image_queue.front();
      image_queue.pop();
      image_time_queue.pop();
      ros::Time stamp = stamp_queue.front();
      stamp_queue.pop();
      stamp_time_queue.pop();
      stamp.nsec += time_offset;
      cam_msg.header.stamp = stamp;
      image_pub_.publish(cam_msg);
      if(image_queue.size() > 0)
      {
        ROS_ERROR("mismatched messages");
      }
  }
}

bool fcuIO::paramGetSrvCallback(fcu_io::ParamGet::Request &req, fcu_io::ParamGet::Response &res)
{
  res.exists = mavrosflight_->param.get_param_value(req.name, &res.value);
  return true;
}

bool fcuIO::paramSetSrvCallback(fcu_io::ParamSet::Request &req, fcu_io::ParamSet::Response &res)
{
  res.exists = mavrosflight_->param.set_param_value(req.name, req.value);
  return true;
}

bool fcuIO::paramWriteSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = mavrosflight_->param.write_params();
  if (!res.success)
  {
    res.message = "Request rejected: write already in progress";
  }

  return true;
}

bool fcuIO::calibrateImuBiasSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  mavlink_message_t msg;
  mavlink_msg_command_int_pack(1, 50, &msg, 1, MAV_COMP_ID_ALL,
                               0, MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 1, 0, 0, 0, 1, 0, 0);
  mavrosflight_->serial.send_message(msg);

  res.success = true;
  return true;
}

bool fcuIO::calibrateImuTempSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  // First, reset the previous calibration
  mavrosflight_->param.set_param_value("ACC_X_TEMP_COMP", 0);
  mavrosflight_->param.set_param_value("ACC_Y_TEMP_COMP", 0);
  mavrosflight_->param.set_param_value("ACC_Z_TEMP_COMP", 0);
  mavrosflight_->param.set_param_value("ACC_X_BIAS", 0);
  mavrosflight_->param.set_param_value("ACC_Y_BIAS", 0);
  mavrosflight_->param.set_param_value("ACC_Z_BIAS", 0);

  // tell the IMU to start a temperature calibration
  imu_.start_temp_calibration();
  ROS_WARN("IMU temperature calibration started");

  res.success = true;
  return true;
}

} // namespace fcu_io
