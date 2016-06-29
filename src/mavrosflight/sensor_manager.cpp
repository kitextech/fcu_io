/**
 * \file sensor_manager.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <mavrosflight/sensor_manager.h>

namespace mavrosflight
{

bool SensorManager::correct_imu(mavlink_small_imu_t msg,
                                double *xacc, double *yacc, double *zacc, double *xgyro, double *ygyro, double *zgyro)
{
  const double accel_scale = 0.002349;
  const double gyro_scale = 0.004256;

  *xacc = msg.xacc * accel_scale;
  *yacc = msg.yacc * accel_scale;
  *zacc = msg.zacc * accel_scale;

  *xgyro = msg.xgyro * gyro_scale;
  *ygyro = msg.ygyro * gyro_scale;
  *zgyro = msg.zgyro * gyro_scale;
}

bool SensorManager::correct_diff_pressure(mavlink_diff_pressure_t msg, double *pressure, double *temperature)
{
  const double P_min = -1.0f;
  const double P_max = 1.0f;
  const double PSI_to_Pa = 6894.757f;

  static int calibration_counter = 0;
  static int calibration_count = 100;
  static double _diff_pres_offset = 0.0;

  // conversion from pixhawk source code
  double temp = ((200.0f * msg.temperature) / 2047) - 50;

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
  double diff_press_PSI = -((msg.diff_pressure - 0.1f*16383) * (P_max-P_min)/(0.8f*16383) + P_min);
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

} // namespace mavrosflight
