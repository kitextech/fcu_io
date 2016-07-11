/**
 * \file imu.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_SENSORS_IMU_H
#define MAVROSFLIGHT_SENSORS_IMU_H

#include <mavrosflight/mavlink_bridge.h>

namespace mavrosflight
{
namespace sensors
{

/**
 * \brief IMU sensor class
 */
class Imu
{
public:

  /**
   * \brief Get corrected measurement values
   * \param msg The raw IMU message
   * \param[out] xacc The accelerometer X value (m/s^2)
   * \param[out] yacc The accelerometer Y value (m/s^2)
   * \param[out] zacc The accelerometer Z value (m/s^2)
   * \param[out] xgyro The rate gyro X value (rad/s)
   * \param[out] ygyro The rate gyro Y value (rad/s)
   * \param[out] zgyro The rate gyro Z value (rad/s)
   * \return True if the measurement is valid
   */
  bool correct(mavlink_camera_stamped_small_imu_t msg,
               double *xacc, double *yacc, double *zacc, double *xgyro, double *ygyro, double *zgyro, double *temperature);
  bool correct(mavlink_small_imu_t msg,
               double *xacc, double *yacc, double *zacc, double *xgyro, double *ygyro, double *zgyro, double *temperature);

private:
  //! \todo explicitly compute these so it's clear where they come from
  static const double ACCEL_SCALE = 0.002349;
  static const double GYRO_SCALE = 0.004256;
};

} // namespace sensors
} // namespace mavrosflight

#endif // MAVROSFLIGHT_SENSORS_IMU_H
