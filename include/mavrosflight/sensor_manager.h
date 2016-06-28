/**
 * \file sensor_manager.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_SENSOR_MANAGER_H
#define MAVROSFLIGHT_SENSOR_MANAGER_H

#include <mavrosflight/mavlink_bridge.h>

namespace mavrosflight
{

class SensorManager
{
public:
  bool correct_imu(mavlink_small_imu_t msg,
                   double *xacc, double *yacc, double *zacc, double *xgyro, double *ygyro, double *zgyro);
  bool correct_diff_pressure(mavlink_diff_pressure_t msg, double *pressure, double *temperature);
};

} // namespace mavrosflight

#endif MAVROSFLIGHT_SENSOR_MANAGER_H
