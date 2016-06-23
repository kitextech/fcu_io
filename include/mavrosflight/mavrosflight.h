/**
 * \file mavrosflight.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_MAVROSFLIGHT_H
#define MAVROSFLIGHT_MAVROSFLIGHT_H

#include <mavrosflight/mavlink_bridge.h>
#include <mavrosflight/serial_exception.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <stdint.h>
#include <string>
#include <list>

#define MAVROSFLIGHT_READ_BUF_SIZE 256

namespace mavrosflight
{

class MavROSflight
{
public:

  /**
   * \brief Instantiates the class and begins communication on the specified serial port
   * \param port Name of the serial port (e.g. "/dev/ttyUSB0")
   * \param baud_rate Serial communication baud rate
   */
  MavROSflight(std::string port, int baud_rate, uint8_t sysid = 1, uint8_t compid = 50);

  /**
   * \brief Stops communication and closes the serial port before the object is destroyed
   */
  ~MavROSflight();

  /**
   * \brief Stops communication and closes the serial port
   */
  void close();

  // callback functions
  void register_param_value_callback(boost::function<void (std::string, float, MAV_PARAM_TYPE)> f); //! \todo use boost::variant to handle multiple param types
  void unregister_param_value_callback();

  void register_heartbeat_callback(boost::function<void (void)> f);
  void unregister_heartbeat_callback();

  void register_imu_callback(boost::function<void (double, double, double, double, double, double)> f);
  void unregister_imu_callback();

  void register_diff_press_callback(boost::function<void (int16_t, int16_t)> f);
  void unregister_diff_press_callback();

  void register_baro_callback(boost::function<void (int16_t, int16_t)> f);
  void unregister_baro_callback();

  void register_rc_raw_callback(boost::function<void (uint32_t, uint8_t, uint16_t[8])> f);
  void unregister_rc_raw_callback();

  void register_servo_output_raw_callback(boost::function<void (uint32_t, uint8_t, uint16_t[8])> f);
  void unregister_servo_output_raw_callback();

  void register_command_ack_callback(boost::function<void (uint16_t, uint8_t)> f);
  void unregister_command_ack_callback();

  void register_named_value_int_callback(boost::function<void (uint32_t, std::string, int32_t)> f);
  void unregister_named_value_int_callback();

  void register_named_value_float_callback(boost::function<void (uint32_t, std::string, float)> f);
  void unregister_named_value_float_callback();

  // send functions
  void send_param_request_list(uint8_t target_system, uint8_t target_component = MAV_COMP_ID_ALL);
  void send_param_request_read(uint8_t target_system, uint8_t target_component, std::string name);
  void send_param_set(uint8_t target_system, uint8_t target_component, std::string name, int32_t value);
  void send_param_write(uint8_t target_system, uint8_t target_component = MAV_COMP_ID_ALL);
  void send_command(OFFBOARD_CONTROL_MODE mode,
                    OFFBOARD_CONTROL_IGNORE ignore,
                    float value1, float value2, float value3, float value4);

private:

  //===========================================================================
  // definitions
  //===========================================================================

  struct WriteBuffer
  {
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    size_t len;
    size_t pos;

    WriteBuffer() : len(0), pos(0) {}

    WriteBuffer(const uint8_t * buf, uint16_t len) : len(len), pos(0)
    {
      assert(len <= MAVLINK_MAX_PACKET_LEN); //! \todo Do something less catastrophic here
      memcpy(data, buf, len);
    }

    uint8_t * dpos()
    {
      return data + pos;
    }

    size_t nbytes()
    {
      return len - pos;
    }
  };

  /**
   * \brief Convenience typedef for mutex lock
   */
  typedef boost::mutex::scoped_lock mutex_lock;

  //===========================================================================
  // methods
  //===========================================================================

  /**
   * \brief Initiate an asynchronous read operation
   */
  void do_async_read();

  /**
   * \brief Handler for end of asynchronous read operation
   * \param error Error code
   * \param bytes_transferred Number of bytes received
   */
  void async_read_end(const boost::system::error_code& error, size_t bytes_transferred);

  void send_message(const mavlink_message_t &msg);
  void do_async_write(bool check_write_state);
  void async_write_end(const boost::system::error_code& error, size_t bytes_transferred);

  /**
   * \brief Handle a received mavlink message
   */
  void handle_message();

  /**
   * \brief Saturate a value between lower and upper limits
   * \param value The raw value
   * \param min The lower limit
   * \param max The upper limit
   * \returns The saturated value
   */
  template<class T> inline T saturate(T value, T min, T max)
  {
    return value < min ? min : (value > max ? max : value);
  }

  //===========================================================================
  // member variables
  //===========================================================================

  boost::asio::io_service io_service_; //!< boost io service provider
  boost::asio::serial_port serial_port_; //!< boost serial port object
  boost::thread io_thread_; //!< thread on which the io service runs
  boost::mutex mutex_; //!< mutex for threadsafe operation

  uint8_t sysid_;
  uint8_t compid_;

  uint8_t read_buf_raw_[MAVROSFLIGHT_READ_BUF_SIZE];

  mavlink_message_t msg_in_;
  mavlink_status_t status_in_;

  boost::function<void (std::string, float, MAV_PARAM_TYPE)> param_value_callback_;
  boost::function<void (void)> heartbeat_callback_;
  boost::function<void (double, double, double, double, double, double)> imu_callback_;
  boost::function<void (uint32_t, uint8_t, uint16_t[8])> servo_output_raw_callback_;
  boost::function<void (uint32_t, uint8_t, uint16_t[8])> rc_raw_callback_;
  boost::function<void (uint16_t, uint8_t)> command_ack_callback_;
  boost::function<void (int16_t, int16_t)> diff_press_callback_;
  boost::function<void (int16_t, int16_t)> baro_callback_;
  boost::function<void (uint32_t, std::string, int32_t)> named_value_int_callback_;
  boost::function<void (uint32_t, std::string, float)> named_value_float_callback_;

  std::list<WriteBuffer*> write_queue_; //!< queue of buffers to be written to the serial port
  bool write_in_progress_; //!< flag for whether async_write is already running
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_MAVROSFLIGHT_H

