#ifndef MAVtelemetry_MAVtelemetry_H
#define MAVtelemetry_MAVtelemetry_H

#include <telemetry/mavtelemetry/mavlink_bridge.h>
#include <telemetry/mavtelemetry/mavlink_serial.h>

#include <telemetry/mavtelemetry/mavlink_listener_interface.h>

#include <boost/function.hpp>

#include <stdint.h>
#include <string>

namespace mavtelemetry
{

class Mavtelemetry
{
public:

  /**
   * \brief Instantiates the class and begins communication on the specified serial port
   * \param port Name of the serial port (e.g. "/dev/ttyUSB0")
   * \param baud_rate Serial communication baud rate
   */
  Mavtelemetry(std::string port, int baud_rate, uint8_t sysid = 1, uint8_t compid = 50);

  /**
   * \brief Stops communication and closes the serial port before the object is destroyed
   */
  ~Mavtelemetry();

  // public member objects
  MavlinkSerial serial;

private:

  // member variables
  uint8_t sysid_;
  uint8_t compid_;
};

} // namespace mavtelemetry

#endif // MAVtelemetry_MAVtelemetry_H
