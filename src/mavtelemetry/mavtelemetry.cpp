#include <telemetry/mavtelemetry/mavtelemetry.h>

#include <ros/ros.h>

namespace mavtelemetry
{

using boost::asio::serial_port_base;

Mavtelemetry::Mavtelemetry(std::string port, int baud_rate, uint8_t sysid /* = 1 */, uint8_t compid /* = 50 */) :
  serial(port, baud_rate),
  sysid_(sysid),
  compid_(compid)
{}

Mavtelemetry::~Mavtelemetry()
{}

} // namespace mavtelemetry
