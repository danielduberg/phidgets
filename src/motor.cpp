// Phidgets
#include <phidgets/motor.h>

namespace phidgets {
Motor::Motor(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : server(ros::NodeHandle(nh_priv, "motor")) {  // Dynamic reconfigure
  f = boost::bind(&Motor::configCallback, this, _1, _2);
  server.setCallback(f);
}

Motor::~Motor() {}

void Motor::configCallback(phidgets::MotorConfig& config, uint32_t level) {}
}  // namespace phidgets