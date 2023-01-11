#ifndef PHIDGETS_MOTOR_H
#define PHIDGETS_MOTOR_H

// Phidget
extern "C" {
#include <phidget22.h>
}

// ROS
#include <dynamic_reconfigure/server.h>
#include <phidgets/MotorConfig.h>
#include <ros/ros.h>

// STL

namespace phidgets {
class Motor {
 public:
  Motor(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  ~Motor();

	private:
	void configCallback(phidgets::MotorConfig& config, uint32_t level);

 private:
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;
  ros::Publisher temp_pub_;

  dynamic_reconfigure::Server<phidgets::MotorConfig> server;
  dynamic_reconfigure::Server<phidgets::MotorConfig>::CallbackType f;
};
}  // namespace phidgets

#endif  // PHIDGETS_MOTOR_H