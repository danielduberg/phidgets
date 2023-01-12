#ifndef PHIDGETS_MOTORS_H
#define PHIDGETS_MOTORS_H

// Phidgets
#include <phidgets/DutyCycles.h>
#include <phidgets/MotorConfig.h>
#include <phidgets/motor.h>

// ROS
#include <dynamic_reconfigure/server.h>

// STL
#include <memory>

namespace phidgets
{
class Motors
{
 public:
	Motors(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

	~Motors();

 private:
	void dutyCyclesCallback(DutyCycles::ConstPtr const& msg);

	void configCallback(MotorConfig& config, uint32_t level);

 private:
	ros::Subscriber sub_;

	std::unique_ptr<Motor> left_;
	std::unique_ptr<Motor> right_;

	dynamic_reconfigure::Server<MotorConfig> server_;
	dynamic_reconfigure::Server<MotorConfig>::CallbackType f_;
};
}  // namespace phidgets

#endif  // PHIDGETS_MOTORS_H