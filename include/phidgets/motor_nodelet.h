#ifndef PHIDGETS_MOTOR_NODELET_H
#define PHIDGETS_MOTOR_NODELET_H

// Phidgets
#include <phidgets/motor.h>

// ROS
#include <nodelet/nodelet.h>

// STL
#include <memory>

namespace phidgets
{
class MotorNodelet : public nodelet::Nodelet
{
 public:
	void onInit() override;

 private:
	std::unique_ptr<Motor> motor_;
};
}  // namespace phidgets

#endif  // PHIDGETS_MOTOR_NODELET_H