#ifndef PHIDGETS_MOTORS_NODELET_H
#define PHIDGETS_MOTORS_NODELET_H

// Phidgets
#include <phidgets/motors.h>

// ROS
#include <nodelet/nodelet.h>

// STL
#include <memory>

namespace phidgets
{
class MotorsNodelet : public nodelet::Nodelet
{
 public:
	~MotorsNodelet() override = default;

 private:
	void onInit() override;

 private:
	std::unique_ptr<Motors> motors_;
};
}  // namespace phidgets

#endif  // PHIDGETS_MOTORS_NODELET_H