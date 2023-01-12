#ifndef PHIDGETS_TEMPERATURE_NODELET_H
#define PHIDGETS_TEMPERATURE_NODELET_H

// Phidgets
#include <phidgets/temperature.h>

// ROS
#include <nodelet/nodelet.h>

// STL
#include <memory>

namespace phidgets
{
class TemperatureNodelet : public nodelet::Nodelet
{
 public:
	~TemperatureNodelet() override = default;

 private:
	void onInit() override;

 private:
	std::unique_ptr<Temperature> temperature_;
};
}  // namespace phidgets

#endif  // PHIDGETS_TEMPERATURE_NODELET_H