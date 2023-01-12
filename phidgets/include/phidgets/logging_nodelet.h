#ifndef PHIDGETS_LOGGING_NODELET_H
#define PHIDGETS_LOGGING_NODELET_H

// Phidgets
#include <phidgets/LoggingConfig.h>

// ROS
#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>

// STL
#include <memory>

namespace phidgets
{
class LoggingNodelet : public nodelet::Nodelet
{
 public:
	~LoggingNodelet() override = default;

 private:
	void onInit() override;

	void callback(LoggingConfig& config, uint32_t level);

 private:
	std::unique_ptr<dynamic_reconfigure::Server<LoggingConfig>> server_;
	dynamic_reconfigure::Server<LoggingConfig>::CallbackType f_;
};
}  // namespace phidgets

#endif  // PHIDGETS_LOGGING_NODELET_H