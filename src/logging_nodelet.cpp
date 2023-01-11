// Phidgets
#include <phidgets/logging_nodelet.h>

// ROS
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

// Phidget
extern "C" {
#include <phidget22.h>
}

PLUGINLIB_EXPORT_CLASS(phidgets::LoggingNodelet, nodelet::Nodelet)

namespace phidgets
{
void LoggingNodelet::onInit()
{
	NODELET_INFO("Initializing Phidgets Logging nodelet");
	server_ = std::make_unique<dynamic_reconfigure::Server<LoggingConfig>>(
	    getPrivateNodeHandle());
	f_ = boost::bind(&LoggingNodelet::callback, this, _1, _2);
	server_->setCallback(f_);
}

void LoggingNodelet::callback(LoggingConfig& config, uint32_t level)
{
	// TODO: Handle errors

	Phidget_LogLevel cur;
	PhidgetLog_getLevel(&cur);
	if (static_cast<Phidget_LogLevel>(config.logging) != cur || 1 == level) {
		PhidgetLog_disable();
		PhidgetLog_enable(static_cast<Phidget_LogLevel>(config.logging),
		                  config.console
		                      ? NULL
		                      : (ros::package::getPath("phidgets") + "/phidget_log").c_str());
	}

	PhidgetLog_setRotating(config.rotating_size, config.rotating_max_files);

	int rotating;
	PhidgetLog_isRotating(&rotating);
	if ((rotating ? true : false) != config.rotate) {
		if (config.rotate) {
			PhidgetLog_enableRotating();
		} else {
			PhidgetLog_disableRotating();
		}
	}
}
}  // namespace phidgets
