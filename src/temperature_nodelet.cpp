// Phidgets
#include <phidgets/temperature_nodelet.h>

// ROS
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(phidgets::TemperatureNodelet, nodelet::Nodelet)

namespace phidgets
{
void TemperatureNodelet::onInit()
{
	NODELET_INFO("Initializing Phidgets Temperature nodelet");
	ros::NodeHandle nh = getNodeHandle();
	ros::NodeHandle nh_priv = getPrivateNodeHandle();
	temperature_ = std::make_unique<Temperature>(nh, nh_priv);
}
}  // namespace phidgets
