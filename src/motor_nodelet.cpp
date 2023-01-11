// Phidgets
#include <phidgets/motor_nodelet.h>

// ROS
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(phidgets::MotorNodelet, nodelet::Nodelet)

namespace phidgets
{
void MotorNodelet::onInit()
{
	NODELET_INFO("Initializing Phidgets Motor nodelet");
	ros::NodeHandle nh = getNodeHandle();
	ros::NodeHandle nh_priv = getPrivateNodeHandle();
	motor_ = std::make_unique<Motor>(nh, nh_priv);
}
}  // namespace phidgets
