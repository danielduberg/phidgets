// Phidgets
#include <phidgets/encoders_nodelet.h>

// ROS
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(phidgets::EncodersNodelet, nodelet::Nodelet)

namespace phidgets
{
void EncodersNodelet::onInit()
{
	NODELET_INFO("Initializing Phidgets Encoders nodelet");
	ros::NodeHandle nh = getNodeHandle();
	ros::NodeHandle nh_priv = getPrivateNodeHandle();
	encoders_ = std::make_unique<Encoders>(nh, nh_priv);
}
}  // namespace phidgets
