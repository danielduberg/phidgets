// Phidgets
#include <phidgets/spatial_nodelet.h>

// ROS
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(phidgets::SpatialNodelet, nodelet::Nodelet)

namespace phidgets
{
void SpatialNodelet::onInit()
{
	NODELET_INFO("Initializing Phidgets Spatial nodelet");
	ros::NodeHandle nh = getNodeHandle();
	ros::NodeHandle nh_priv = getPrivateNodeHandle();
	spatial_ = std::make_unique<Spatial>(nh, nh_priv);
}
}  // namespace phidgets
