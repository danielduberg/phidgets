#ifndef PHIDGETS_SPATIAL_NODELET_H
#define PHIDGETS_SPATIAL_NODELET_H

// Phidgets
#include <phidgets/spatial.h>

// ROS
#include <nodelet/nodelet.h>

// STL
#include <memory>

namespace phidgets
{
class SpatialNodelet : public nodelet::Nodelet
{
 public:
	~SpatialNodelet() override = default;

 private:
	void onInit() override;

 private:
	std::unique_ptr<Spatial> spatial_;
};
}  // namespace phidgets

#endif  // PHIDGETS_SPATIAL_NODELET_H