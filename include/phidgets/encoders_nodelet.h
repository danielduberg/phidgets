#ifndef PHIDGETS_ENCODER_NODELET_H
#define PHIDGETS_ENCODER_NODELET_H

// Phidgets
#include <phidgets/encoders.h>

// ROS
#include <nodelet/nodelet.h>

// STL
#include <memory>

namespace phidgets
{
class EncodersNodelet : public nodelet::Nodelet
{
 public:
	~EncodersNodelet() override = default;

 private:
	void onInit() override;

 private:
	std::unique_ptr<Encoders> encoders_;
};
}  // namespace phidgets

#endif  // PHIDGETS_ENCODER_NODELET_H