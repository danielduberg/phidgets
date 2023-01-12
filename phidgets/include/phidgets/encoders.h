#ifndef PHIDGETS_ENCODERS_H
#define PHIDGETS_ENCODERS_H

// Phidgets
#include <phidgets/EncoderConfig.h>
#include <phidgets/encoder.h>

// ROS
#include <dynamic_reconfigure/server.h>

// STL
#include <memory>

namespace phidgets
{
class Encoders
{
 public:
	Encoders(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

	~Encoders();

 private:
	void publish(ros::TimerEvent const& event);

	void configCallback(EncoderConfig& config, uint32_t level);

 private:
	ros::NodeHandle& nh_;

	ros::Publisher pub_;

	ros::Timer pub_timer_;

	std::unique_ptr<Encoder> left_;
	std::unique_ptr<Encoder> right_;

	dynamic_reconfigure::Server<EncoderConfig> server_;
	dynamic_reconfigure::Server<EncoderConfig>::CallbackType f_;
};
}  // namespace phidgets

#endif  // PHIDGETS_ENCODERS_H