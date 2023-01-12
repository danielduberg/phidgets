// Phidgets
#include <phidgets/motors.h>

namespace phidgets
{
Motors::Motors(ros::NodeHandle& nh, ros::NodeHandle& nh_priv) : server_(nh_priv)
{
	int port_left, port_right;
	if (!nh_priv.getParam("left_port", port_left)) {
		ROS_FATAL("Must specify 'left_port'");
		exit(1);
	}
	if (!nh_priv.getParam("right_port", port_right)) {
		ROS_FATAL("Must specify 'right_port'");
		exit(1);
	}

	uint32_t attach_timeout_ms = nh_priv.param("timeout", PHIDGET_TIMEOUT_DEFAULT);

	left_ = std::make_unique<Motor>(port_left, attach_timeout_ms);
	right_ = std::make_unique<Motor>(port_right, attach_timeout_ms);

	sub_ = nh.subscribe("duty_cycles", 1, &Motors::dutyCyclesCallback, this);

	// Dynamic reconfigure
	f_ = boost::bind(&Motors::configCallback, this, _1, _2);
	server_.setCallback(f_);
}

Motors::~Motors() {}

void Motors::dutyCyclesCallback(DutyCycles::ConstPtr const& msg)
{
	left_->setTargetVelocity(msg->duty_cycle_left);
	right_->setTargetVelocity(msg->duty_cycle_right);
}

void Motors::configCallback(MotorConfig& config, uint32_t level)
{
	left_->setAcceleration(config.acceleration);
	right_->setAcceleration(config.acceleration);

	left_->setTargetBrakingStrength(config.braking_strength);
	right_->setTargetBrakingStrength(config.braking_strength);

	left_->setCurrentLimit(config.current_limit);
	right_->setCurrentLimit(config.current_limit);

	left_->setDataRate(config.data_rate);
	right_->setDataRate(config.data_rate);

	left_->setFailsafe(config.failsafe_timeout);
	right_->setFailsafe(config.failsafe_timeout);
}
}  // namespace phidgets