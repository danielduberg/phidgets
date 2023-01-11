// Phidgets
#include <phidgets/Encoder.h>
#include <phidgets/motor.h>
#include <phidgets/util.h>

// ROS
#include <ros/console.h>

namespace phidgets
{
Motor::Motor(ros::NodeHandle& nh, ros::NodeHandle& nh_priv) : server_(nh_priv)
{
	encoder_pub_ = nh.advertise<Encoder>("encoder", 1);

	int port_left, port_right;
	if (!nh_priv.getParam("motor_left_port", port_left)) {
		ROS_FATAL("Must specify 'motor_left_port'");
		exit(1);
	}
	if (!nh_priv.getParam("motor_right_port", port_right)) {
		ROS_FATAL("Must specify 'motor_right_port'");
		exit(1);
	}

	create();
	setHubPort(port_left, port_right);
	assignEventHandlers();
	attach(nh_priv.param("timeout", PHIDGET_TIMEOUT_DEFAULT));

	pwm_sub_ = nh.subscribe<PWM>("pwm", 1, &Motor::pwmCallback, this);

	// Dynamic reconfigure
	f_ = boost::bind(&Motor::configCallback, this, _1, _2);
	server_.setCallback(f_);
}

Motor::~Motor()
{
	if (EPHIDGET_OK != Phidget_close(reinterpret_cast<PhidgetHandle>(motor_left_)) ||
	    EPHIDGET_OK != Phidget_close(reinterpret_cast<PhidgetHandle>(motor_right_)) ||
	    EPHIDGET_OK != Phidget_close(reinterpret_cast<PhidgetHandle>(encoder_left_)) ||
	    EPHIDGET_OK != Phidget_close(reinterpret_cast<PhidgetHandle>(encoder_right_))) {
		handleError();
		exit(2);
	}

	PhidgetDCMotor_delete(&motor_left_);
	PhidgetDCMotor_delete(&motor_right_);

	PhidgetEncoder_delete(&encoder_left_);
	PhidgetEncoder_delete(&encoder_right_);
}

void Motor::create()
{
	PhidgetDCMotor_create(&motor_left_);
	PhidgetDCMotor_create(&motor_right_);

	PhidgetEncoder_create(&encoder_left_);
	PhidgetEncoder_create(&encoder_right_);
}

void Motor::setHubPort(int port_left, int port_right)
{
	if (EPHIDGET_OK !=
	        Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(motor_left_), port_left) ||
	    EPHIDGET_OK !=
	        Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(motor_right_), port_right) ||
	    EPHIDGET_OK !=
	        Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(encoder_left_), port_left) ||
	    EPHIDGET_OK != Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(encoder_right_),
	                                      port_right)) {
		handleError();
		exit(3);
	}
}

void Motor::assignEventHandlers()
{
	static std::string motor_left_str{"Motor left"};
	static std::string motor_right_str{"Motor right"};
	Phidget_setOnAttachHandler(reinterpret_cast<PhidgetHandle>(motor_left_), attachCallback,
	                           &motor_left_str);
	Phidget_setOnAttachHandler(reinterpret_cast<PhidgetHandle>(motor_right_),
	                           attachCallback, &motor_right_str);
	Phidget_setOnDetachHandler(reinterpret_cast<PhidgetHandle>(motor_left_), detachCallback,
	                           &motor_left_str);
	Phidget_setOnDetachHandler(reinterpret_cast<PhidgetHandle>(motor_right_),
	                           detachCallback, &motor_right_str);
	Phidget_setOnErrorHandler(reinterpret_cast<PhidgetHandle>(motor_left_), errorCallback,
	                          &motor_left_str);
	Phidget_setOnErrorHandler(reinterpret_cast<PhidgetHandle>(motor_right_), errorCallback,
	                          &motor_right_str);

	static std::string encoder_left_str{"Encoder left"};
	static std::string encoder_right_str{"Encoder right"};
	PhidgetEncoder_setOnPositionChangeHandler(encoder_left_, encoderLeftCallback, this);
	PhidgetEncoder_setOnPositionChangeHandler(encoder_right_, encoderRightCallback, this);
	Phidget_setOnAttachHandler(reinterpret_cast<PhidgetHandle>(encoder_left_),
	                           attachCallback, &encoder_left_str);
	Phidget_setOnAttachHandler(reinterpret_cast<PhidgetHandle>(encoder_right_),
	                           attachCallback, &encoder_right_str);
	Phidget_setOnDetachHandler(reinterpret_cast<PhidgetHandle>(encoder_left_),
	                           detachCallback, &encoder_left_str);
	Phidget_setOnDetachHandler(reinterpret_cast<PhidgetHandle>(encoder_right_),
	                           detachCallback, &encoder_right_str);
	Phidget_setOnErrorHandler(reinterpret_cast<PhidgetHandle>(encoder_left_), errorCallback,
	                          &encoder_left_str);
	Phidget_setOnErrorHandler(reinterpret_cast<PhidgetHandle>(encoder_right_),
	                          errorCallback, &encoder_right_str);
}

void Motor::attach(uint32_t timeout_ms)
{
	if (EPHIDGET_OK != Phidget_openWaitForAttachment(
	                       reinterpret_cast<PhidgetHandle>(motor_left_), timeout_ms) ||
	    EPHIDGET_OK != Phidget_openWaitForAttachment(
	                       reinterpret_cast<PhidgetHandle>(motor_right_), timeout_ms) ||
	    EPHIDGET_OK != Phidget_openWaitForAttachment(
	                       reinterpret_cast<PhidgetHandle>(encoder_left_), timeout_ms) ||
	    EPHIDGET_OK != Phidget_openWaitForAttachment(
	                       reinterpret_cast<PhidgetHandle>(encoder_right_), timeout_ms)) {
		handleError();
		exit(4);
	}
}

void Motor::encoderLeftCallback(PhidgetEncoderHandle ch, void* ctx, int position_change,
                                double time_change, int index_triggered)
{
	static_cast<Motor*>(ctx)->last_encoder_left_ = position_change;
	static_cast<Motor*>(ctx)->acc_encoder_left_ += position_change;
	std::cerr << "Hej\n";
}

void Motor::encoderRightCallback(PhidgetEncoderHandle ch, void* ctx, int position_change,
                                 double time_change, int index_triggered)
{
	static_cast<Motor*>(ctx)->last_encoder_right_ = position_change;
	static_cast<Motor*>(ctx)->acc_encoder_right_ += position_change;
}

void Motor::pwmCallback(PWM::ConstPtr const& msg)
{
	if (EPHIDGET_OK != PhidgetDCMotor_setTargetVelocity(motor_left_, msg->pwm_left) ||
	    EPHIDGET_OK != PhidgetDCMotor_setTargetVelocity(motor_right_, msg->pwm_right)) {
		handleError();
		exit(4);
	}
}

void Motor::configCallback(MotorConfig& config, uint32_t level) {}
}  // namespace phidgets