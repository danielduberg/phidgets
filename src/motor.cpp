// Phidgets
#include <phidgets/Encoder.h>
#include <phidgets/motor.h>
#include <phidgets/util.h>

// ROS
#include <ros/console.h>

namespace phidgets
{
Motor::Motor(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : server_(nh_priv), encoder_worker_(&Motor::encoderPublisher, this)
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
	done_.store(true);
	encoder_worker_.join();

	handleError(Phidget_close(reinterpret_cast<PhidgetHandle>(motor_left_)), 2, "Motor");
	handleError(Phidget_close(reinterpret_cast<PhidgetHandle>(motor_right_)), 2, "Motor");
	handleError(Phidget_close(reinterpret_cast<PhidgetHandle>(encoder_left_)), 2, "Motor");
	handleError(Phidget_close(reinterpret_cast<PhidgetHandle>(encoder_right_)), 2, "Motor");

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
	handleError(Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(motor_left_), port_left),
	            3, "Motor");
	handleError(
	    Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(motor_right_), port_right), 3,
	    "Motor");
	handleError(
	    Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(encoder_left_), port_left), 3,
	    "Motor");
	handleError(
	    Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(encoder_right_), port_right), 3,
	    "Motor");
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
	handleError(Phidget_openWaitForAttachment(reinterpret_cast<PhidgetHandle>(motor_left_),
	                                          timeout_ms),
	            4, "Motor");
	handleError(Phidget_openWaitForAttachment(reinterpret_cast<PhidgetHandle>(motor_right_),
	                                          timeout_ms),
	            4, "Motor");
	handleError(Phidget_openWaitForAttachment(
	                reinterpret_cast<PhidgetHandle>(encoder_left_), timeout_ms),
	            4, "Motor");
	handleError(Phidget_openWaitForAttachment(
	                reinterpret_cast<PhidgetHandle>(encoder_right_), timeout_ms),
	            4, "Motor");
}

void Motor::encoderLeftCallback(PhidgetEncoderHandle ch, void* ctx, int position_change,
                                double time_change, int index_triggered)
{
	std::unique_lock<std::mutex> lk(static_cast<Motor*>(ctx)->encoder_m_);
	static_cast<Motor*>(ctx)->encoder_left_queue_.push(position_change);
	lk.unlock();
	static_cast<Motor*>(ctx)->encoder_cv_.notify_one();
}

void Motor::encoderRightCallback(PhidgetEncoderHandle ch, void* ctx, int position_change,
                                 double time_change, int index_triggered)
{
	std::unique_lock<std::mutex> lk(static_cast<Motor*>(ctx)->encoder_m_);
	static_cast<Motor*>(ctx)->encoder_right_queue_.push(position_change);
	lk.unlock();
	static_cast<Motor*>(ctx)->encoder_cv_.notify_one();
}

void Motor::encoderPublisher()
{
	std::queue<int> left, right;
	Encoder msg;
	while (!done_.load()) {
		std::unique_lock<std::mutex> lk(encoder_m_);
		encoder_cv_.wait(lk, [&] {
			return !encoder_left_queue_.empty() && !encoder_right_queue_.empty();
		});
		std::swap(left, encoder_left_queue_);
		std::swap(right, encoder_right_queue_);
		lk.unlock();

		while (left.size() > right.size()) {
			left.pop();
		}
		while (right.size() > left.size()) {
			right.pop();
		}

		while (!left.empty() && !right.empty()) {
			msg.header.frame_id = "";
			msg.header.stamp = ros::Time::now();

			msg.delta_encoder_left = left.front();
			msg.delta_encoder_right = right.front();
			msg.encoder_left += msg.delta_encoder_left;
			msg.encoder_right += msg.delta_encoder_right;

			encoder_pub_.publish(msg);

			left.pop();
			right.pop();
		}
	}
}

void Motor::pwmCallback(PWM::ConstPtr const& msg)
{
	handleError(PhidgetDCMotor_setTargetVelocity(motor_left_, msg->pwm_left), 5, "Motor");
	handleError(PhidgetDCMotor_setTargetVelocity(motor_right_, msg->pwm_right), 5, "Motor");
}

void Motor::configCallback(MotorConfig& config, uint32_t level) {}
}  // namespace phidgets