#ifndef PHIDGETS_MOTOR_H
#define PHIDGETS_MOTOR_H

// Phidgets
#include <phidgets/PWM.h>

// Phidget
extern "C" {
#include <phidget22.h>
}

// ROS
#include <dynamic_reconfigure/server.h>
#include <phidgets/MotorConfig.h>
#include <ros/ros.h>

// STL
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

namespace phidgets
{
class Motor
{
 public:
	Motor(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

	~Motor();

 private:
	void create();

	void setHubPort(int port_left, int port_right);

	void assignEventHandlers();

	void attach(uint32_t timeout_ms);

	static void CCONV encoderLeftCallback(PhidgetEncoderHandle ch, void* ctx,
	                                      int position_change, double time_change,
	                                      int index_triggered);

	static void CCONV encoderRightCallback(PhidgetEncoderHandle ch, void* ctx,
	                                       int position_change, double time_change,
	                                       int index_triggered);

	void encoderPublisher();

	void pwmCallback(PWM::ConstPtr const& msg);

	void configCallback(MotorConfig& config, uint32_t level);

 private:
	ros::Publisher encoder_pub_;

	ros::Subscriber pwm_sub_;

	dynamic_reconfigure::Server<MotorConfig> server_;
	dynamic_reconfigure::Server<MotorConfig>::CallbackType f_;

	std::thread encoder_worker_;
	std::mutex encoder_m_;
	std::condition_variable encoder_cv_;

	PhidgetDCMotorHandle motor_left_;
	PhidgetDCMotorHandle motor_right_;
	PhidgetEncoderHandle encoder_left_;
	PhidgetEncoderHandle encoder_right_;

	std::queue<int> encoder_left_queue_;
	std::queue<int> encoder_right_queue_;
	int encoder_left_acc_{};
	int encoder_right_acc_{};
};
}  // namespace phidgets

#endif  // PHIDGETS_MOTOR_H