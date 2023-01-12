#ifndef PHIDGETS_MOTOR_H
#define PHIDGETS_MOTOR_H

// Phidget
extern "C" {
#include <phidget22.h>
}

namespace phidgets
{
class Motor
{
 public:
	Motor(int port, uint32_t attach_timeout_ms);

	~Motor();

	double acceleration() const;

	void setAcceleration(double acceleration);

	double targetBrakingStrength() const;

	void setTargetBrakingStrength(double braking);

	double brakingStrength() const;

	double currentLimit() const;

	void setCurrentLimit(double limit);

	double dataRate() const;

	void setDataRate(double rate);

	void setFailsafe(uint32_t time_ms);

	void resetFailsafe();

	double targetVelocity() const;

	void setTargetVelocity(double velocity);

	double velocity() const;

	int port() const;

 private:
	void create();

	void assignEventHandlers();

	void init();

	static void CCONV attachCallback(PhidgetHandle ch, void *ctx);

	static void CCONV detachCallback(PhidgetHandle ch, void *ctx);

	static void CCONV errorCallback(PhidgetHandle ch, void *ctx,
	                                Phidget_ErrorEventCode code, char const *description);

 private:
	PhidgetDCMotorHandle motor_;

	int port_;

	double acceleration_{-1};
	double target_braking_strength_{-1};
	double current_limit_{-1};
	double data_rate_{-1};
	uint32_t failsafe_time_{0};
};
}  // namespace phidgets

#endif  // PHIDGETS_MOTOR_H