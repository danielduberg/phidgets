// Phidgets
#include <phidgets/spatial.h>
#include <phidgets/util.h>

// ROS
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// STL

namespace phidgets
{
Spatial::Spatial(ros::NodeHandle& nh, ros::NodeHandle& nh_priv) : server_(nh_priv)
{
	imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
	mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
	temp_pub_ = nh.advertise<sensor_msgs::Temperature>("imu/temp", 1);

	int port;
	if (!nh_priv.getParam("imu_port", port)) {
		ROS_FATAL("Must specify 'imu_port'");
		exit(1);
	}

	create();
	setHubPort(port);
	assignEventHandlers();
	attach(nh_priv.param("timeout", PHIDGET_TIMEOUT_DEFAULT));

	// Dynamic reconfigure
	f_ = boost::bind(&Spatial::configCallback, this, _1, _2);
	server_.setCallback(f_);
}

Spatial::~Spatial()
{
	if (EPHIDGET_OK != Phidget_close(reinterpret_cast<PhidgetHandle>(spatial_)) ||
	    EPHIDGET_OK !=
	        Phidget_close(reinterpret_cast<PhidgetHandle>(temperature_sensor_))) {
		handleError();
		exit(2);
	}

	PhidgetSpatial_delete(&spatial_);
	PhidgetTemperatureSensor_delete(&temperature_sensor_);
}

double Spatial::spatialDataRate() const
{
	double r;
	if (EPHIDGET_OK != PhidgetSpatial_getDataRate(spatial_, &r)) {
		handleError();
		exit(3);
	}
	return r;
}

double Spatial::temperatureSensorDataRate() const
{
	double r;
	if (EPHIDGET_OK != PhidgetTemperatureSensor_getDataRate(temperature_sensor_, &r)) {
		handleError();
		exit(3);
	}
	return r;
}

void Spatial::create()
{
	PhidgetSpatial_create(&spatial_);
	PhidgetTemperatureSensor_create(&temperature_sensor_);
}

void Spatial::setHubPort(int port)
{
	if (EPHIDGET_OK !=
	        Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(spatial_), port) ||
	    EPHIDGET_OK != Phidget_setHubPort(
	                       reinterpret_cast<PhidgetHandle>(temperature_sensor_), port)) {
		handleError();
		exit(3);
	}
}

void Spatial::assignEventHandlers()
{
	static std::string spatial_str{"Spatial"};
	PhidgetSpatial_setOnSpatialDataHandler(spatial_, spatialCallback, this);
	PhidgetSpatial_setOnAlgorithmDataHandler(spatial_, algorithmCallback, this);
	Phidget_setOnAttachHandler(reinterpret_cast<PhidgetHandle>(spatial_), attachCallback,
	                           &spatial_str);
	Phidget_setOnDetachHandler(reinterpret_cast<PhidgetHandle>(spatial_), detachCallback,
	                           &spatial_str);
	Phidget_setOnErrorHandler(reinterpret_cast<PhidgetHandle>(spatial_), errorCallback,
	                          &spatial_str);

	static std::string temperature_sensor_str{"Temperature sensor"};
	PhidgetTemperatureSensor_setOnTemperatureChangeHandler(temperature_sensor_,
	                                                       temperatureCallback, this);
	Phidget_setOnAttachHandler(reinterpret_cast<PhidgetHandle>(temperature_sensor_),
	                           attachCallback, &temperature_sensor_str);
	Phidget_setOnDetachHandler(reinterpret_cast<PhidgetHandle>(temperature_sensor_),
	                           detachCallback, &temperature_sensor_str);
	Phidget_setOnErrorHandler(reinterpret_cast<PhidgetHandle>(temperature_sensor_),
	                          errorCallback, &temperature_sensor_str);
}

void Spatial::attach(uint32_t timeout_ms)
{
	if (EPHIDGET_OK != Phidget_openWaitForAttachment(
	                       reinterpret_cast<PhidgetHandle>(spatial_), timeout_ms) ||
	    EPHIDGET_OK !=
	        Phidget_openWaitForAttachment(
	            reinterpret_cast<PhidgetHandle>(temperature_sensor_), timeout_ms)) {
		handleError();
		exit(4);
	}
}

void Spatial::setHeating(bool enable)
{
	if (EPHIDGET_OK != PhidgetSpatial_setHeatingEnabled(spatial_, enable)) {
		handleError();
		exit(4);
	}
}

void Spatial::setSpatialDataRate(double rate)
{
	if (EPHIDGET_OK != PhidgetSpatial_setDataRate(spatial_, rate)) {
		handleError();
		exit(4);
	}
}

void Spatial::setTemperatureSensorDataRate(double rate)
{
	if (EPHIDGET_OK != PhidgetTemperatureSensor_setDataRate(temperature_sensor_, rate)) {
		handleError();
		exit(4);
	}
}

void Spatial::publishSpatial()
{
	sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu);
	imu_msg->header.stamp = ros::Time::now();
	imu_msg->header.frame_id = frame_id_;

	imu_msg->orientation = tf2::toMsg(orientation_);

	imu_msg->angular_velocity = tf2::toMsg(angular_velocity_);

	imu_msg->linear_acceleration = tf2::toMsg(linear_acceleration_);

	imu_pub_.publish(imu_msg);

	sensor_msgs::MagneticField::Ptr msg_msg(new sensor_msgs::MagneticField);
	msg_msg->header = imu_msg->header;
	msg_msg->magnetic_field = tf2::toMsg(magnetic_field_);
	mag_pub_.publish(msg_msg);
}

void Spatial::spatialCallback(PhidgetSpatialHandle ch, void* ctx,
                              double const acceleration[3], double const angular_rate[3],
                              double const magnetic_field[3], double timestamp)
{
	Spatial* s = static_cast<Spatial*>(ctx);

	s->linear_acceleration_ =
	    tf2::Vector3(acceleration[0], acceleration[1], acceleration[2]);
	s->angular_velocity_ = tf2::Vector3(angular_rate[0], angular_rate[1], angular_rate[2]);
	s->magnetic_field_ =
	    tf2::Vector3(magnetic_field[0], magnetic_field[1], magnetic_field[2]);

	s->publishSpatial();
}

void Spatial::algorithmCallback(PhidgetSpatialHandle ch, void* ctx,
                                double const quaternion[4], double timestamp)
{
	static_cast<Spatial*>(ctx)->orientation_ =
	    tf2::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
}

void Spatial::publishTemp(double temp)
{
	sensor_msgs::Temperature::Ptr temp_msg(new sensor_msgs::Temperature);
	temp_msg->header.stamp = ros::Time::now();
	temp_msg->header.frame_id = frame_id_;
	temp_msg->temperature = temp;
	temp_pub_.publish(temp_msg);
}

void Spatial::temperatureCallback(PhidgetTemperatureSensorHandle ch, void* ctx,
                                  double temperature)
{
	static_cast<Spatial*>(ctx)->publishTemp(temperature);
}

void Spatial::configCallback(phidgets::SpatialConfig& config, uint32_t level)
{
	frame_id_ = config.frame_id;

	setHeating(config.heating_enabled);

	if (spatialDataRate() != config.spatial_data_rate) {
		setSpatialDataRate(config.spatial_data_rate);
	}

	if (temperatureSensorDataRate() != config.temperature_data_rate) {
		setTemperatureSensorDataRate(config.temperature_data_rate);
	}
}
}  // namespace phidgets