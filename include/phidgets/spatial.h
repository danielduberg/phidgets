#ifndef PHIDGETS_SPATIAL_H
#define PHIDGETS_SPATIAL_H

// Phidget
extern "C" {
#include <phidget22.h>
}

// ROS
#include <dynamic_reconfigure/server.h>
#include <phidgets/SpatialConfig.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

// STL
#include <limits>
#include <string>

namespace phidgets {
class Spatial {
 public:
  Spatial(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

  ~Spatial();

  double spatialDataRate() const;

  double temperatureSensorDataRate() const;

 private:
  void create();

  void setHubPort(int port);

  void assignEventHandlers();

  void attach(uint32_t timeout_ms);

  void setHeating(bool enable);

  void setSpatialDataRate(double rate);

  void setTemperatureSensorDataRate(double rate);

  void publishSpatial();

  static void CCONV spatialCallback(PhidgetSpatialHandle ch, void *ctx,
                                    double const acceleration[3],
                                    double const angular_rate[3],
                                    double const magnetic_field[3],
                                    double timestamp);

  static void CCONV algorithmCallback(PhidgetSpatialHandle ch, void *ctx,
                                      double const quaternion[4],
                                      double timestamp);

  void publishTemp(double temp);

  static void CCONV temperatureCallback(PhidgetTemperatureSensorHandle ch,
                                        void *ctx, double temperature);

  static void CCONV attachCallback(PhidgetHandle ch, void *ctx);

  static void CCONV detachCallback(PhidgetHandle ch, void *ctx);

  static void CCONV errorCallback(PhidgetHandle ch, void *ctx,
                                  Phidget_ErrorEventCode code,
                                  char const *description);

  void configCallback(phidgets::SpatialConfig &config, uint32_t level);

 private:
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;
  ros::Publisher temp_pub_;

  dynamic_reconfigure::Server<phidgets::SpatialConfig> server;
  dynamic_reconfigure::Server<phidgets::SpatialConfig>::CallbackType f;

  PhidgetSpatialHandle spatial_{};
  PhidgetTemperatureSensorHandle temperature_sensor_{};

  std::string frame_id_{"imu_link"};

  tf2::Quaternion orientation_;
  tf2::Vector3 angular_velocity_;
  tf2::Vector3 linear_acceleration_;

  tf2::Vector3 magnetic_field_;
};

void CCONV accelerationChange(PhidgetAccelerometerHandle ch, void *ctx,
                              double const acceleration[3], double timestamp);
}  // namespace phidgets

#endif  // PHIDGETS_SPATIAL_H