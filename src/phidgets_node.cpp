// Phidgets
#include <phidgets/motor.h>
#include <phidgets/spatial.h>

// ROS
#include <dynamic_reconfigure/server.h>
#include <phidgets/PhidgetsConfig.h>
#include <ros/package.h>
#include <ros/ros.h>

// Phidget
extern "C" {
#include <phidget22.h>
}

void callback(phidgets::PhidgetsConfig &config, uint32_t level) {
  PhidgetLog_enable(
      static_cast<Phidget_LogLevel>(config.logging),
      (ros::package::getPath("phidgets") + "/phidgetlog.log").c_str());
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "phidgets");

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  dynamic_reconfigure::Server<phidgets::PhidgetsConfig> server;
  dynamic_reconfigure::Server<phidgets::PhidgetsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  phidgets::Spatial spatial(nh, nh_priv);
  phidgets::Motor motor(nh, nh_priv);

  ros::spin();

  return 0;
}