#include "common/common.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo.hh"
#include <gazebo_msgs/ApplyBodyWrench.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Wrench.h"
#include "gazebo/gazebo_ros_api_plugin.h"

/* Handful of defines, should port to include eventually */
#define LOOP_RATE_HZ 10

/**
 * Applies given force to given target. Returns whether message send succeeded.
 * Call *_init before * or bad things will happen!
 */
bool apply_wrench_to_target(const std::string& target_name, ros::ServiceClient client_apply, double fx, double fy, double fz, 
                                             double tx, double ty, double tz, double dura)
{
  gazebo_msgs::ApplyBodyWrench abw_msg;
  ros::Duration dur(dura);
  abw_msg.request.body_name = target_name;
  geometry_msgs::Point wrench_point;
  wrench_point.x = 0;
  wrench_point.y = 0;
  wrench_point.z = 0;
  geometry_msgs::Wrench wrench_itself;
  geometry_msgs::Vector3 forcer; forcer.x = fx; forcer.y = fy; forcer.z = fz;
  geometry_msgs::Vector3 torquer; torquer.x = tx; torquer.y = ty; torquer.z = tz;
  wrench_itself.force = forcer;
  wrench_itself.torque = torquer;
  abw_msg.request.reference_point = wrench_point;
  abw_msg.request.wrench = wrench_itself;
  abw_msg.request.duration = dur;
  client_apply.call(abw_msg);
  ROS_INFO("%d: %s on %s\n", abw_msg.response.success, abw_msg.response.status_message.data(), target_name.data());
  return abw_msg.response.success;
}

/**
 * Waits for motor commands, and applies appropriate forces to quad.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  printf("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flight_controller");
  ros::NodeHandle n;
  ros::Rate loop_rate(LOOP_RATE_HZ);
  ros::Subscriber sub = n.subscribe("nmotor", 1000, chatterCallback);
  ros::ServiceClient client_apply;

  while (ros::ok())
  {
    const std::string s("quadrotor::motor-e");
    apply_wrench_to_target(s, client_apply, 0, 0, 10, 0, 0, 0, 1/LOOP_RATE_HZ);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
