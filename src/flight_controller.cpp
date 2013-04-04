#include <math.h>

#include "common/common.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "gazebo.hh"
#include <gazebo_msgs/ApplyBodyWrench.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Wrench.h"

/* Handful of defines, should port to include eventually */
/* Controller update loop rate */
#define LOOP_RATE_HZ 10
/* Max rad/sec motors achieve at max thrust */
#define MAX_MOTOR_W (1313.0f)
/* Prop proportionality constant; assuming Thrust (N) = K * w^3, knowing that max
    T is 0.69kg, and max w is above, then K = */
#define PROP_K (0.69f / (MAX_MOTOR_W*MAX_MOTOR_W*MAX_MOTOR_W))
/* Prop torque proportionality constant: really just a guess to make it reasonable
  at max thrust. */
#define PROP_TORQUE_K (0.1f / (MAX_MOTOR_W));

/* Global record of current commanded quad state */
double quad_cmds[] = {0.0f, 0.0f, 0.0f, 0.0f};

/**
 * Applies given force to given target. Returns whether message send succeeded.
 * Call *_init before * or bad things will happen!
 */
bool apply_wrench_to_target(const char * target_name, ros::ServiceClient client_apply, double fx, double fy, double fz, 
                                             double tx, double ty, double tz, double dura)
{
  gazebo_msgs::ApplyBodyWrench abw_msg;
  ros::Duration dur(dura);
  abw_msg.request.body_name = target_name;
  abw_msg.request.reference_frame = target_name;
  geometry_msgs::Point wrench_point;
  wrench_point.x = 0;
  wrench_point.y = 0;
  wrench_point.z = 0;
  abw_msg.request.reference_point = wrench_point;
  geometry_msgs::Wrench wrench_itself;
  geometry_msgs::Vector3 forcer; forcer.x = fx; forcer.y = fy; forcer.z = fz;
  geometry_msgs::Vector3 torquer; torquer.x = tx; torquer.y = ty; torquer.z = tz;
  wrench_itself.force = forcer;
  wrench_itself.torque = torquer;
  abw_msg.request.wrench = wrench_itself;
  abw_msg.request.duration = dur;
  client_apply.call(abw_msg);
  //ROS_INFO("%d: %s on %s\n", abw_msg.response.success, abw_msg.response.status_message.data(), target_name);
  return abw_msg.response.success;
}

/**
 * Waits for motor commands, and applies appropriate forces to quad. Expects motor commands as floats
 * between 0 and 1, in an array of 4 (in float32 multi array stdmsg), in order N,S,E,W
 */
void motorChatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  /* Copy over contents into local store */
  for (int i=0; i<4; i++)
    quad_cmds[i] = msg->data[i];
}

int main(int argc, char **argv)
{
  double forces[4];
  double torques[4];
  int i;

  ros::init(argc, argv, "flight_controller");
  ros::NodeHandle n;
  ros::Rate loop_rate(LOOP_RATE_HZ);
  ros::Subscriber sub = n.subscribe("xen_quad_motor_chatter", 1000, motorChatterCallback);
  ros::ServiceClient client_apply = n.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

  while (ros::ok())
  {
    /* Calculate motor forces and torques */
    /* We're saying that the angular velocity scales linearly with its input, and force is
      related to angular velocity^3. Using measured max force for our motor and prop,
      we derive the proportionality constant K, which we use here... see include for that info. 
      We're also assuming torque from prop is linearly related to angular velocity... close enough,
      hopefully. */
    for (i=0; i<4; i++){
      forces[i] = pow(quad_cmds[i]*MAX_MOTOR_W, 3.0f)*PROP_K;
      torques[i] = quad_cmds[i]*MAX_MOTOR_W*PROP_TORQUE_K;
    }
    apply_wrench_to_target("quadrotor::motor-n", client_apply, 0, 0, forces[0], 0, 0, torques[0], 1/(double)LOOP_RATE_HZ);
    apply_wrench_to_target("quadrotor::motor-s", client_apply, 0, 0, forces[1], 0, 0, torques[1], 1/(double)LOOP_RATE_HZ);
    apply_wrench_to_target("quadrotor::motor-e", client_apply, 0, 0, forces[2], 0, 0, torques[2], 1/(double)LOOP_RATE_HZ);
    apply_wrench_to_target("quadrotor::motor-w", client_apply, 0, 0, forces[3], 0, 0, torques[3], 1/(double)LOOP_RATE_HZ);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
