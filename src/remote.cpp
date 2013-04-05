#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <sstream>

#include "xq_comms.h"

/** 
 * Manages comms with the gamepad controller and translates that into motor commands
 * to send to quad.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "xq_remote");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("xen_quad_motor_chatter", 1000);

  ros::Rate loop_rate(LOOP_RATE_HZ);

  while (ros::ok())
  {
    /* Create this loop's message */
    std_msgs::Float32MultiArray msg;
    std::vector<float> floats(4);
    floats[0] = 0.1;
    floats[1] = 0.2;
    msg.data = floats;

    /* Send off message */
    chatter_pub.publish(msg);


    /* Wait out till next loop */
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
