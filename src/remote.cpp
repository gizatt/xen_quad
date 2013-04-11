#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "std_msgs/Float32MultiArray.h"

#include <sstream>

#include "xq_comms.h"


float q_x_spin = 0;
float q_y_spin = 0;
float q_z_spin = 0;
float q_z_tran = 0;
bool q_toggle = false;
bool q_toggle_last = false;

/* Joystick callback; will parse out joystick buttons and record important values */
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  /* Parse out data into particular sets of axes and a given toggle button. */
  q_x_spin = joy->axes[XQ_X_SPIN_OFF];
  q_y_spin = joy->axes[XQ_Y_SPIN_OFF];
  q_z_spin = joy->axes[XQ_Z_SPIN_OFF];
  q_z_tran = joy->axes[XQ_Z_TRAN_OFF];

  if (joy->buttons[XQ_TOGGLE_OFF] && joy->buttons[XQ_TOGGLE_OFF] != q_toggle_last){
    q_toggle = !q_toggle;
  }
  q_toggle_last = joy->buttons[XQ_TOGGLE_OFF];
}

/** 
 * Manages comms with the gamepad controller and translates that into motor commands
 * to send to quad.
 */
int main(int argc, char **argv)
{
  int i;
  ros::init(argc, argv, "xq_remote");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("xen_quad_motor_chatter", 1000);

  /* Create joystick listener */
  ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

  ros::Rate loop_rate(LOOP_RATE_HZ);

  while (ros::ok())
  {
    /* Create this loop's message */
    std_msgs::Float32MultiArray msg;
    /* Populate */
    std::vector<float> floats(4);

    /* Assembly message in order N S E W */
    
    /* z-level */
    floats[0] = fabs(q_z_tran);
    floats[1] = fabs(q_z_tran);
    floats[2] = fabs(q_z_tran);
    floats[3] = fabs(q_z_tran);

    /* Rotation around arms */
    floats[0] += q_y_spin;
    floats[1] -= q_y_spin;
    floats[2] += q_x_spin;
    floats[3] -= q_x_spin;

    /* Yaw */
    floats[0] += q_z_spin;
    floats[1] += q_z_spin;
    floats[2] -= q_z_spin;
    floats[3] -= q_z_spin;

    for (i=0; i<3; i++){
      if (floats[i]<0) floats[i] = 0;
      else if (floats[i] > 1) floats[i] = 1;
    }

    msg.data = floats;
    /* Send off message */
    chatter_pub.publish(msg);


    /* Wait out till next loop */
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
