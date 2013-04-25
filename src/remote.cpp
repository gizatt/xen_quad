#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/Float32MultiArray.h"

#include <sstream>
#include <time.h> 
#include <stdio.h>

#include "xq_comms.h"
#include "xq_ctrl.h"

#define VERBOSE_IMU_CHATTER 1
#define VERBOSE_MAINLOOP_CHATTER 1

float q_x_spin = 0;
float q_y_spin = 0;
float q_z_spin = 0;
float q_z_tran = 0;
bool q_toggle = false;
bool q_toggle_last = false;

/* Trims */
float trimAX = 0;
float trimAY = 0;

/* Known orientation and delta orientation */
float lastKnownAX = 0, lastKnownAY = 0, deltaAX = 0, deltaAY = 0;

/* Log file */
FILE *log_file;

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

/* imu chatter callback that rips out imu chatter info and parses into orientation
   info with weighted average kalmar filter */
float imu_accs[3] = {};
float imu_vels[3] = {};
struct timespec last_timespec = {1, 1};;
struct timespec now_timespec = {1, 1};;
void imuChatterCallback(const sensor_msgs::Imu::ConstPtr& imumsg){
  /* Parse out data into accs and vels and throw them, low-pass filtered, into
    imu accs */
  if (fabs(imu_accs[0]) < 100)
    imu_accs[0] += ACC_LP_WEIGHT*(imumsg->linear_acceleration.x - imu_accs[0]);
  if (fabs(imu_accs[1]) < 100)
    imu_accs[1] += ACC_LP_WEIGHT*(imumsg->linear_acceleration.y - imu_accs[1]);
  if (fabs(imu_accs[2]) < 100)
    imu_accs[2] += ACC_LP_WEIGHT*(imumsg->linear_acceleration.z - imu_accs[2]);
  if (fabs(imu_vels[0]) < 100)
    imu_vels[0] += GYRO_LP_WEIGHT*(imumsg->angular_velocity.x - imu_vels[0]);
  if (fabs(imu_vels[1]) < 100)
   imu_vels[1] += GYRO_LP_WEIGHT*(imumsg->angular_velocity.y - imu_vels[1]);
  if (fabs(imu_vels[2]) < 100)
   imu_vels[2] += GYRO_LP_WEIGHT*(imumsg->angular_velocity.z - imu_vels[2]);

  #if VERBOSE_IMU_CHATTER
  if (log_file)
    fprintf(log_file, "Imu accs and vels: %f, %f, %f, %f, %f, %f\n", imu_accs[0], imu_accs[1], imu_accs[2],
                           imu_vels[0], imu_vels[1], imu_vels[2]);
  #endif

  /* Filter together the x and y directions with kfilter */
  /* First convert to angle by normalizing to g and taking sin*/
  if (imu_accs[0] > 9.8) imu_accs[0] = 9.8;
     else if (imu_accs[0] < -9.8) imu_accs[0] = -9.8;
  if (imu_accs[1] > 9.8) imu_accs[1] = 9.8;
     else if (imu_accs[1] < -9.8) imu_accs[1] = -9.8; 
  float xangle = asin(imu_accs[0]/9.81) * 180. / 3.14;
  float yangle = asin(imu_accs[1]/9.81) * 180. / 3.14;

  #if VERBOSE_IMU_CHATTER
  if (log_file)
    fprintf(log_file, "Imu accs and vels: %f, %f, %f, %f, %f, %f\n", imu_accs[0], imu_accs[1], imu_accs[2],
                           imu_vels[0], imu_vels[1], imu_vels[2]);
  #endif

  clock_gettime(CLOCK_REALTIME, &now_timespec);
  long elapsed_ns = 1000000000*((long)(now_timespec.tv_sec - last_timespec.tv_sec)) +
                           (now_timespec.tv_nsec - last_timespec.tv_nsec);
  float timeElapsed = ((float)elapsed_ns)/1000000;
  last_timespec.tv_sec = now_timespec.tv_sec;
  last_timespec.tv_nsec = now_timespec.tv_nsec;

  #if VERBOSE_IMU_CHATTER
  if (log_file)
    fprintf(log_file, "Elapsed: %f, Xang: %f, YAng: %f\n", timeElapsed, xangle, yangle);
  #endif

  if (fabs(timeElapsed) < 10.0){
    /* And generate actual angles and delta angles */
    xangle = (K_ANGLE_WEIGHT*xangle + K_GYRO_WEIGHT*(lastKnownAX + imu_vels[0]*timeElapsed))/(K_ANGLE_WEIGHT+K_GYRO_WEIGHT);
    yangle = (K_ANGLE_WEIGHT*yangle + K_GYRO_WEIGHT*(lastKnownAY + imu_vels[1]*timeElapsed))/(K_ANGLE_WEIGHT+K_GYRO_WEIGHT);
    /* Add in trims */
    xangle += trimAX; yangle += trimAY;

    /* Find deltas */
    deltaAX = (float)((xangle - lastKnownAX) / timeElapsed);
    deltaAY = (float)((yangle - lastKnownAY) / timeElapsed);

    /* Finally update global estimations */
    lastKnownAX = xangle;
    lastKnownAY = yangle;
  }
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
  ros::Subscriber imu_listener = n.subscribe("xen_quad_imu_chatter", 1000, imuChatterCallback);

  /* Open log file */
  log_file = fopen("/home/gizatt/xen_quad_log.csv", "a");
  if (log_file == NULL){
    ROS_ERROR("XEN_QUAD: Couldn't open log file!");
    return 1;
  }
  fprintf(log_file, "\n\n*********************************************\n");
  fprintf(log_file, "**************** NEW SESSION ****************\n");
  fflush(log_file);

  /* Create joystick listener */
  ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

  ros::Rate loop_rate(LOOP_RATE_HZ);
   
  int iter = 0;
  while (ros::ok())
  {
    /* Create this loop's message */
    std_msgs::Float32MultiArray msg;
    /* Populate */
    std::vector<float> floats(4);
    
    #if VERBOSE_MAINLOOP_CHATTER
    fprintf(log_file, "%d, %f, %f\n", iter, lastKnownAX, lastKnownAX);
    #endif
    iter++;
    /* Sanity check orientation info, reset if stupid */
    if (fabs(lastKnownAX) > 10000)   lastKnownAX = 0;
    if (fabs(lastKnownAY) > 10000)   lastKnownAY = 0;

    /* Assembly message in order N S E W */
    
    /* z-level */
    floats[0] = fabs(q_z_tran);
    floats[1] = fabs(q_z_tran);
    floats[2] = fabs(q_z_tran);
    floats[3] = fabs(q_z_tran);

    /* Rotation around arms */ 
    floats[0] += q_y_spin - lastKnownAY/200.;
    floats[1] += -q_y_spin + lastKnownAY/200.;
    floats[2] += q_x_spin - lastKnownAX/200.;
    floats[3] += -q_x_spin + lastKnownAX/200.;
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
  fclose(log_file);

  return 0;
}
