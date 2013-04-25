#ifndef XEN_QUAD_CTRL_H
#define XEN_QUAD_CTRL_H

/* Remote update loop rate */
#define LOOP_RATE_HZ 20

/* Gyro low-pass filter weight */
#define GYRO_LP_WEIGHT (0.2)
/* Acc low-pass filter weight */
#define ACC_LP_WEIGHT (0.2)

/* IMU kalman filtering */
#define K_ANGLE_WEIGHT (2.0)
#define K_GYRO_WEIGHT (3.0)

#endif
