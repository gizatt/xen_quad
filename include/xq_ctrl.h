#ifndef XEN_QUAD_CTRL_H
#define XEN_QUAD_CTRL_H

/* Gyro low-pass filter weight */
#define GYRO_LP_WEIGHT (0.5)
/* Acc low-pass filter weight */
#define ACC_LP_WEIGHT (0.5)

/* IMU kalman filtering */
#define K_ANGLE_WEIGHT (2.0)
#define K_GYRO_WEIGHT (3.0)

#endif
