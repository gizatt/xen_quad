#ifndef XEN_QUAD_COMMS_H
#define XEN_QUAD_COMMS_H

/* Controller update loop rate */
#define LOOP_RATE_HZ 20

/* Max rad/sec motors achieve at max thrust */
#define MAX_MOTOR_W (1313.0f)
/* Prop proportionality constant; assuming Thrust (N) = K * w^3, knowing that max
    T is 0.69kg, and max w is above, then K = */
#define PROP_K (9.8f * 0.69f / (MAX_MOTOR_W*MAX_MOTOR_W*MAX_MOTOR_W))
/* Prop torque proportionality constant: really just a guess to make it reasonable
  at max thrust. */
#define PROP_TORQUE_K (0.005f / (MAX_MOTOR_W));


/* Controller axis offsets into axes and button arrays */
#define XQ_X_SPIN_OFF 0
#define XQ_Y_SPIN_OFF 1
#define XQ_Z_SPIN_OFF 3
#define XQ_Z_TRAN_OFF 4
#define XQ_TOGGLE_OFF 1

#endif
