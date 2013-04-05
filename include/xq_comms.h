#ifndef XEN_QUAD_COMMS_H
#define XEN_QUAD_COMMS_H

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


#endif
