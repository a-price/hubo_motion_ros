#ifndef DUMMYPARAMS_H
#define DUMMYPARAMS_H

// TODO: Replace this with ROS message stuff


typedef enum 
{
    T_RIGHT=0,
    T_LEFT=1,
    T_BOTH=3
    
} teleop_arm_t;


typedef struct teleop_params
{
    teleop_arm_t arm;
} teleop_params_t;


#endif // DUMMYPARAMS_H
