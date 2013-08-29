#ifndef DUMMYPARAMS_H
#define DUMMYPARAMS_H

// TODO: Replace this with ROS message stuff


typedef enum 
{
    T_RIGHT=0,
    T_LEFT=1,
    T_BOTH=2
    
} teleop_arm_t;

typedef enum
{
    T_LOOSEN=0,
    T_GRASP=1,
    T_OPEN=2
} teleop_fin_t;


typedef struct teleop_params
{
    teleop_arm_t arm;
    teleop_fin_t leftFin;
    teleop_fin_t rightFin;
    teleop_fin_t trigFin;

} teleop_params_t;


#endif // DUMMYPARAMS_H
