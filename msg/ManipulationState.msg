# Command State Definitions
uint8 READY=0
uint8 TRANS_EULER=1
uint8 TRANS_QUAT=2
uint8 TRAJ=3

# Grasping State Definitions
uint8 GRASP_LIMP=0
uint8 GRASP_STATIC=1
uint8 GRASP_NOW=2
uint8 GRASP_AT_END=3
uint8 GRASP_RELEASE_NOW=4
uint8 GRASP_RELEASE_AT_END=5

# Error State Definitions
uint8 NO_ERROR=0
uint8 INVALID_POSE=1
uint8 INVALID_TRANSITION=2
uint8 BROKEN_CHAIN=3

uint8 CommandState
uint8 GraspState
uint8 ErrorState
