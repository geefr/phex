#ifndef __KEYFRAMES_H__
#define __KEYFRAMES_H__

#define PHEX_NUM_SERVOS 18

//constants
#define PHEX_BODY_WIDTH 195
#define PHEX_BODY_HEIGHT 177
#define PHEX_BODY_DEPTH 50
#define PHEX_COXA_LENGTH 50
#define PHEX_FEMUR_LENGTH 130
#define PHEX_TIBIA_LENGTH 92

// Servo settings
// Control table address
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_GOAL_SPEED_L		32
#define P_GOAL_SPEED_H		33
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING		46

// Default setting
#define DEFAULT_BAUDNUM		1 // 1Mbps
#define DEFAULT_ID		2

#define RAD_CONV_CONST (180/3.1415926535897931)
#define PI 3.1415926535897931
#define WALK_STEPS 6

struct pose
{
	int servos[6][3];
	int moving;
};

#endif
