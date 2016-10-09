#ifndef __PHEX_HPP__
#define __PHEX_HPP__

#include <iostream>

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

#define CONVERT_CONSTANT (180/3.1415926535897931)
#define PI 3.1415926535897931

class phex
{
	public:
	phex();
	~phex();
	
	static int init(void);
	static int deinit(void);
	
	
	private:
	
}

#endif
