/* phex_pose class - a pose to move the robot into, and associated functions */

#ifndef __PHEX_POSE_HPP__
#define __PHEX_POSE_HPP__

#include "keyframes.h"
#include <iostream>

class phex_pose
{
	public:
	phex_pose();
	~phex_pose();
	phex_pose(const phex_pose& that);
	phex_pose(const std::string& pose_string);
	phex_pose(int ang[]);
	
	/* moves to this pose, returns 0 if sucessful, 1 if there was an error (servo overload etc) */
	int move(void);
	int movesync(void);
	
	/* set servo number servo to angle, angle is measured from mid point (-150 - +150 degrees)  */
	void set(int servo, int angle);
	
	/* return 0 if all the angles are sane, index of last insane servo(highest index) otherwise */
	int angle_sanity(void);
	
	int angles[PHEX_NUM_SERVOS];
	int old_angles[PHEX_NUM_SERVOS];
};

#endif
