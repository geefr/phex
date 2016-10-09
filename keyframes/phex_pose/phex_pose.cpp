#include "phex_pose.hpp"
#include "keyframes.h"
#include "dynamixel.h"
#include <iostream>
#include <sstream>

using namespace std;

/* default constructor - sets angles to default pose - really this    *
 * shouldn't be hardcoded to 18 servos, at least read from a file     */
phex_pose::phex_pose()
{
	angles[0]=600;
	angles[1]=400;
	angles[2]=500;
	angles[3]=500;
	angles[4]=200;
	angles[5]=200;
	angles[6]=500;
	angles[7]=500;
	angles[8]=500;
	angles[9]=500;
	angles[10]=200;
	angles[11]=200;
	angles[12]=400;
	angles[13]=600;
	angles[14]=500;
	angles[15]=500;
	angles[16]=200;
	angles[17]=200;
}

phex_pose::~phex_pose()
{
	
}

phex_pose::phex_pose(const phex_pose& that)
{
		for(int i=0;i<PHEX_NUM_SERVOS;i++)
			angles[i]=that.angles[i];
}
	
/* 
 * initialise the pose from a give string:                             
 * format: x [ y y y y y y ... 
 * x: number of servos
 */
phex_pose::phex_pose(const std::string& pose_string)
{
	std:;string num_servos_ignored="";
	std::string junk_str;
	std::stringstream pose_stream(pose_string);
	pose_stream >> num_servos_ignored >> junk_str;
	//if it's an IK pose, find the angles from each legs coords
	if(num_servos_ignored=="IK")
	{
		
	}
	else
	{
		for(int i=0;i<PHEX_NUM_SERVOS;i++)
			pose_stream >> angles[i];
	}
}
	
phex_pose::phex_pose(int ang[])
{
	for(int i=0;i<PHEX_NUM_SERVOS;i++)
		angles[i]=ang[i];
}
	
	/* moves to this pose, returns 0 if sucessful, 1 if there was an error (servo overload etc) */
int phex_pose::move(void)
{
	int insane=angle_sanity();
	if(insane==0)
	{
		//NOTE: numbering starts at 1, as that's where the servo indexes start
		for(int i=1;i<=PHEX_NUM_SERVOS;i++)
		{
			if(angles[i-1]!=old_angles[i-1])
			{
				dxl_write_word(i,P_GOAL_POSITION_L,(int)(((float)angles[i-1]/300)*1024));
				old_angles[i-1]=angles[i-1];
			 	#ifdef EBUG
					std::cerr << "INFO: moving servo: " << i << " To absolute value: " << angles[i-1] << std::endl;
				#endif
			}
		}
		int moving=1;
		/* TODO - fix this so it's not hardcoded */
		while(moving)
		{
			moving=
			(
			dxl_read_byte( 1, P_MOVING) ||
			dxl_read_byte( 2, P_MOVING) ||
			dxl_read_byte( 3, P_MOVING) ||
			dxl_read_byte( 4, P_MOVING) ||
			dxl_read_byte( 5, P_MOVING) ||
			dxl_read_byte( 6, P_MOVING) ||
			dxl_read_byte( 7, P_MOVING) ||
			dxl_read_byte( 8, P_MOVING) ||
			dxl_read_byte( 9, P_MOVING) ||
			dxl_read_byte(10, P_MOVING) ||
			dxl_read_byte(11, P_MOVING) ||
			dxl_read_byte(12, P_MOVING) ||
			dxl_read_byte(13, P_MOVING) ||
			dxl_read_byte(14, P_MOVING) ||
			dxl_read_byte(15, P_MOVING) ||
			dxl_read_byte(16, P_MOVING) ||
			dxl_read_byte(17, P_MOVING) ||
			dxl_read_byte(18, P_MOVING)
			);		
		}
	}
	else
	{
		std::cerr << "WARNING: a servo " << insane << " is insane, you might want to check the input value for it" << std::endl;
	}
	/* TODO - fix this, should read packet status and report an error if there is one */
	return 0;
}
	
int phex_pose::movesync(void)
{
	int insane=angle_sanity();
	if(insane==0)
	{
		// Set goal speed
		dxl_write_word( BROADCAST_ID, P_GOAL_SPEED_L, 0x7E );
		
		// Make syncwrite packet
		dxl_set_txpacket_id(BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
		dxl_set_txpacket_parameter(1, 2);
		for(int i(0); i<PHEX_NUM_SERVOS; i++ )
		{
			//if(angles[i]!=old_angles[i])
			//{
				dxl_set_txpacket_parameter(2+3*i, i+1);
				int GoalPos = (int)(((float)angles[i]/300)*1024);
				//cerr << GoalPos << endl;
				
				dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(GoalPos));
				dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(GoalPos));
			//}
		}
		
		dxl_set_txpacket_length((2+1)*PHEX_NUM_SERVOS+4);
		

		cerr << endl;
		
		dxl_txrx_packet();
		
		/*int CommStatus = dxl_get_result();
		if( CommStatus == COMM_RXSUCCESS )
		{
			//PrintErrorCode();
		}
		else
		{
			//PrintCommStatus(CommStatus);
			
		}*/	
		
		
		
		
		int moving=1;
		/* TODO - fix this so it's not hardcoded - interpolate movement, wait the right amount */
		while(moving)
		{
			moving=
			(
				dxl_read_byte( 1, P_MOVING) ||
				dxl_read_byte( 2, P_MOVING) ||
				dxl_read_byte( 3, P_MOVING) ||
				dxl_read_byte( 4, P_MOVING) ||
				dxl_read_byte( 5, P_MOVING) ||
				dxl_read_byte( 6, P_MOVING) ||
				dxl_read_byte( 7, P_MOVING) ||
				dxl_read_byte( 8, P_MOVING) ||
				dxl_read_byte( 9, P_MOVING) ||
				dxl_read_byte(10, P_MOVING) ||
				dxl_read_byte(11, P_MOVING) ||
				dxl_read_byte(12, P_MOVING) ||
				dxl_read_byte(13, P_MOVING) ||
				dxl_read_byte(14, P_MOVING) ||
				dxl_read_byte(15, P_MOVING) ||
				dxl_read_byte(16, P_MOVING) ||
				dxl_read_byte(17, P_MOVING) ||
				dxl_read_byte(18, P_MOVING)
			);		
		}
	}
	else
	{
		std::cerr << "WARNING: servo " << insane << " is insane, you might want to check the input value for it" << std::endl;
	}
	/* TODO - fix this, should read packet status and report an error if there is one */
	return 0;
}	
	
int phex_pose::angle_sanity(void)
{
	//checks for sanity, limits to 60->240 degrees
	int insane(0);
	for(int i=0;i<PHEX_NUM_SERVOS;i++)
	{
		if(angles[i]<0 || angles[i]>300)
		//if(angles[i]<30 || angles[i]>270)
			insane=i+1;
	}
	return insane;
}
	
void phex_pose::set(int servo, int angle)
{
	angles[servo-1] = ((angle+150)/360)*1000;
}
