//##########################################################
//##                      R O B O T I S                   ##
//##          ReadWrite Example code for Dynamixel.       ##
//##                                           2009.11.10 ##
//##########################################################
//#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include "dynamixel.h"
#include "keyframes.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>

//include the joystick header from wordwarvi


using namespace std;

#include "phex_pose.hpp"

int setup_walk();
void do_walk();
int move_pose(struct pose* pose);
int get_position(int k);
int inverse_kinematics_test(int x, int y, int z);

int moving=0;

struct pose walk[WALK_STEPS];
vector<phex_pose*> walk_forward_vector;

void step_forward(void)
{
	vector<phex_pose*>::iterator it;
	for(it=walk_forward_vector.begin();it!=walk_forward_vector.end();it++)
		(*it)->movesync();
}

/* test the IK calculations, x,y,z is the new point to move the body to 
 * (this code calculates the leg angles needed for the movement) */
int inverse_kinematics_test(float x, float y, float z)
{

	
	//mega arrays for points etc. - magic numbers taken from IKmegasheet.ods
	//coordinates of leg tops
	int legtopcoords[6][3];
	//foot positions - these won't move for now, if a foot needs to move then nothing will happen
	int footcoords[6][3];
	//the 3 angles for each leg
	float legangles[6][3];
	
	//1
	legtopcoords[0][0]=x+(PHEX_BODY_WIDTH/2);
	legtopcoords[0][1]=y+(PHEX_BODY_HEIGHT/2);
	legtopcoords[0][2]=z-(PHEX_BODY_DEPTH/2);
	//2
	legtopcoords[1][0]=x-(PHEX_BODY_WIDTH/2);
	legtopcoords[1][1]=y+(PHEX_BODY_HEIGHT/2);
	legtopcoords[1][2]=z-(PHEX_BODY_DEPTH/2);
	//3
	legtopcoords[2][0]=x+(PHEX_BODY_WIDTH/2);
	legtopcoords[2][1]=y;
	legtopcoords[2][2]=z-(PHEX_BODY_DEPTH/2);
	//4
	legtopcoords[3][0]=x-(PHEX_BODY_WIDTH/2);
	legtopcoords[3][1]=y;
	legtopcoords[3][2]=z-(PHEX_BODY_DEPTH/2);
	//5
	legtopcoords[4][0]=x+(PHEX_BODY_WIDTH/2);
	legtopcoords[4][1]=y-(PHEX_BODY_HEIGHT/2);
	legtopcoords[4][2]=z-(PHEX_BODY_DEPTH/2);
	//6
	legtopcoords[5][0]=x-(PHEX_BODY_WIDTH/2);
	legtopcoords[5][1]=y-(PHEX_BODY_HEIGHT/2);
	legtopcoords[5][2]=z-(PHEX_BODY_DEPTH/2);
	
	for(int i(0);i<6;i++)
	{
		cerr << "legtop for leg: " << i+1 << ": " << legtopcoords[i][0] << "," << legtopcoords[i][1] << "," << legtopcoords[i][2] << endl;
	}
	/*
	//foot coords, hardcoded with magic for now
	//1
	footcoords[0][0]=227.5;
	footcoords[0][1]=88.5;
	footcoords[0][2]=-117;
	//2
	footcoords[1][0]=-227.5;
	footcoords[1][1]=88.5;
	footcoords[1][2]=-117;
	//3
	footcoords[2][0]=227.5;
	footcoords[2][1]=0;
	footcoords[2][2]=-117;
	//4
	footcoords[3][0]=-227.5;
	footcoords[3][1]=0;
	footcoords[3][2]=-117;
	//5
	footcoords[4][0]=227.5;
	footcoords[4][1]=-88.5;
	footcoords[4][2]=-117;
	//6
	footcoords[5][0]=-227.5;
	footcoords[5][1]=-88.5;
	footcoords[5][2]=-117;
	*/
	
	//foot coords, hardcoded with magic for now
	//1
	footcoords[0][0]=220;
	footcoords[0][1]=180;
	footcoords[0][2]=-150;
	//2
	footcoords[1][0]=-220;
	footcoords[1][1]=180;
	footcoords[1][2]=-150;
	//3
	footcoords[2][0]=220;
	footcoords[2][1]=0;
	footcoords[2][2]=-150;
	//4
	footcoords[3][0]=-220;
	footcoords[3][1]=0;
	footcoords[3][2]=-150;
	//5
	footcoords[4][0]=220;
	footcoords[4][1]=-180;
	footcoords[4][2]=-150;
	//6
	footcoords[5][0]=-220;
	footcoords[5][1]=-180;
	footcoords[5][2]=-150;
	
	//calculate angles for each leg in turn
	for(int i(0);i<6;i++)
	{
		float px,py,pz,ll,hf,a1,a1_deg,a2,a2_deg,f_theta,b1,b1_deg,t_theta,c_theta,cx;
		//calculate the x,y,z of the foot wrt the top of the leg
		px=(float)(footcoords[i][0]-legtopcoords[i][0]);
		py=(float)(footcoords[i][1]-legtopcoords[i][1]);
		pz=(float)(footcoords[i][2]-legtopcoords[i][2]);
		
		//cerr << endl << "====LEG " << i+1 << "====" << endl;
		//cerr << "P for leg: " << i+1 << ": " << px << "," << py << "," << pz << endl;
		
		//leg length, ll
		ll=(float)(sqrt((px*px)+(py*py)));
		
		//hf - no idea what it stands for, stole it from someone elses maths XD
		hf=(float)(sqrt(((ll-PHEX_COXA_LENGTH)*(ll-PHEX_COXA_LENGTH))+(pz*pz)));
		//hf=(float)(sqrt((px*px)+(pz*pz))); - apparently no idea that it's just ll-cl either
		//hf=(float)ll-PHEX_COXA_LENGTH; - wow that was a stupid idea, failed even worse than that one ^^
		
		//a1
		//old a1
		//a1=(float)(atan((ll-PHEX_COXA_LENGTH)/pz));
		//a1=(float)(atan2((ll-PHEX_COXA_LENGTH),pz));
		//new a1 (for a2-a1=f_theta maths)
		//a1=(float)(pz/(px-PHEX_COXA_LENGTH)) -- What the fuck was this???
		
		
		if(pz<0)
		{
			a1=(float)(asin(pz/hf)); //actually ax, but the 90+/- is moved to the a1_deg assignment 
			if(a1<0)a1*=-1.0;
			a1_deg=90-((float)(a1*RAD_CONV_CONST));
		}
		else if(pz==0)
		{
			a1=90.0;
		}
		else if(pz>0)
		{
			a1=(float)(asin(pz/hf));
			if(a1<0)a1=-1.0;
			a1_deg=90+((float)(a1*RAD_CONV_CONST));
		}
		
		
		
		//float a1_deg((float)(a1*(180/PI)));
		//if(a1_deg<=0)a1_deg+=180;
		
		//a2
		a2=(float)(acos((((PHEX_TIBIA_LENGTH*PHEX_TIBIA_LENGTH)-(PHEX_FEMUR_LENGTH*PHEX_FEMUR_LENGTH)-(hf*hf))/
						(-2*PHEX_FEMUR_LENGTH*hf))));
		a2_deg=(float)(a2*RAD_CONV_CONST);
		//if(a2_deg<=0)a2_deg+=180;
		
		//b1
		b1=(float)acos(((hf*hf)-(PHEX_TIBIA_LENGTH*PHEX_TIBIA_LENGTH)-(PHEX_FEMUR_LENGTH*PHEX_FEMUR_LENGTH))/
					   (-2*PHEX_FEMUR_LENGTH*PHEX_TIBIA_LENGTH));
		b1_deg=(float)(b1*RAD_CONV_CONST);
		
		//c_theta
		if(py>0)
		{	
			//cx=(float)(atan2f(py,px)); // atan2f gives some really weird results, not sure why
			cx=(float)(atan(py/px));
			if(cx<0)cx*=-1.0; //correcting the sign
			//c_theta=180-90-(cx*(180/PI))+60; - WRONG!
			c_theta=90+(cx*RAD_CONV_CONST)+60;
		}
		else if(py==0)
		{
			cx=0;
			c_theta=150;
		}
		else if(py<0)
		{
			//cx=(float)(atan2f(py,px));
			cx=(float)(atan(py/px));
			if(cx<0)cx*=-1.0; //correcting the sign
			//c_theta=90+(cx*(180/PI))+60; - WRONG!
			c_theta=180-90-(cx*RAD_CONV_CONST)+60;
		}
		if((i+1)%2==0) //c_theta is indexed backwards on the even legs, as that's how the robot is built (would look bad the other way round wouldn't it XD)
		{
			c_theta=300-c_theta;
		}
		
		//cerr << "c_theta: " << c_theta << " cx:" << cx << " cx_deg:" << (cx*(180/PI)) << endl;
		//c_theta=(float)(c_theta*(180.0/PI));
		
		//if(c_theta>=60)c_theta-=180; - seriously, what the crap?
		//if(c_theta<=-60)c_theta+=180;
		
		//f_theta
		//f_theta=(float)(90-((a1*(180.0/PI))+(a2*(180.0/PI))));
		//f_theta=(float)(300-30-a1_deg-a2_deg);
		f_theta=((180-a2_deg-a1_deg)+60);
		//cerr << "f_theta: " << f_theta << " a1:" << a1 << "a2:" << a2 << endl;
		//f_theta=(float)(150-f_theta);
		
		
		//t_theta
		//t_theta=(float)(90.0-b1_deg);
		t_theta=(b1_deg-30);
		
		
		
		//cerr << "values(raw), leg: " << i+1 << ":" << " ll:" << ll << " hf:" << hf << " a1_deg:" << a1_deg << " a2_deg:" << a2_deg << " b1:" << b1	<< " b1_deg:" << b1_deg << endl;
		
		
		//cerr << "angles(deg, raw) for leg: " << i+1 << ": " << c_theta << ", " << f_theta << ", " << t_theta << endl;
		
		//cerr << "angles(deg, c_theta-rad->deg) for leg: " << i+1 << ": " << c_theta << ", " << f_theta << ", " << t_theta << endl;
		
		
		//play with the angles a bit, as some of the servos are backwards (makes sense doesn't it)
		//c_theta-=180;
		
				
		//f_theta*=-1.0;
		//f_theta-=90; //TODO - is this correct? i think it is but best check
		//t_theta*=-1.0;
		
		//cerr << "angles(deg, after *-1.0) for leg: " << i+1 << ": " << c_theta << ", " << f_theta << ", " << t_theta << endl;
		//attempt to find the correct solution, with a very basic check of if it's wron, invert femur and tibia angles
		//this probably won't work, and there's still the matter of teh 6th leg, which is doing really weird things
		/*if(f_theta>=0)
		{
			f_theta*=-1.0;
			t_theta*=-1.0;
		}*/
		
		//cerr << "angles(deg, before +150) for leg: " << i+1 << ": " << c_theta << ", " << f_theta << ", " << t_theta << endl;
		
		
		
		legangles[i][0]=c_theta; 
		legangles[i][1]=f_theta;
		//legangles[i][2]=t_theta+150.0;
		legangles[i][2]=t_theta;
		//legangles[i][2]=150.0-t_theta;
		
		//cerr << "angles(deg) for leg: " << i+1 << ": " << legangles[i][0] << ", " << legangles[i][1] << ", " << legangles[i][2] << endl;
				
		//convert from degrees to the absolute servo positions (need to invert for one of the sides probably
		/*legangles[i][0]/=300;
		legangles[i][1]/=300;
		legangles[i][2]/=300;
		
		legangles[i][0]*=1024;
		legangles[i][1]*=1024;
		legangles[i][2]*=1024;*/
		
		//and print out the computed angles just for fun :)
		//cerr << "angles(abs) for leg: " << i+1 << ": " << legangles[i][0] << ", " << legangles[i][1] << ", " << legangles[i][2] << endl;
		
	}
	//time to move the legs to the requested angles, and watch it all go horribly horribly wrong woo!
	int ang[18];
	ang[0]=(int)legangles[0][0];
	ang[1]=(int)legangles[1][0];
	ang[2]=(int)legangles[0][1];
	ang[3]=(int)legangles[1][1];
	ang[4]=(int)legangles[0][2];
	ang[5]=(int)legangles[1][2];
	
	ang[6]=(int)legangles[2][0];
	ang[7]=(int)legangles[3][0];
	ang[8]=(int)legangles[2][1];
	ang[9]=(int)legangles[3][1];
	ang[10]=(int)legangles[2][2];
	ang[11]=(int)legangles[3][2];
	
	ang[12]=(int)legangles[4][0];
	ang[13]=(int)legangles[5][0];
	ang[14]=(int)legangles[4][1];
	ang[15]=(int)legangles[5][1];
	ang[16]=(int)legangles[4][2];
	ang[17]=(int)legangles[5][2];
	
	phex_pose pose(ang);
	pose.movesync();
	
	
return 0;
}



int main()
{
	int baudnum = 1;
	int GoalPos[2] = {250, 750};
	//int GoalPos[2] = {0, 4095}; // for Ex series
	int index = 0;
	int deviceIndex = 0;
	int PresentPos;
	int CommStatus;
	int i=0;
	
	printf( "\n\nRead/Write example for Linux\n\n" );
	///////// Open USB2Dynamixel ////////////
	if( dxl_initialize(deviceIndex, baudnum) == 0 )
	{
		printf( "Failed to open USB2Dynamixel!\n" );
		printf( "Press Enter key to terminate...\n" );
		getchar();
		return 0;
	}
	else
		printf( "Succeed to open USB2Dynamixel!\n" );
		
		
	setup_walk();
	char c=' ';
	
	ifstream pose_file("poses/default.poses");
	string line;
	getline(pose_file, line, ']');
		
	phex_pose test(line);
	test.move();
	pose_file.close();
	
	
	
	pose_file.open("poses/walk_forward_3_3.poses");
	while(!pose_file.eof())
	{
		getline(pose_file, line, ']');
		#ifdef EBUG
			cerr << "INFO: read line from walk file: " << line << endl;
		#endif
		if(line.length()>0)
			walk_forward_vector.push_back(new phex_pose(line));
	}
	pose_file.close();
	
	while(c!='/')
	{
	cout << "   Select function: " << endl
		 << "1. Read Positions" << endl
		 << "2. Set Individual Servo" << endl
		 << "3. Inverse Kinematics test" << endl
		 << "4. walk (move forward poses)" << endl
		 << "5. Inverse Kinematics test (continuous)" << endl
		 << "6. Joystick Inverse Kinematics test" << endl;
		
	c=getchar();
		if(c=='1')
		{
			for(i=1;i<=18;i++)
			{
				fprintf(stderr, "servo %i position is %i\n", i, dxl_read_word( i, P_PRESENT_POSITION_L ));
			}
		}
		else if(c=='2')
		{
			int servo=0;
			int degrees=0;
			int position=0;
			
			while(1)
			{
				
				fprintf(stdout, "enter servo number\n");
				fscanf(stdin, "%i", &servo);
				
				fprintf(stdout, "enter servo degrees\n");
				fscanf(stdin, "%i", &degrees);
				
				
				position=get_position(degrees);
				dxl_write_word( BROADCAST_ID, P_GOAL_SPEED_L, 0 );	
				dxl_write_word( servo, P_GOAL_POSITION_L, position);
				
				moving=1;
				while(moving)
					moving=dxl_read_byte( servo, P_MOVING );
				
			}
		}
		else if(c=='4')
		{
			//do_walk();
			step_forward();
			
		}
		else if(c=='3')
		{
			fprintf(stderr, "ik selected \n");
			float x, y, z;
			while(1)
			{
				fprintf(stderr, "input x\n");
				fscanf(stdin, "%f", &x);
				
				fprintf(stderr, "input y\n");
				fscanf(stdin, "%f", &y);
				
				fprintf(stderr, "input z\n");
				fscanf(stdin, "%f", &z);
				
				inverse_kinematics_test(x, y, z);
				
			}
			
		}
		else if(c=='5')
		{
			float x=0,y=0,z=0;
			float index=0.0;
			bool direc=false; //true=up, false=down;
			while(1)
			{
				if(direc)
				{
					x+=5;
					y+=5;
					z+=5;
				}
				else
				{
					x-=5;
					y-=5;
					z-=5;
				}
				if(x<-40)direc=true;
				if(x>40)direc=false;
				
				
				cerr << "P for test: " << x << "," << y << "," << z << endl;
				inverse_kinematics_test(x,y,z);
				
			}
		}
		else if(c=='6')
		{
			
		}
	}

	// Close device
	dxl_terminate();
	printf( "Press Enter key to terminate...\n" );
	getchar();
	return 0;
}


int get_position(int k)
{
	int position=0;
	position=((float)(((float)k/300)*1024));
	if(position<0)
		position=0;
	else if(position>1024)
		position=1024;
	fprintf(stderr, "position=%i\n", position);
	return position;
}

int setup_walk()
{
	/* pose 1 */
	{
	walk[0].servos[0][0]=600;
	walk[0].servos[1][0]=400;
	walk[0].servos[2][0]=500;
	walk[0].servos[3][0]=500;
	walk[0].servos[4][0]=400;
	walk[0].servos[5][0]=600;
	
	walk[0].servos[0][1]=450;
	walk[0].servos[1][1]=450;
	walk[0].servos[2][1]=450;
	walk[0].servos[3][1]=450;
	walk[0].servos[4][1]=450;
	walk[0].servos[5][1]=450;
	
	walk[0].servos[0][2]=200;
	walk[0].servos[1][2]=200;
	walk[0].servos[2][2]=200;
	walk[0].servos[3][2]=200;
	walk[0].servos[4][2]=200;
	walk[0].servos[5][2]=200;
}	
	
	/* pose 2 */
	{
	walk[1].servos[0][0]=600;
	walk[1].servos[1][0]=400;
	walk[1].servos[2][0]=500;
	walk[1].servos[3][0]=500;
	walk[1].servos[4][0]=400;
	walk[1].servos[5][0]=600;
	
	walk[1].servos[0][1]=400;
	walk[1].servos[1][1]=450;
	walk[1].servos[2][1]=450;
	walk[1].servos[3][1]=400;
	walk[1].servos[4][1]=400;
	walk[1].servos[5][1]=450;
	
	walk[1].servos[0][2]=200;
	walk[1].servos[1][2]=200;
	walk[1].servos[2][2]=200;
	walk[1].servos[3][2]=200;
	walk[1].servos[4][2]=200;
	walk[1].servos[5][2]=200;
	}	
	
		/* pose 3 */
	{
	walk[2].servos[0][0]=700;
	walk[2].servos[1][0]=400;
	walk[2].servos[2][0]=500;
	walk[2].servos[3][0]=400;
	walk[2].servos[4][0]=500;
	walk[2].servos[5][0]=600;
	
	walk[2].servos[0][1]=450;
	walk[2].servos[1][1]=450;
	walk[2].servos[2][1]=450;
	walk[2].servos[3][1]=450;
	walk[2].servos[4][1]=450;
	walk[2].servos[5][1]=450;
	
	walk[2].servos[0][2]=200;
	walk[2].servos[1][2]=200;
	walk[2].servos[2][2]=200;
	walk[2].servos[3][2]=200;
	walk[2].servos[4][2]=200;
	walk[2].servos[5][2]=200;
}	

	/* pose 4 */
	{
	walk[3].servos[0][0]=700;
	walk[3].servos[1][0]=400;
	walk[3].servos[2][0]=500;
	walk[3].servos[3][0]=400;
	walk[3].servos[4][0]=500;
	walk[3].servos[5][0]=600;

	walk[3].servos[0][1]=450;
	walk[3].servos[1][1]=400;
	walk[3].servos[2][1]=400;
	walk[3].servos[3][1]=450;
	walk[3].servos[4][1]=450;
	walk[3].servos[5][1]=400;
	
	walk[3].servos[0][2]=200;
	walk[3].servos[1][2]=200;
	walk[3].servos[2][2]=200;
	walk[3].servos[3][2]=200;
	walk[3].servos[4][2]=200;
	walk[3].servos[5][2]=200;
}	

	/* pose 5 */
	{
	walk[4].servos[0][0]=600;
	walk[4].servos[1][0]=400;
	walk[4].servos[2][0]=500;
	walk[4].servos[3][0]=500;
	walk[4].servos[4][0]=400;
	walk[4].servos[5][0]=600;
	
	walk[4].servos[0][1]=450;
	walk[4].servos[1][1]=450;
	walk[4].servos[2][1]=450;
	walk[4].servos[3][1]=450;
	walk[4].servos[4][1]=450;
	walk[4].servos[5][1]=450;
	
	walk[4].servos[0][2]=200;
	walk[4].servos[1][2]=200;
	walk[4].servos[2][2]=200;
	walk[4].servos[3][2]=200;
	walk[4].servos[4][2]=200;
	walk[4].servos[5][2]=200;
}	

	/* pose 6 */
	{
	walk[5].servos[0][0]=600;
	walk[5].servos[1][0]=400;
	walk[5].servos[2][0]=500;
	walk[5].servos[3][0]=500;
	walk[5].servos[4][0]=400;
	walk[5].servos[5][0]=600;
	
	walk[5].servos[0][1]=450;
	walk[5].servos[1][1]=450;
	walk[5].servos[2][1]=450;
	walk[5].servos[3][1]=450;
	walk[5].servos[4][1]=450;
	walk[5].servos[5][1]=450;
	
	walk[5].servos[0][2]=200;
	walk[5].servos[1][2]=200;
	walk[5].servos[2][2]=200;
	walk[5].servos[3][2]=200;
	walk[5].servos[4][2]=200;
	walk[5].servos[5][2]=200;
}	
	return 0;
}

void do_walk()
{
	int i=0;
	for(i=0;i<WALK_STEPS;i++)
		move_pose(&walk[i]);
	
}


int move_pose(struct pose* pose)
{
	
	dxl_write_word( 1, P_GOAL_POSITION_L, pose->servos[0][0] );
	dxl_write_word( 2, P_GOAL_POSITION_L, pose->servos[1][0] );
	dxl_write_word( 7, P_GOAL_POSITION_L, pose->servos[2][0] );
	dxl_write_word( 8, P_GOAL_POSITION_L, pose->servos[3][0] );
	dxl_write_word( 13, P_GOAL_POSITION_L, pose->servos[4][0] );
	dxl_write_word( 14, P_GOAL_POSITION_L, pose->servos[5][0] );
	
	dxl_write_word( 3, P_GOAL_POSITION_L, pose->servos[0][1] );
	dxl_write_word( 4, P_GOAL_POSITION_L, pose->servos[1][1] );
	dxl_write_word( 9, P_GOAL_POSITION_L, pose->servos[2][1] );
	dxl_write_word( 10, P_GOAL_POSITION_L, pose->servos[3][1] );
	dxl_write_word( 15, P_GOAL_POSITION_L, pose->servos[4][1] );
	dxl_write_word( 16, P_GOAL_POSITION_L, pose->servos[5][1] );	
	
	dxl_write_word( 5, P_GOAL_POSITION_L, pose->servos[0][2] );
	dxl_write_word( 6, P_GOAL_POSITION_L, pose->servos[1][2] );
	dxl_write_word( 11, P_GOAL_POSITION_L, pose->servos[2][2] );
	dxl_write_word( 12, P_GOAL_POSITION_L, pose->servos[3][2] );
	dxl_write_word( 17, P_GOAL_POSITION_L, pose->servos[4][2] );
	dxl_write_word( 18, P_GOAL_POSITION_L, pose->servos[5][2] );	
	moving=1;
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
		dxl_read_byte( 10, P_MOVING) ||
		dxl_read_byte( 11, P_MOVING) ||
		dxl_read_byte( 12, P_MOVING) ||
		dxl_read_byte( 13, P_MOVING) ||
		dxl_read_byte( 14, P_MOVING) ||
		dxl_read_byte( 15, P_MOVING) ||
		dxl_read_byte( 16, P_MOVING) ||
		dxl_read_byte( 17, P_MOVING) ||
		dxl_read_byte( 18, P_MOVING)
		);		
	}
	return 0;
}
