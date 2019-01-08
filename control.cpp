#include <fstream>
#include <sstream>
#include <iostream>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/dji_sdk.h>
#include "iarc/pilot/control.h"
#include "iarc/obstacle.h"
#include "iarc/vehicle_pos.h"
#include "iarc/pilot/subscribe.h"
#include "iarc/pilot/pid_controller.h"
#include <ros/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <iarc/pilot/XU_LADRC.h>
using namespace std;
using namespace DJI::onboardSDK;


/****
This file should include two function 
1. deciding whether to interact or not
2. to generate and send a serial of command [roll,pitch,yaw',z']
*****/


// Flags starts here
bool patrol_random=1;           
// 1 denotes random mode which rely on no positioning imformation
// 0 denotes squaring mode which rely on the positioning imformation

bool attitude_control=false;    // 1 denotes attitude control
                                // 0 denotes velocity control

bool pid_flag=true;             // 1 denotes pid control
                                // 0 denotes LQR control


bool velocity_track=1;          
//  1 denotes using veocity control to track (AKA h >1.0m)
//  0 denotes using veocity control to track (AKA h <1.0m)
//  A schmitt trigger should be designed here, the range is set as 0.9-1.0

// Flags ends here

/*****
attitude test 
*********/

float x = 1.0;
float y = 0.;



extern CvMat *att_init;
extern CvMat *R_init;
extern CvMat *R_init_i;
extern CvMat *q_right_now;
extern CvMat *att_right_now;
CvMat *q=cvCreateMat(4,1,CV_32FC1);
CvMat *atti=cvCreateMat(3,1,CV_32FC1);
CvMat *R_now_i=cvCreateMat(3,3,CV_32FC1);

extern iarc::obstacle store;
extern int control_mode;
extern int control_mode_previous;
int control_mode;
int control_mode_previous=-100;

_attitude_command attitude_command;
_attitude_command attitude_arena;
_attitude_command attitude_ground;

_velocity_command velocity_command;
_velocity_command velocity_arena;
_velocity_command velocity_ground;
_patrol patrolr={0,0};

unsigned int period = 0;


/***
test flags
***/
bool fisrt_time=1;
float tar_x=0.,tar_y=0.;


/*******
test flag ends
*******/

int decision(int distance)
{
    cout<<distance<<endl;
    return 0;
}
int patrol_stage=0;
double patrol_time=0.;
iarc::vehicle_pos position;


pid_ctrl pid_x,pid_y,pid_z,pid_yaw,pid_pitch,pid_roll;
//pid_ctrl




void wny_45()
{
    return;
}

void stop()
{
    velocity_command.vx = 0.;
    velocity_command.vy = 0.;
    velocity_command.vh = 0.;
    velocity_command.dyaw =0.;
    attitude_control = false;
    return;
}

int patrol(dji_sdk::Velocity velocity,dji_sdk::AttitudeQuaternion 
          attitude_quaternion,iarc::vehicle_pos pose,float delta_h/**position*/)
{
    dji_sdk::Velocity velo = velocity;
    cvmSet(q, 0, 0, attitude_quaternion.q0);
	cvmSet(q, 1, 0, attitude_quaternion.q1);
	cvmSet(q, 2, 0, attitude_quaternion.q2);
	cvmSet(q, 3, 0, attitude_quaternion.q3);
    Quaternion_To_Euler(q,atti);

    int re=0;
    switch(patrol_stage)
    {
        case 0:
        {
            patrol_time=tic();
            patrol_stage=1;
            break;
        }
        case 1:
        {
            if(tic()-patrol_time>=2.)
            {
                patrol_stage = 2;
            }
            if(cvmGet(atti,2,0)>=2.82&&cvmGet(atti,2,0)<=2.80)
            {
                velocity_command.vx = 0.5;
                velocity_command.vy = 0.;
                velocity_command.vh = 0.2*delta_h;
                velocity_command.dyaw =0.;
                attitude_control = false;
                //if(velo.vx)
            }
            else
            {
                velocity_command.vx = 0.;
                velocity_command.vy = 0.;
                velocity_command.vh = 0.2*delta_h;
                velocity_command.dyaw =(2.82-cvmGet(atti,2,0))*0.4;
                attitude_control = false;
            }

            break;
        }
        case 2:
        {
            re=1;
            break;
        }
    }
    return re;
}

void patrol_r(bool side_one, bool side_two, 
              float angle_wall, float angle_quad,float delta_h)
{
	//Integral_reset();

	float angle_error=fabs(angle_wall-angle_quad);
	if(angle_error<1.57)
	{
		angle_error=3.14-angle_error;
	}
	else
	{
	}
	
	if(side_one==1&&side_two==0)	//side_one before side_two
	{
		patrolr.i++;
		if(patrolr.i>10)
		{
			patrolr.i=0;
			patrolr.angle=patrolr.angle+1.57+0.5*angle_error;
			printf("one wall!\n");
		}
		else
		{
			printf("patrolling!\n");
		}
	}
	else if(side_one==1&&side_two==1)
	{
		patrolr.i++;
		if(patrolr.i>10)
		{
			patrolr.i=0;
			patrolr.angle=patrolr.angle+3.14;
			printf("two wall!\n");
		}
		else
		{
			printf("patrolling!\n");
		}
	}
	else
	{
		printf("patrolling!\n");
	}

    velocity_command.vh     = 0.4*delta_h;
    velocity_command.vx     = 0.25*cos(patrolr.angle);
    velocity_command.vy     = 0.25*sin(patrolr.angle);
    velocity_command.dyaw   = 0.;
    attitude_control = false;
    // TODO: yaw.kp_vx=0.;
}


void ascend( float current_height, int ascend_mode,float direction=-1000)
{
    //This fucntion is used to control the drone to a specific height
    // define ascend_mode   1: back to patrol height 
    //                      2: Ascend till the target is in sight
    //                      3: Avoid the obstacle, emergent
    //                      *4: Avoid the obstacle, normal ( not to consider)
    
    switch(ascend_mode)
    {
        case 1:
        {
            if(1.3-current_height>=0.5)
                velocity_command.vh = 0.3*(1.3-current_height);
            else
                velocity_command.vh = 0.4*(1.3-current_height);
            velocity_command.vx     = 0.;
            velocity_command.vy     = 0.;
            velocity_command.dyaw   = 0.;
            attitude_control = false;
            break;
        }

        case 2:
        {
            velocity_command.vh = 0.1*(1.3-current_height);
            if(direction>-1000)
            {
                velocity_command.vx     = 0.;
                velocity_command.vy     = 0.;
            }
            else
            {
                velocity_command.vx     = 0.33*cos(direction);
                velocity_command.vy     = 0.33*sin(direction);
            }
            velocity_command.dyaw   = 0.;
            attitude_control = false;
            break;
        }

        case 3:
        {
            if(1.3-current_height>=0.5)
                velocity_command.vh = 0.3*(1.3-current_height);
            else
                velocity_command.vh = 0.4*(1.3-current_height);
            if(direction>-1000)
            {
                velocity_command.vx     = 0.;
                velocity_command.vy     = 0.;
            }
            else
            {
                velocity_command.vx     = -0.33*cos(direction);
                velocity_command.vy     = -0.33*sin(direction);
            }
            velocity_command.dyaw   = 0.;
            attitude_control = false;
            break;
        }
        default:
        {
            cout << "error"<< endl;
            break;
        }

    }
    cout << current_height<<"\t"<<velocity_command.vh <<endl;
}

void track(float x,float y,float h,float h_ideal,bool is_direction,float direction,bool tar)
{
    
    if(x==-10 && y==-10)
    {
        velocity_command.dyaw   = 0.;
        velocity_command.vx     = 0.;
        velocity_command.vy     = 0.;
        velocity_command.vh     = 0.;
        attitude_control        = false;
        return;
    }


    if(!velocity_track)//ToDo - output attitude command
    {
    //////////////////////////////////////////////////////////////////////////
        attitude_control=true;

        attitude_command.dyaw   = 0.;
        velocity_arena.vh       = pid_z.out(h_ideal,h,0);
        velocity_arena.vx       = pid_x.out(0,x,1);
        velocity_arena.vy       = pid_y.out(0,y,1);

        /*
        if(period%2==0)
        {
             attitude_arena.pitch =   pid_pitch.out(velocity_arena.vx,tempvx,0);
             attitude_arena.roll  =   pid_roll.out(velocity_arena.vy,tempvy,0);           
        }*/


        float tempvx = 0.; 
//v_guidance.vector.x*cvmGet(R_now,0,0)+v_guidance.vector.y*cvmGet(R_now,0,1);
		
		float tempvy = 0.; 
//v_guidance.vector.x*cvmGet(R_now,1,0)+v_guidance.vector.y*cvmGet(R_now,1,1);

        attitude_command.pitch =   pid_pitch.out(velocity_arena.vx,tempvx,0);
        attitude_command.roll  =   pid_roll.out(velocity_arena.vy,tempvy,0); 
       /* attitude_control=true;
        attitude_command.dyaw=0.;
        attitude_arena.pitch    = pid_x.out(0,x,1);
        attitude_arena.roll     = pid_y.out(0,y,1);
        attitude_arena.vh       = pid_z.out(h_ideal,h,1);
        attitude_command.pitch  = 
		-1*(attitude_arena.pitch*cvmGet(R_init,0,0) + 
		    attitude_arena.roll*cvmGet(R_init,0,1));
        attitude_command.roll   = 
		(attitude_arena.pitch*cvmGet(R_init,1,0) + 
		 attitude_arena.roll*cvmGet(R_init,1,1));
        attitude_command.vh     = attitude_arena.vh;*/
    }
    else//output velocity command
    {
        
        velocity_arena.dyaw   = 0.;
        velocity_arena.vx       = pid_x.out(0,x,1);
        velocity_arena.vy       = pid_y.out(0,y,1);
        velocity_arena.vh       = pid_z.out(h_ideal,h,0);
        attitude_control = false;
    }

}

void test_att(float dx, float dy,float h,float h_ideal)
{
        
        attitude_control=true;
        attitude_command.roll=0.;
        attitude_command.pitch=0.;
        attitude_command.vh=pid_z.out(h_ideal,h,0);
        attitude_command.dyaw=0.;
}

void control()
{
    time_t nowtime;
    nowtime = time(NULL);
    ostringstream oss;
    tm *ltm = localtime(&nowtime);
    oss << ltm->tm_mon+1 << "Month" << ltm->tm_mday << "Day" 
	    << ltm->tm_hour << "Time" << ltm->tm_min;
    string path = "/home/hitcsc/catkin_ws/log/iarc/pilot/" + 
	              oss.str();//string(ctime(&nowtime));
    string file = path + "/control.txt";
    string temp_file = path + "/height.txt";
    char* c = new char[path.length()+1];
    strcpy(c,path.c_str());
    if(mkdir(c,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
    {
        cout << "Mkdir success"<< endl;
    }
    else
    {
        cout << "Mkdir error"<< endl;
    }
    delete[] c;
    ofstream control_log(file);
    ofstream height_log(temp_file);
    ros::NodeHandle pilot;
    DJIDrone* drone = new DJIDrone(pilot);
    LEODrone* wsq_drone=new LEODrone(pilot);
    ///string param="/home/hitcsc/catkin_ws/src/iarc/cfg/pidconfig.txt";
    float time0 = tic();
    pid_x.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidx.txt");
    pid_y.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidy.txt");
    pid_z.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidz.txt");
    pid_pitch.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidpitch.txt");
    pid_roll.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidroll.txt");    
    pid_yaw.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidyaw.txt");
    control_mode = -1;
    // cout << "1"<<endl;
    /*
    Five modes in total : 
    1: 45   degree interaction
    2: 180  degree interaction
    3: Tracking
    4: Patrol
    5: Ascend 

    IF PID 
    1.1:
    1.2:
    1.3:

    IF LQR
    1:    
    */

    ros::Rate rate(50);
    while(ros::ok())
    {
        //ROS_INFO("Here comes the control part");

        //1.If controller is not in F mode, release contorl and exit the control program.
        ros::spinOnce();
        while((abs(drone->rc_channels.mode+8000.0)<1)||abs(drone->rc_channels.mode)<1)
        {
            ros::spinOnce();
            drone-> release_sdk_permission_control();
            ROS_INFO("Release control.");
            control_log.close();
            height_log.close();
            usleep(5000);

            return;
        }
        
        if(abs(drone->rc_channels.mode-8000.0)<1)
        {
		    drone->request_sdk_permission_control();
	    }
        /*This is the main control function*/

        //2. Decide the situation, modify the system mode. 4 modes in total.
        
        control_mode=3;
        period++;
        if(period==100)
        {
            period=0;
        }
        if(control_mode!=control_mode_previous)
        {
            pid_x.clean();
            pid_y.clean();
            pid_z.clean();
            pid_yaw.clean();
            pid_pitch.clean();
            pid_roll.clean();
cout<< "Clean the integral and history data of pid when the mode is changed"
    <<endl;
        }

        ///// information initialization begins 
        cvmSet(q_right_now, 0, 0, wsq_drone->attitude_quaternion.q0);
        cvmSet(q_right_now, 1, 0, wsq_drone->attitude_quaternion.q1);
        cvmSet(q_right_now, 2, 0, wsq_drone->attitude_quaternion.q2);
        cvmSet(q_right_now, 3, 0, wsq_drone->attitude_quaternion.q3);
        Quaternion_To_Euler(q_right_now,att_right_now);
        Euler_To_Matrix(-1*cvmGet(att_right_now,0,0),
		                -1*cvmGet(att_right_now,1,0),
						-1*cvmGet(att_right_now,2,0), R_now_i);
        float yaw_now=(-cvmGet(att_init,2,0)+cvmGet(att_right_now,2,0));
        if(yaw_now>PI)
		{
			yaw_now=yaw_now-2*PI;
		}
		else if(yaw_now<-PI)
		{
			yaw_now=yaw_now+2*PI;
		}

        int target_number = wsq_drone->object_pos_dynamic.target_num;
        wsq_drone->object_pos_dynamic.target_x.resize(target_number);
        wsq_drone->object_pos_dynamic.target_y.resize(target_number);
        wsq_drone->object_pos_dynamic.target_vx.resize(target_number);
        wsq_drone->object_pos_dynamic.target_vy.resize(target_number);
        

        // reserved for obstacles 


        // information initialization ends 

        // 3.Realization of each mode. Each mode will be divided into 
		// several occations accoding to the situation. 
        // In each occation, the danger of obstacles should be considered.
        // In these occations, a velocity/attitude contorl command should be 
		// generated, and the value of 'attitude_control' should be changed.

        switch (control_mode)
        {
            case 1:     //45 degree
            {
                if(pid_flag)
                {
                    wny_45();
                }
                else
                {

                    attitude_control=true;
                    attitude_command.roll=0.;
                    attitude_command.pitch=0.;
                    attitude_command.vh=0.;
                    attitude_command.dyaw=0.;
                }
                break;
            }
            case 2:     // 180 degree 
            {
                if(pid_flag)
                {
                    ;
                }
                else
                {
                    
                    attitude_control=true;
                    attitude_command.roll=0.;
                    attitude_command.pitch=0.;
                    attitude_command.vh=0.;
                    attitude_command.dyaw=0.;
                }
                break;
            }
            case 3:     // track 
            {              
                float height_test=0.0;
                if(drone->rc_channels.gear==-4545.0)
                {
                    height_test=0.25;
                }
                else if(drone->rc_channels.gear==-10000.0)
                {
                    height_test=1.1;
                }

                if(wsq_drone->object_pos_dynamic.target_num==1)
                {
                    fisrt_time = 1;
                    float dx = 0.01*wsq_drone->object_pos_dynamic.target_x[0];
                    float dy = 0.01*wsq_drone->object_pos_dynamic.target_y[0];
                    control_log<<tic()-time0 <<"\t"<< target_number
					           <<"\t"<<dx<<"\t"<<dy<<"\t";
                    //track(dx,dy,wsq_drone->local_position.z,height_test,0,0.,1);
                    track(0.,0.,wsq_drone->local_position.z,height_test,0,0.,1);
                    height_log<< wsq_drone->object_pos_dynamic.target_vx[0]
					<< "\t" << wsq_drone->object_pos_dynamic.target_vy[0]<<"\t";

                }
                else
                {
                    //stop();
                    track(0.,0.,wsq_drone->local_position.z,height_test,0,0.,1);
                    height_log<< 0.<< "\t" << 0.<<"\t";
                }
                control_log<<tic()-time0 <<"\t";
                
                attitude_control = 0; 
                break;
            }

            case 4:     // patrol
            {
                if(patrol_random)
                {
                    patrol_r(1,1,0.5,0.5,2.15-wsq_drone->local_position.z);
                }
                else
                {

                    int bb=patrol(wsq_drone->velocity,
					              wsq_drone->attitude_quaternion,position,2.14);
                    if(bb==1)
                    {
                        return;
                    }
                }                
                break;
            }

            case 5:     // ascend
            {
                ascend(wsq_drone->local_position.z,1);
                break;
            }
        }
        // cout << "4"<<endl;
        //4. Send the control command to M100.
        if(attitude_control)
        {
            // TODO

            // test only
            //attitude_arena.pitch   = 0.0;
            //attitude_arena.roll    = 1.0;
            //attitude_arena.vh      = 0.;
            //attitude_arena.dyaw    = 0.;
            // test only

            attitude_ground.pitch  = -1*(attitude_arena.pitch*cvmGet(R_init,0,0)+
			                             attitude_arena.roll*cvmGet(R_init,0,1));
            attitude_ground.roll   = -1*(attitude_arena.pitch*cvmGet(R_init,1,0)+
			                             attitude_arena.roll*cvmGet(R_init,1,1));

            attitude_command.pitch  = 0.0+attitude_ground.pitch*cvmGet(R_now_i,0,0)+
			                          attitude_ground.roll*cvmGet(R_now_i,0,1);
            attitude_command.roll   = -0.7+attitude_ground.pitch*cvmGet(R_now_i,1,0)+
			                          attitude_ground.roll*cvmGet(R_now_i,1,1);
            attitude_command.dyaw   = attitude_arena.dyaw;
            attitude_command.vh     = attitude_arena.vh;
            drone->attitude_control(    Flight::HorizontalLogic::HORIZONTAL_ANGLE|
									    Flight::VerticalLogic::VERTICAL_VELOCITY |
									    Flight::YawLogic::YAW_PALSTANCE |
									    Flight::HorizontalCoordinate::HORIZONTAL_BODY |	
									    Flight::SmoothMode::SMOOTH_DISABLE,
									    attitude_command.roll, 
										-1*attitude_command.pitch, 
										attitude_command.vh, 
										attitude_command.dyaw);
           // cout <<"att"<<endl;
        }
        else
        {
            velocity_command.vx       = -1*(velocity_arena.vx*cvmGet(R_init,0,0)
			                            +velocity_arena.vy*cvmGet(R_init,0,1)
										+velocity_arena.vh*cvmGet(R_init,0,2));
            velocity_command.vy       = -1*(velocity_arena.vx*cvmGet(R_init,1,0)
			                            +velocity_arena.vy*cvmGet(R_init,1,1)
										+velocity_arena.vh*cvmGet(R_init,1,2));
            velocity_command.vh       = velocity_arena.vx*cvmGet(R_init,2,0)
			                            +velocity_arena.vy*cvmGet(R_init,2,1)
										+velocity_arena.vh*cvmGet(R_init,2,2);
            velocity_command.dyaw     = velocity_arena.dyaw;

            drone->attitude_control(    Flight::HorizontalLogic::HORIZONTAL_VELOCITY|
									    Flight::VerticalLogic::VERTICAL_VELOCITY |
									    Flight::YawLogic::YAW_PALSTANCE |
									    Flight::HorizontalCoordinate::HORIZONTAL_GROUND |	
									    Flight::SmoothMode::SMOOTH_DISABLE,
									    velocity_command.vx, velocity_command.vy, 
										velocity_command.vh, velocity_command.dyaw);
        }
        //5. 
        // cout << "5"<<endl;
        if(attitude_control)
        {
            control_log<<attitude_command.roll <<"\t"<< attitude_command.pitch  
			           <<"\t"<<attitude_command.vh <<"\t"<<attitude_command.dyaw
			           <<"\t"<<-cvmGet(att_init,0,0)+cvmGet(att_right_now,0,0)
					   <<"\t"<<-cvmGet(att_init,1,0)+cvmGet(att_right_now,1,0)
					   <<"\t"<<-cvmGet(att_init,2,0)+cvmGet(att_right_now,2,0)
					   <<endl;
            cout << "attitude"<<attitude_command.roll  <<"\t"
			     << attitude_command.pitch <<"\t"<<attitude_command.vh 
				 <<"\t"<<attitude_command.dyaw<<endl;
        }
        else
        {
            control_log<<velocity_arena.vx <<"\t"<<velocity_arena.vy <<"\t"
			           <<velocity_arena.vh <<"\t"<<velocity_command.dyaw<<"\t"
					   <<wsq_drone->local_position.z<<"\t"<<"1.3"<<"\t"<<"0.25"
					   <<"\t"<<endl;
            cout <<"velocity"<<velocity_arena.vx <<"\t"<<velocity_arena.vy 
			     <<"\t"<<velocity_arena.vh <<"\t"<<velocity_command.dyaw<<"\t"
				 <<wsq_drone->local_position.z<<endl;
        }
        // cout << "36"<<endl;
        wsq_drone->ultrasonic.ranges.resize(2);
        height_log << tic()-time0 <<"\t"<< wsq_drone->local_position.z
		           <<"\t"<<wsq_drone->ultrasonic.ranges[1]<< endl;

        rate.sleep();
        control_mode_previous = control_mode;
    }

}