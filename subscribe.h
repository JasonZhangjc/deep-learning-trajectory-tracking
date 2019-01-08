#include <dji_sdk/dji_sdk.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <string>
#include <geometry_msgs/TransformStamped.h>   //IMU
#include <geometry_msgs/Vector3Stamped.h>     //velocity
#include <sensor_msgs/LaserScan.h>            //obstacle distance && ultrasonic
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iarc/model_choose.h>
#include "iarc/pilot/leo_math.h"
#include <dji_sdk/LocalPosition.h>
#include <iarc/object.h>

extern CvMat *R_init;


class LEODrone
{
private:
	ros::Subscriber ultrasonic_subscriber;
	ros::Subscriber acceleration_subscriber;
	ros::Subscriber attitude_quaternion_subscriber;
    ros::Subscriber rc_channels_subscriber;
    ros::Subscriber velocity_subscriber;
    ros::Subscriber activation_subscriber;
	ros::Subscriber sdk_permission_subscriber;
	ros::Subscriber time_stamp_subscriber;
	ros::Subscriber velocity_guidance_subscriber;
	ros::Subscriber l_position;

	ros::Subscriber object_subscriber;

public:
	ros::Publisher model_choose_publisher;
	sensor_msgs::LaserScan ultrasonic;
	dji_sdk::Acceleration acceleration;
	dji_sdk::AttitudeQuaternion attitude_quaternion;
	dji_sdk::RCChannels rc_channels;
	dji_sdk::Velocity velocity;
    dji_sdk::TimeStamp time_stamp;
	geometry_msgs::Vector3Stamped velocity_guidance;	
	// this velocity is in arena coordinate
	dji_sdk::LocalPosition local_position;
	iarc::model_choose state_choose;
    bool sdk_permission_opened=true;
    bool activation = false;

	iarc::object object_pos_dynamic;
	iarc::object object_pos_history;
	iarc::object object_prediction;

private:
	    void ultrasonic_subscriber_callback(sensor_msgs::LaserScan g_ul);

	    void acceleration_subscriber_callback(dji_sdk::Acceleration acceleration);

		void attitude_quaternion_subscriber_callback(dji_sdk::AttitudeQuaternion attitude_quaternion);

		void rc_channels_subscriber_callback(dji_sdk::RCChannels rc_channels);

		void velocity_subscriber_callback(dji_sdk::Velocity velocity);

		void activation_subscriber_callback(std_msgs::UInt8 activation);

		void sdk_permission_subscriber_callback(std_msgs::UInt8 sdk_permission);

		void time_stamp_subscriber_callback(dji_sdk::TimeStamp time_stamp);

		void velocity_guidance_subscriber_callback(geometry_msgs::Vector3Stamped g_vo);

		void l_pose_callback(dji_sdk::LocalPosition posi);
		
		void object_subscriber_callback(iarc::object object_pos);

		void kalman_filter(iarc::object object_pos);
	
public:
	LEODrone(ros::NodeHandle& nh)
	{
			    ultrasonic_subscriber = 
				nh.subscribe<sensor_msgs::LaserScan>("/guidance/ultrasonic", 
				1, &LEODrone::ultrasonic_subscriber_callback, this);
			    
				velocity_guidance_subscriber = 
				nh.subscribe<geometry_msgs::Vector3Stamped>("/guidance/velocity",
				1, &LEODrone::velocity_guidance_subscriber_callback,this);

		 	 	acceleration_subscriber = 
				nh.subscribe<dji_sdk::Acceleration>("dji_sdk/acceleration", 
				10, &LEODrone::acceleration_subscriber_callback, this);
		        
				attitude_quaternion_subscriber = 
				nh.subscribe<dji_sdk::AttitudeQuaternion>("dji_sdk/attitude_quaternion", 
				10, &LEODrone::attitude_quaternion_subscriber_callback, this);
		        
				rc_channels_subscriber = 
				nh.subscribe<dji_sdk::RCChannels>("dji_sdk/rc_channels", 
				10, &LEODrone::rc_channels_subscriber_callback, this);
		        
				velocity_subscriber = 
				nh.subscribe<dji_sdk::Velocity>("dji_sdk/velocity", 
				10, &LEODrone::velocity_subscriber_callback, this);
		        
				activation_subscriber = 
				nh.subscribe<std_msgs::UInt8>("dji_sdk/activation", 
				10, &LEODrone::activation_subscriber_callback, this);
		        
				sdk_permission_subscriber = 
				nh.subscribe<std_msgs::UInt8>("dji_sdk/sdk_permission", 
				10, &LEODrone::sdk_permission_subscriber_callback, this);
				
				time_stamp_subscriber = 
				nh.subscribe<dji_sdk::TimeStamp>("dji_sdk/time_stamp", 
				10, &LEODrone::time_stamp_subscriber_callback,this);
				
				l_position = 
				nh.subscribe("/dji_sdk/local_position",
				10,&LEODrone::l_pose_callback,this);

				object_subscriber = 
				nh.subscribe<iarc::object>("/hy_object", 
				1, &LEODrone::object_subscriber_callback,this);			    
	}
};
//