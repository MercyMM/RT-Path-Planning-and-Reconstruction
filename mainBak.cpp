#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include "DJI_guidance.h"
#include "DJI_utility.h"

#include "LinuxSerialDevice.h"
#include "LinuxThread.h"
#include "LinuxSetup.h"
#include "ReadUserConfig.h"


#include <DJI_Flight.h>
#include <DJI_Version.h>

using namespace std;

#ifdef HAVE_OPENCV
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#endif


int WIDTH=320;
int HEIGHT=240;
#define IMAGE_SIZE (HEIGHT * WIDTH)

#define USE_GUIDANCE_ASSISTANT_CONFIG 0 //use GUIDANCE ASSISTANT's configure
#define SELECT_DEPTH_DATA 1

#ifdef HAVE_OPENCV
using namespace cv;
Mat     g_greyscale_image_left;
Mat		g_greyscale_image_right;
Mat		g_depth;
Mat		g_disparity;
#endif
e_vbus_index sensor_id = e_vbus1;

DJI_lock    g_lock;
DJI_event   g_event;
char		key = 0;

int my_callback(int data_type, int data_len, char *content)
{
	g_lock.enter();
	if (e_image == data_type && NULL != content)
	{
		image_data* data = (image_data* )content;
		printf( "frame index:%d,stamp:%d\n", data->frame_index, data->time_stamp );
 #ifdef HAVE_OPENCV
		if ( data->m_greyscale_image_left[sensor_id] ){
			g_greyscale_image_left = Mat::zeros(HEIGHT,WIDTH,CV_8UC1);
			memcpy( g_greyscale_image_left.data, data->m_greyscale_image_left[sensor_id], IMAGE_SIZE );
		}else g_greyscale_image_left.release();
		if ( data->m_greyscale_image_right[sensor_id] ){
			g_greyscale_image_right = Mat::zeros(HEIGHT,WIDTH,CV_8UC1);
			memcpy( g_greyscale_image_right.data, data->m_greyscale_image_right[sensor_id], IMAGE_SIZE );
		}else g_greyscale_image_right.release();
		if ( data->m_depth_image[sensor_id] ){
			g_depth = Mat::zeros(HEIGHT,WIDTH,CV_16SC1);
			memcpy( g_depth.data, data->m_depth_image[sensor_id], IMAGE_SIZE * 2 );
		}else g_depth.release();
		if ( data->m_disparity_image[sensor_id] ){
			g_disparity = Mat::zeros(HEIGHT,WIDTH,CV_16SC1);
			memcpy( g_disparity.data, data->m_disparity_image[sensor_id], IMAGE_SIZE * 2 );
		}else g_disparity.release();
 #endif
	}

	if ( e_imu == data_type && NULL != content )
	{
		imu *imu_data = (imu*)content;
		printf( "imu:%f %f %f,%f %f %f %f\n", imu_data->acc_x, imu_data->acc_y, imu_data->acc_z, imu_data->q[0], imu_data->q[1], imu_data->q[2], imu_data->q[3] );
		printf( "frame index:%d,stamp:%d\n", imu_data->frame_index, imu_data->time_stamp );
	}

	if ( e_velocity == data_type && NULL != content )
	{
		velocity *vo = (velocity*)content;
		printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo->vx, 0.001f * vo->vy, 0.001f * vo->vz );
		printf( "frame index:%d,stamp:%d\n", vo->frame_index, vo->time_stamp );
	}

	if ( e_obstacle_distance == data_type && NULL != content )
	{
		obstacle_distance *oa = (obstacle_distance*)content;
		printf( "obstacle distance:" );
		for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
			printf( " %f ", 0.01f * oa->distance[i] );
		
		printf( "\n" );
		printf( "frame index:%d,stamp:%d\n", oa->frame_index, oa->time_stamp );
	}

	if ( e_ultrasonic == data_type && NULL != content )
	{
		ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
		for ( int d = 0; d < CAMERA_PAIR_NUM; ++d )
			printf( "ultrasonic distance:%f,reliability:%d\n", ultrasonic->ultrasonic[d] * 0.001f, (int)ultrasonic->reliability[d] );
		
		printf( "frame index:%d,stamp:%d\n", ultrasonic->frame_index, ultrasonic->time_stamp );
	}

	if(e_motion == data_type && NULL!=content){
		motion* m=(motion*)content;
		printf("(px,py,pz)=(%.2f,%.2f,%.2f)\n", m->position_in_global_x,m->position_in_global_y,m->position_in_global_z);
	}
	g_lock.leave();
	g_event.set_event();

	return 0;
}


#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return -1;}}


int init_Guidance(){

	reset_config();  // clear all data subscription

	int err_code = init_transfer(); //wait for board ready and init transfer thread
	RETURN_IF_ERR( err_code );

	int online_status[CAMERA_PAIR_NUM];
	err_code = get_online_status(online_status);
	RETURN_IF_ERR(err_code);
	cout<<"Sensor online status: ";
	for (int i=0; i<CAMERA_PAIR_NUM; i++)
		cout<<online_status[i]<<" ";
	cout<<endl;

	// get cali param
	stereo_cali cali[CAMERA_PAIR_NUM];
	err_code = get_stereo_cali(cali);
	RETURN_IF_ERR(err_code);
	cout<<"cu\tcv\tfocal\tbaseline\n";
	for (int i=0; i<CAMERA_PAIR_NUM; i++)
	{
		cout<<cali[i].cu<<"\t"<<cali[i].cv<<"\t"<<cali[i].focal<<"\t"<<cali[i].baseline<<endl;
	}

 #if !USE_GUIDANCE_ASSISTANT_CONFIG
	err_code = select_greyscale_image( sensor_id, true );
	RETURN_IF_ERR( err_code );
	err_code = select_greyscale_image( sensor_id, false );
	RETURN_IF_ERR( err_code );
 #if SELECT_DEPTH_DATA
	err_code = select_depth_image( sensor_id );
	RETURN_IF_ERR( err_code );
	err_code = select_disparity_image( sensor_id );
	RETURN_IF_ERR( err_code );
 #endif
	select_imu();
	select_ultrasonic();
	select_obstacle_distance();
	select_velocity();
	select_motion();
 #endif

	e_device_type dt;
	get_device_type(&dt);
	cout<<"device type: "<<(dt==Guidance?"Guidance":"GuidanceLite")<<endl;

	get_image_size(&WIDTH, &HEIGHT);
	cout<<"(width, height)="<<WIDTH<<", "<<HEIGHT<<endl;

	err_code = set_sdk_event_handler( my_callback );
	RETURN_IF_ERR( err_code );
	err_code = start_transfer();
	RETURN_IF_ERR( err_code );

	// for setting exposure
	exposure_param para;
	para.m_is_auto_exposure = 1;
	para.m_step = 10;
	para.m_expected_brightness = 120;
	para.m_camera_pair_index = sensor_id;

}

/*
void init_Onboard(){

}
*/

int init(LinuxSerialDevice* serialDevice, CoreAPI* api, LinuxThread* read){


	/* init Guidance*/
	init_Guidance();

	/* init Onboard*/
	//init_Onboard();

  //! Setup
  int setupStatus = setup(serialDevice, api, read);
  if (setupStatus == -1)
  {
    std::cout << "This program will exit now. \n";
    return 0;
  }
  //! Set broadcast Freq Defaults
  unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);
  usleep(500000);

}

void VRC(void* param){

}

int main(int argc, char** argv){


	/* prepare object & share data

	*/

  //! Instantiate a serialDevice, an API object, flight and waypoint objects and a read thread.
  LinuxSerialDevice* serialDevice = new LinuxSerialDevice(UserConfig::deviceName, UserConfig::baudRate);
  CoreAPI* api = new CoreAPI(serialDevice);

  LinuxThread read(api, 2);



	/*init Guidiance & Onboard
	it will produce two thread:
		1. callback save image
		2. readpoll read boardcast

	*/
  init(serialDevice, api, &read);




	/*	create ultrasonic thread
	it will produce one thread
		1. ultrasonic
	*/

	/*process thread of imageProcess & map & path.
	it will produce three threads:
		1. imageProcess
		2. map produce
		3. pathPlan
	*/


	/*	create VRC
	it will produce one thread:
		1. VRC		
	*/
  
   /* takeoff */

  /*follow me*/
  Follow* follow = new Follow(api);
  FollowData data = follow->getData();
  data.target.latitude += 0.000003;
  data.target.longitude += 0.000003;

  follow->start(&data, 1);

//  pthread_create(&waypointID, NULL, VRC, api);

	
}