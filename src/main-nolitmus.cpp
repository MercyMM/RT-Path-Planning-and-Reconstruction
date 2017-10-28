#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
//#include <time.h>



#include "DJI_guidance.h"
#include "DJI_utility.h"

#include "LinuxSerialDevice.h"
#include "LinuxThread.h"
#include "LinuxSetup.h"
#include "LinuxFlight.h"
#include "LinuxCleanup.h"
#include "ReadUserConfig.h"
#include "LinuxWaypoint.h"

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
e_vbus_index sensor_id = e_vbus2;

DJI_lock    g_lock;
DJI_event   g_event;
char		key = 0;

std::ostream& operator<<(std::ostream& out, const e_sdk_err_code value){
	const char* s = 0;
	static char str[100]={0};
#define PROCESS_VAL(p) case(p): s = #p; break;
	switch(value){
		PROCESS_VAL(e_OK);     
		PROCESS_VAL(e_load_libusb_err);     
        PROCESS_VAL(e_sdk_not_inited);
		PROCESS_VAL(e_disparity_not_allowed);
		PROCESS_VAL(e_image_frequency_not_allowed);
		PROCESS_VAL(e_config_not_ready);
		PROCESS_VAL(e_online_flag_not_ready);
		PROCESS_VAL(e_stereo_cali_not_ready);
		PROCESS_VAL(e_libusb_io_err);
		PROCESS_VAL(e_timeout);
	default:
		strcpy(str, "Unknown error");
		s = str;
		break;
	}
#undef PROCESS_VAL

	return out << s;
}

int my_callback(int data_type, int data_len, char *content)
{
	g_lock.enter();
	if (e_image == data_type && NULL != content)
	{
		image_data* data = (image_data* )content;
//		printf( "frame index:%d,stamp:%d\n", data->frame_index, data->time_stamp );
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
//		printf( "imu:%f %f %f,%f %f %f %f\n", imu_data->acc_x, imu_data->acc_y, imu_data->acc_z, imu_data->q[0], imu_data->q[1], imu_data->q[2], imu_data->q[3] );
//		printf( "frame index:%d,stamp:%d\n", imu_data->frame_index, imu_data->time_stamp );
	}

	if ( e_velocity == data_type && NULL != content )
	{
		velocity *vo = (velocity*)content;
//		printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo->vx, 0.001f * vo->vy, 0.001f * vo->vz );
//		printf( "frame index:%d,stamp:%d\n", vo->frame_index, vo->time_stamp );
	}

//	if ( e_obstacle_distance == data_type && NULL != content )
//	{
//		obstacle_distance *oa = (obstacle_distance*)content;
//		printf( "obstacle distance:" );
//		for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
//			printf( " %f ", 0.01f * oa->distance[i] );
		
//		printf( "\n" );
//		printf( "frame index:%d,stamp:%d\n", oa->frame_index, oa->time_stamp );
//	}

//	if ( e_ultrasonic == data_type && NULL != content )
//	{
//		ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
//		for ( int d = 0; d < CAMERA_PAIR_NUM; ++d )
//			printf( "ultrasonic distance:%f,reliability:%d\n", ultrasonic->ultrasonic[d] * 0.001f, (int)ultrasonic->reliability[d] );
		
//		printf( "frame index:%d,stamp:%d\n", ultrasonic->frame_index, ultrasonic->time_stamp );
//	}

//	if(e_motion == data_type && NULL!=content){
//		motion* m=(motion*)content;
//		printf("(px,py,pz)=(%.2f,%.2f,%.2f)\n", m->position_in_global_x,m->position_in_global_y,m->position_in_global_z);
//	}
	g_lock.leave();
	g_event.set_event();

	return 0;
}

#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return -1;}}
#define RETURN_IF_ERR2(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return NULL;}}


pthread_mutex_t m_MC;

int init(LinuxSerialDevice* serialDevice, CoreAPI* api, LinuxThread* read){




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

	/* init Guidance*/
	//init_Guidance();

}



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
//	err_code = select_disparity_image( sensor_id );
//	RETURN_IF_ERR( err_code );
 #endif
    select_imu();
    select_ultrasonic();
    select_obstacle_distance();
    select_velocity();
    //select_motion();
 #endif

//	e_device_type dt;
//	get_device_type(&dt);
//	cout<<"device type: "<<(dt==Guidance?"Guidance":"GuidanceLite")<<endl;

//	get_image_size(&WIDTH, &HEIGHT);
//	cout<<"(width, height)="<<WIDTH<<", "<<HEIGHT<<endl;

    err_code = set_sdk_event_handler( my_callback );
    RETURN_IF_ERR( err_code );
    err_code = start_transfer();
    RETURN_IF_ERR( err_code );


}

void* imageProcess(void* api)
{

    init_Guidance();

    // for setting exposure
    exposure_param para;
    para.m_is_auto_exposure = 1;
    para.m_step = 10;
    para.m_expected_brightness = 120;
    para.m_camera_pair_index = sensor_id;
    int err_code;
	while(1)
	{
		g_event.wait_event();
#ifdef HAVE_OPENCV
		if(!g_greyscale_image_left.empty())
			imshow(string("left_")+char('0'+sensor_id), g_greyscale_image_left);
		if(!g_greyscale_image_right.empty())
			imshow(string("right_")+char('0'+sensor_id), g_greyscale_image_right);

		if(!g_depth.empty()){
			Mat depth8(HEIGHT,WIDTH,CV_8UC1);
			g_depth.convertTo(depth8, CV_8UC1);
			imshow(string("depth_")+char('0'+sensor_id), depth8);
//			printf("Depth at point (%d,%d) is %f meters!\n", HEIGHT/2, WIDTH/2,  float(g_depth.at<short>( HEIGHT/2,WIDTH/2))/128);
		}
		if(!g_disparity.empty()){
			Mat disp8(HEIGHT,WIDTH, CV_8UC1);
			g_disparity.convertTo(disp8, CV_8UC1);
			imshow(string("disparity_")+char('0'+sensor_id), disp8);
//			printf("Disparity at point (%d,%d) is %f pixels!\n", HEIGHT/2, WIDTH/2,  float(g_disparity.at<short>( HEIGHT/2,WIDTH/2))/16);
		}

		key = waitKey(1);


        usleep(50000);
        pthread_mutex_unlock(&m_MC);


#endif

		if (key > 0){
            // set exposure parameters
			if (key == 'j' || key == 'k' || key == 'm' || key == 'n'){
				if (key == 'j'){
					if (para.m_is_auto_exposure) para.m_expected_brightness += 20;
					else para.m_exposure_time += 3;
				}
				else if (key == 'k'){
					if (para.m_is_auto_exposure) para.m_expected_brightness -= 20;
					else para.m_exposure_time -= 3;
				}
				else if (key == 'm'){
					para.m_is_auto_exposure = !para.m_is_auto_exposure;
					cout << "exposure is " << para.m_is_auto_exposure << endl;
				}
				else if (key == 'n'){//return to default
					para.m_expected_brightness = para.m_exposure_time = 0;
				}

				cout << "Setting exposure parameters....SensorId=" << sensor_id << endl;
				para.m_camera_pair_index = sensor_id;
//				set_exposure_param(&para);
				key = 0;
			}
			else if (key == 'q' || key == 'w' || key == 'd' || key == 'x' || key == 'a' || key == 's'){// switch image direction
#ifdef HAVE_OPENCV
				destroyAllWindows();
#endif
				err_code = stop_transfer();
                RETURN_IF_ERR2(err_code);
				reset_config();

				if (key == 'q') break;
				if (key == 'w') sensor_id = e_vbus1;
				if (key == 'd') sensor_id = e_vbus2;
				if (key == 'x') sensor_id = e_vbus3;
				if (key == 'a') sensor_id = e_vbus4;
				if (key == 's') sensor_id = e_vbus5;

				select_greyscale_image(sensor_id, true);
				select_greyscale_image(sensor_id, false);

				select_depth_image(sensor_id);
//				select_disparity_image(sensor_id);
				err_code = start_transfer();
                RETURN_IF_ERR2(err_code);
				key = 0;
			}
		}
	}

	err_code = stop_transfer();
	RETURN_IF_ERR2( err_code );
	//make sure the ack packet from GUIDANCE is received
    sleep( 1000000 );
	err_code = release_transfer();
	RETURN_IF_ERR2( err_code );

	return 0;
}

int main(int argc, char** argv)
{


    //! Instantiate a serialDevice, an API object, flight and waypoint objects and a read thread.
    LinuxSerialDevice* serialDevice = new LinuxSerialDevice(UserConfig::deviceName, UserConfig::baudRate);
    CoreAPI* api = new CoreAPI(serialDevice);
    Flight* flight = new Flight(api);

    LinuxThread read(api, 2);

    pthread_t imagePID;
    pthread_create(&imagePID, NULL, imageProcess, (void*)api);

//    cout<<"after start guidance,sleep 10 seconds"<<endl;
//    usleep(10000000);


    /*init Guidiance & Onboard
    it will produce two thread:
        1. callback save image
        2. readpoll read boardcast

    */
  init(serialDevice, api, &read);



      /*! Set a blocking timeout - this is the timeout for blocking API calls
          to wait for acknowledgements from the aircraft. Do not set to 0.
      !*/
      int blockingTimeout = 1; //Seconds

      //!  Takeoff
      cout<<"take off satart"<<endl;
      ackReturnData takeoffStatus = monitoredTakeoff(api, flight, blockingTimeout);
      cout<<"take off over"<<endl;

      //! Set broadcast Freq Defaults
      unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);

      //! If the aircraft took off, continue to do flight control tasks
      if (takeoffStatus.status == 1)
      {

        /*! This is where you can add your own flight functionality.
            Check out LinuxWaypoint and LinuxFlight for available APIs.
            You can always execute direct Onboard SDK Library API calls
            through the api object available in this example.
        !*/
//            cout<<"3 seconds late , start waypoint"<<endl;
//            usleep(3000000);


//            drawSqrPosCtrlSample(api, flight);

            float roll,pitch, throttle, yaw;
            roll = 5.0f;
            pitch = 0.0f;
            throttle = 0.0f;
            yaw = 0.0;
//            clock_t begin = times(NULL);
            //when Vset = 0.8,Vreal = 1.0. so distance is 1.0s * 12m/s = 12m
//            moveWithVelocity(api, flight, 0.8, 0.8, 0, 0, 100,0, 0);
//flight->setMovementControl(0x49, 1, 0, 0, 0);
//flight->setMovementControl(0x49, 1, 0, 0, 0);
//flight->setMovementControl(0x49, 1, 0, 0, 0);
// usleep(20000);
           // moveByPositionOffset(api, flight, 0, 20, 0, 0);
            uint8_t flag = 0x49; //Velocity Control
            int elapsedTime = 0;
            while (elapsedTime <= 10)

            {
                flight->setMovementControl(flag, 0.8, 0.8, 0, 0);
                usleep(1000000);
                elapsedTime += 1;
            }



//            clock_t end = times(NULL);
//            cout<< "times = " << ((end-begin)/CLOCK_PERSEC)<<endl;

//            moveByPositionOffset(api, flight, 10, 0, -3, 0);
//            moveByPositionOffset(api, flight, 0, -10, 3, 0);
//            moveByPositionOffset(api, flight, -10, 0, -3, 0);
           // flight->setMovementControl(0x91, roll, pitch, throttle, yaw);
//            flight->setMovementControl(0x91, 0.0f, 0.0f, 0.0f, 0.0f);
            cout<< "sleep 2 s" << endl;
            usleep(2000000);

            int timeout = 1;
//             WayPoint* waypointObj = new WayPoint(api);


//            //! Create an ackReturnData struct
//            ackReturnData wpAck;
//            //! Create four waypoints near your current position
//            int numWaypoints = 2;
//            //! Init the mission
//            std::cout << "Init Waypoint mission\n";
//            wpAck = initWaypointMission(api, waypointObj, numWaypoints, timeout);
//            std::cout << "Init Waypoint mission over \n";

//            //! Get current GPS location
//            PositionData curPosition = api->getBroadcastData().pos;

//            //! Create an offset in radians for the waypoint creation loop to access and permute.
//            float64_t offsetInRadians = 0.000003;
//            cout<< "la, lo orign :"<<curPosition.latitude<< " " << curPosition.longitude <<endl;
//            //cout<< "la, lo:"<<curPosition.latitude<< curPosition.longitude<<endl;

//cout<<"offsetInRadians:" << offsetInRadians<<endl;

//            //! Waypoint creation and upload
//            for (int index = 0; index < numWaypoints; index++)
//            {
//              PositionData wpPosition;
//              switch(index)
//              {
//                case 0:
//                  wpPosition = curPosition;
//                  wpPosition.latitude += offsetInRadians;
//                  wpPosition.altitude = 2;
//                  break;
//                case 1:
//                  wpPosition = curPosition;
//                  wpPosition.latitude += offsetInRadians;
//                  wpPosition.longitude += offsetInRadians;
//                  wpPosition.altitude = 2;
//                  break;
//                case 2:
//                  wpPosition = curPosition;
//                  wpPosition.longitude += offsetInRadians;
//                  wpPosition.altitude = 2;
//                  break;
//                case 3:
//                  wpPosition = curPosition;
//                  wpPosition.altitude = 2;
//                  break;
//              }
//              cout<< "la, lo "<< index <<" :  "<<wpPosition.latitude<< " " << wpPosition.longitude <<endl;
////              std::cout << "Adding waypoint " << index << "\n";
//              wpAck = addWaypoint(api, waypointObj, &wpPosition, (uint8_t)index, timeout);
////              std::cout << "Adding waypoint over" << index << "\n";

//            }
//            //! Start mission
////            std::cout << "Starting Waypoint mission\n";
//            wpAck = startWaypointMission(api, waypointObj, timeout);

//            cout<< "20 seconds late, start again"<<endl;
//            usleep(20000000);

            /*follow me*/
//            Follow* follow = new Follow(api);
//            FollowData data = follow->getData();
//            cout<<"follw data now is:"<<data.target.latitude << " "<<data.target.longitude<<endl;

//            FollowTarget target = data.target;
//            target.latitude += 0.000003;
//            target.longitude += 0.000003;
//            cout<<"follw data to is:"<<target.latitude << " "<<target.longitude<<endl;

//            follow->updateTarget(target.latitude, target.longitude, target.height + 5, target.angle);
//            cout<<"update"<<endl;

//            data.target = target;
//            MissionACK followACK = follow->start(&data, 50);
//cout<<"start ACK: "<< followACK <<endl;
//            follow->start(&data, 0, 0);

//            cout<<"start  "<<endl;



//            usleep(50000000);




          //! Land
//            ackReturnData landingStatus = landing(api, flight,blockingTimeout);



//            std::cout << "Starting Waypoint mission over\n";
//            cout<< "5 seconds late, start again"<<endl;
//          usleep(20000000);
//          std::cout << "Init Waypoint mission\n";
//          wpAck = initWaypointMission(api, waypointObj, numWaypoints, timeout);
//          std::cout << "Init Waypoint mission over \n";

//          for (int index = 0; index < numWaypoints; index++)
//          {
//            PositionData wpPosition;
//            switch(index)
//            {
//              case 0:
//                wpPosition = curPosition;
//                wpPosition.latitude += offsetInRadians;
//                wpPosition.altitude = 2;
//                break;
//              case 1:
//                wpPosition = curPosition;
//                wpPosition.latitude += offsetInRadians;
//                wpPosition.longitude += offsetInRadians;
//                wpPosition.altitude = 2;
//                break;
//              case 2:
//                wpPosition = curPosition;
//                wpPosition.longitude += offsetInRadians;
//                wpPosition.altitude = 2;
//                break;
//              case 3:
//                wpPosition = curPosition;
//                wpPosition.altitude = 2;
//                break;
//            }
//            std::cout << "Adding waypoint " << index << "\n";
//            wpAck = addWaypoint(api, waypointObj, &wpPosition, (uint8_t)index, timeout);
//            std::cout << "Adding waypoint over" << index << "\n";

//          }
//          //! Start mission
//          std::cout << "Starting Waypoint mission\n";
//          wpAck = startWaypointMission(api, waypointObj, timeout);
//          std::cout << "Starting Waypoint mission over\n";


//          wpAck = pauseWaypointMission(api, waypointObj, timeout);
//          usleep(5000000);
//          wpAck = resumeWaypointMission(api, waypointObj, timeout);

//            //! Waiting for mission to finish. Increase this sleep if you increase the waypoint offset or number of waypoints.
//            usleep(50000000);







//            int timeout = 1;
//            WayPoint* waypointObj = new WayPoint(api);
//            ackReturnData wpAck;
//            int numWaypoints = 1;
//            std::cout << "Init Waypoint mission\n";
//            wpAck = initWaypointMission(api, waypointObj, numWaypoints, timeout);

//            PositionData curPosition = api->getBroadcastData().pos;
//            float64_t offsetInRadians = 0.000003;
//                int index = 0;
//                PositionData wpPosition;
//                wpPosition = curPosition;
//                wpPosition.latitude += offsetInRadians;
//                wpPosition.latitude += 2;

//                cout<<"add one waypoint" <<endl;
//                wpAck = addWaypoint(api, waypointObj, &wpPosition, (uint8_t)index, timeout);

//                cout << "Starting Waypoint mission\n";
//                wpAck = startWaypointMission(api, waypointObj, timeout);

//                usleep(5000000);
//                wpPosition.longitude += offsetInRadians;
//                cout<<"add one waypoint" <<endl;
//                wpAck = addWaypoint(api, waypointObj, &wpPosition, (uint8_t)index, timeout);

//                cout << "Starting Waypoint mission\n";
//                wpAck = startWaypointMission(api, waypointObj, timeout);


            //! Land
            ackReturnData landingStatus = landing(api, flight,blockingTimeout);



      }
      else
      {
        //Try to land directly
        ackReturnData landingStatus = landing(api, flight,blockingTimeout);
      }

    //! No mode specified or invalid mode specified"

    //! Cleanup
    int cleanupStatus = cleanup(serialDevice, api, flight, &read);
    if (cleanupStatus == -1)
    {
      std::cout << "Unable to cleanly destroy OSDK infrastructure. There may be residual objects in the system memory.\n";
      return 0;
    }
    std::cout << "Program exited successfully." << std::endl;

    int status;
    pthread_join(imagePID, (void**)&status);

    return 0;


}
