#include "main.h"
#include "signal.h"




#define SAFE_DIS	0.3   //M100: change to 1m
int g_reliab , g_distance ;
int32_t D_can_width = 60;  //[15,310] => 60
int32_t D_can_height = 48; //[5, 230] => 46
point goal = {20,0};
PositionData goal_P;
PositionData GoalGPS = {0.5348495959, 1.8183969806, 0, 0, 0};
/*
typedef struct PositionData

{
  float64_t latitude;
  float64_t longitude;
  float32_t altitude;
  float32_t height;
  uint8_t health;
} PositionData;
*/





static int no_printf(const char *format, ...){
    return 0;
}

#ifdef PRINTF
    #define PRINTF_LOG printf
#else
    #define PRINTF_LOG no_printf
#endif
int my_callback(int data_type, int data_len, char *content);
int setupDJI(LinuxSerialDevice* serialDevice, CoreAPI* api, LinuxThread* read);
int init_Guidance();
void* imageProcess(void* api);
//int init_rt_param(struct rt_task param);
int ElasCom(const Elas &elas, uint8_t *I1, uint8_t *I2);
void SetGoal(CoreAPI* api, float Xoff, float Yoff);
int GetOrSetIndex(int flag, int index);
int BitCount(unsigned int n);

//int moveWithVelocity(CoreAPI* api, Flight* flight, float32_t xVelocityDesired, float32_t yVelocityDesired, float32_t zVelocityDesired, float32_t yawRateDesired ,  int timeoutInMs, float yawRateThresholdInDegS = 0.5, float velThresholdInMs = 0.5);
int guidance_start = 0;
void sig_handler(int sig){
    if(sig == SIGINT){
        if(guidance_start == 0)
            exit(0);
        PRINTF_LOG("stop_transfe1\n");
        int err_code = stop_transfer();
        RETURN_IF_ERR2( err_code );
        //make sure the ack packet from GUIDANCE is received
        usleep( 10000 );
        PRINTF_LOG("stop_transfe2\n");
        err_code = release_transfer();
        PRINTF_LOG("stop_transfe3\n");
        RETURN_IF_ERR2( err_code );
        PRINTF_LOG("stop_transfe4\n");
        exit(0);
    }
}

int main(int argc, char** argv)
{
    signal(SIGINT, sig_handler);
//    PRINTF_LOG("wait 5min\n");
//    usleep(1000000);


    int status;
    //! Instantiate a serialDevice, an API object, flight and waypoint objects and a read thread.
    LinuxSerialDevice* serialDevice = new LinuxSerialDevice(UserConfig::deviceName, UserConfig::baudRate);
    CoreAPI* api = new CoreAPI(serialDevice);
    Flight* flight = new Flight(api);
    LinuxThread read(api, 2); //1->send thread; 2->read_call

//    sem_init(&g_empty, 0, QUEUE_LEN); //init 2; not too mach
    sem_init(&g_empty, 0, QUEUE_LEN); //init 2; not too mach
    sem_init(&g_full, 0, 0);
    sem_init(&disTooSmall, 0, 0);
    sem_init(&sem_Index, 0, 1);
    sem_init(&SyncImage, 0, 0);

//    PRINTF_LOG("main tid = %d\n", gettid());
    pthread_t imagePID;
    pthread_t imagePID2;

//        pthread_create(&imagePID2, NULL, changeIndex, NULL);
//        pthread_join( imagePID2, NULL);
//        pthread_join( imagePID, NULL);
//        return 0;

    /*init Guidiance & Onboard
    it will produce two thread:
        1. callback save image
        2. readpoll read boardcast

    */
    setupDJI(serialDevice, api, &read);
    double Xoff = 10;   //go north 10m;
    double Yoff = 0;
    PositionData curPosition = api->getBroadcastData().pos;
    goal_P.latitude = curPosition.latitude + Xoff / C_EARTH;
    goal_P.longitude = curPosition.longitude;

    /*! Set a blocking timeout - this is the timeout for blocking API calls
          to wait for acknowledgements from the aircraft. Do not set to 0.
      !*/
    int blockingTimeout = 1; //Seconds
    //!  Takeoff
    ackReturnData takeoffStatus;
//    takeoffStatus = monitoredTakeoff(api, flight, blockingTimeout);

/*
           for(int i = 0; i < 100; i++){
                flight->setMovementControl(0x4b, 0.2, 0, 0, 0);
                usleep(100000); // 10000 ns;????
           }

return 0;
*/
    pthread_create(&imagePID, NULL, imageProcess, (void*)api);
    sem_wait(&SyncImage);
    usleep(70000);
//    pthread_join( imagePID, NULL);
//    return 0;


/*
while(1){
  QuaternionData curQuaternion = api->getBroadcastData().q;
  DJI::EulerAngle curEuler = Flight::toEulerAngle(curQuaternion);
printf("current yaw: %f\n", curEuler.yaw / DEG2RAD);
usleep(1000000);
}

return 0;

  QuaternionData curQuaternion = api->getBroadcastData().q;
  DJI::EulerAngle curEuler = Flight::toEulerAngle(curQuaternion);
printf("start \n");
attitudeControl(api, flight, 0, 0, 60, 10000, 1);
   curQuaternion = api->getBroadcastData().q;
   curEuler = Flight::toEulerAngle(curQuaternion);
printf("current yaw: %f\n", curEuler.yaw / DEG2RAD);
usleep(10000000);


usleep(5000000);
attitudeControl(api, flight, 0, 0, -60, 10000, 1);
printf("start stop\n");
   curQuaternion = api->getBroadcastData().q;
   curEuler = Flight::toEulerAngle(curQuaternion);
printf("current yaw: %f\n", curEuler.yaw / DEG2RAD);
usleep(10000000);
*/


    //! If the aircraft took off, continue to do flight control tasks
    takeoffStatus.status = 1;
    if (takeoffStatus.status == 1)
    {

        /*! This is where you can add your own flight functionality.
            Check out LinuxWaypoint and LinuxFlight for available APIs.
            You can always execute direct Onboard SDK Library API calls
            through the api object available in this example.
        !*/
        //init flight thread real-time param
        //set goal

//           for(int i = 0; i < 100; i++){
//                flight->setMovementControl(flag, 0.2, 0, 0, 0);
//                usleep(100000); // 10000 ns;????
//           }
//return 0;
        int FlyTime = 0;
        int StopTime = 0;
        unsigned int direct = 0;
        while(1)
        {

            float Vx, Vy;

            //who update GPS ???
            //1. sem_wait Vx, Vy
            //2. moveWithVelocity( Vx, Vy)
            int index_copy ;
            index_copy = GetOrSetIndex(0, 0);


            if(index_copy < 0)
            {
                moveWithVelocity(api, flight, 0, 0, 0, 0, 21);
                usleep(10000);
/*                StopTime += 10;
                if( StopTime > 1000){
                    int count = BitCount(direct);
                    if( count > 5){
                        //move right
                        uint8_t flag = 0x4b; //Velocity Control
                        for(int j = 0; j < 50; j++){
                            flight->setMovementControl(flag, 0, -0.5, 0, 0);
                            usleep(20000);
                        }
                    }else {
                        //move left
                        uint8_t flag = 0x4b; //Velocity Control
                        for(int j = 0; j < 50; j++){
                            flight->setMovementControl(flag, 0, 0.5, 0, 0);
                            usleep(20000);
                        }
                    }

                    StopTime = 0;
                    printf("index_copy < 0; stop too long\n");
                }
                PRINTF_LOG("stop");
*/
            }else //if ( index_copy >= 0 && index_copy <= 16)
            {
                StopTime = 0;
             /*   if (index_copy > 8){
                    direct = direct << 1;
                }else {
                    direct = direct << 1 + 1;
                }
               */
                Vx = -SpeedWindow[index_copy].x;
                Vy = SpeedWindow[index_copy].y;
                PRINTF_LOG("%d: %f, %f\n", index_copy, Vx, Vy);

                uint8_t flag = 0x4b; //Velocity Control
                flight->setMovementControl(flag, Vx, Vy, 0, 0);
                usleep(10000); // 10000 ns;????

/*
                PositionData curPosition = api->getBroadcastData().pos;
                double deltaLon = GoalGPS.longitude - curPosition.longitude;
                double deltaLat = GoalGPS.latitude - curPosition.latitude;
                double x =  deltaLon * C_EARTH;
                double y = deltaLat * C_EARTH * cos(GoalGPS.latitude);
                if( (x*x + y*y) < 1){
                    attitudeControl(api, flight, 0, 0, 0, 10000, 1);
                    attitudeControl(api, flight, 0, 0, 180, 10000, 1);
                    attitudeControl(api, flight, 0, 0, 90, 10000, 1);
                    attitudeControl(api, flight, 0, 0, 270, 10000, 1);
                    printf("arrive :) \n\n");
                    while(1);
                }

                FlyTime += 10;
                if(FlyTime > 5000){
                    FlyTime = 0;
                    deltaLon = GoalGPS.longitude - curPosition.longitude;
                    deltaLat = GoalGPS.latitude - curPosition.latitude;
                    x =  deltaLon * C_EARTH;
                    y =  deltaLat * C_EARTH * cos(GoalGPS.latitude);
                    //printf(" x,y: %lf,  %lf  \n", x, y);
                    double yawDesired;
                    if( x != 0 )
                        yawDesired  = atan(y/x) * 180 / 3.14159265;
                    else {
                        if( y > 0)
                            yawDesired = 90;
                        else
                            yawDesired = -90;
                    }
                    if(x < 0 && y > 0)
                        yawDesired = 180 + yawDesired;
                    else if( x < 0 && y < 0)
                        yawDesired = -(180 - yawDesired);
                    QuaternionData curQuaternion = api->getBroadcastData().q;
                    DJI::EulerAngle curEuler = Flight::toEulerAngle(curQuaternion);
                    if (abs((curEuler.yaw/ DEG2RAD) - yawDesired) < 1){
                        FlyTime = 2000;
                    }else {
                        attitudeControl(api, flight, 0, 0, yawDesired, 10000, 1);
                    }

                }
      */      }

        }
//           for(int i = 0; i < 50; i++){
//                flight->setMovementControl(flag, 0.5, 0, 0, 0);
//                usleep(100000); // 10000 ns;????
//           }
//           for(int i = 0; i < 50; i++){
//                flight->setMovementControl(flag, -0.5, 0, 0, 0);
//                usleep(100000); // 10000 ns;????
//           }
//          int positionControlStatus = moveByPositionOffset(api, flight, 5, 0, 0, 0);
//            positionControlStatus = moveByPositionOffset(api, flight, -5, 0, 0, 0);
        //! Land
        ackReturnData landingStatus = landing(api, flight,blockingTimeout);
    }
    else
    {
        //Try to land directly
        ackReturnData landingStatus = landing(api, flight,blockingTimeout);
    }

    //! No mode specified or invalid mode specified"

      pthread_join( imagePID, NULL);

    //! Cleanup
    int cleanupStatus = cleanup(serialDevice, api, flight, &read);
    if (cleanupStatus == -1)
    {
        std::cout << "Unable to cleanly destroy OSDK infrastructure. There may be residual objects in the system memory.\n";
        return 0;
    }
    std::cout << "Program exited successfully." << std::endl;

    return 0;


}


int init_flag = 1;

clock_t start = 0;
clock_t stop = 0;
int delay;
struct timeval tStartTime, tEndTime;
int mmm = 0;
int my_callback(int data_type, int data_len, char *content)
{

    if( (0 == init_flag) )
    {
        init_flag = 1;
    }

    g_lock.enter();
    if (e_image == data_type && NULL != content)
    {
//        PRINTF_LOG("wait empty\n");
//        sem_wait(&g_empty);
//        PRINTF_LOG("enter empty\n");
        image_data* data = (image_data* )content;

        if ( data->m_greyscale_image_left[sensor_id] && data->m_greyscale_image_right[sensor_id])
        {
            g_greyscale_image_left[which] = Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
            g_greyscale_image_right[which] = Mat::zeros(HEIGHT,WIDTH,CV_8UC1);
            memcpy( g_greyscale_image_left[which].data, data->m_greyscale_image_left[sensor_id], IMAGE_SIZE );
            memcpy( g_greyscale_image_right[which].data, data->m_greyscale_image_right[sensor_id], IMAGE_SIZE );

//            PRINTF_LOG("write\n");

        }else
        {
            g_greyscale_image_left[which].release();
            g_greyscale_image_right[which].release();
            PRINTF_LOG("\n\n=====release\n");
//            while(1);
        }
//        sem_post(&g_full);
//        PRINTF_LOG("enter--\n");
        lock_which.enter();
        which_old = which;
        lock_which.leave();

        which++;
        if(which >= QUEUE_LEN)
        {
            which = 0;
        }
//        PRINTF_LOG("which = %d\n", which);
//        PRINTF_LOG("leave\n");
    }else if(e_ultrasonic == data_type && NULL != content)
    {
        ultrasonic_data *data = (ultrasonic_data* )content;
        short distance = data->ultrasonic[sensor_id];
        unsigned short reliab =  data->reliability[sensor_id];

            g_reliab = reliab;
            g_distance = distance;
//        PRINTF_LOG(" %d, %d\n", reliab, distance);

        if(reliab == 1 && distance < 800) //distance < 0.8m
        {
//            sem_wait(&disTooSmall);
            mmm++;
//            PRINTF_LOG("%d: %d ultrasonic_data = %d \n", mmm, sensor_id,distance);
            _index_vola = -5;
        }


//        gettimeofday(&tEndTime, NULL);
//        delay = 1000000 * (tEndTime.tv_sec - tStartTime.tv_sec) +\
//                (tEndTime.tv_usec - tStartTime.tv_usec);
//        PRINTF_LOG("obstacle_distance time = %d ms\n", delay / 1000);
//        tStartTime = tEndTime;

    }


    g_lock.leave();
    return 0;
}

int setupDJI(LinuxSerialDevice* serialDevice, CoreAPI* api, LinuxThread* read){

  //! Setup
  int setupStatus = setup(serialDevice, api, read);
  if (setupStatus == -1)
  {
    std::cout << "This program will exit now. \n";
    return 0;
  }
  //! Set broadcast Freq Defaults
  unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);
  usleep(2000000);

}

int init_Guidance()
{
    reset_config();  // clear all data subscription

    //Initialize Guidance and create data transfer thread.
//    PRINTF_LOG("create data transfer thread\n");
    int err_code = init_transfer();
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
    //when you are at standard mode and enabled obstacle sensing function(guidance connect M100 by uart)
    //you are not allowed to get disparity or depth
//    err_code = select_depth_image( sensor_id );
//    RETURN_IF_ERR( err_code );
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

//    set_image_frequecy(e_frequecy_10);

    err_code = set_sdk_event_handler( my_callback );
    RETURN_IF_ERR( err_code );
    err_code = start_transfer();
    RETURN_IF_ERR( err_code );

//    PRINTF_LOG("init Mat \n");
    //init g_greyscale_image_left[QUEUE_LEN] data;
//    for(int i = 0; i < QUEUE_LEN; i ++)
//    {
//        g_greyscale_image_right[which] = Mat::zeros(HEIGHT,WIDTH,CV_8UC1);
//    }
}

void* imageProcess(void* api)
{

    init_Guidance();
    guidance_start = 1;
    char key;
    char name_left[] = "l_0.png";
    char name_right[] = "r_0.png";
    char name_disp[] = "d_0.png";

//    cv::VideoWriter right("videoRight.avi", CV_FOURCC('M', 'J','P','G'), \
//                          25.0, Size(320,240), 0);
    // for setting exposure
    exposure_param para;
    para.m_is_auto_exposure = 1;
    para.m_step = 10;
    para.m_expected_brightness = 120;
    para.m_camera_pair_index = sensor_id;
    int err_code;

    //global
    SetDeviceMap();   //cudaSetDeviceFlags(cudaDeviceMapHost);
    Elas::parameters param;
    Elas elas(param, (int32_t)WIDTH, (int32_t)HEIGH, D_can_width, D_can_height );

    sem_post(&SyncImage);

//    string ROOT = "/home/ubuntu/pathplan/pathPlan-test/image/";
//    string left = "left25.png";
//    string right = "right25.png";
//    IplImage *img1 , *img2 ;

	int ind = 0;
    while(1)
    {

//        string str = ROOT+left;
//        cout << str << endl;
//        img1 = cvLoadImage(str.c_str(),0);
//        str = ROOT+right;
//        img2 = cvLoadImage(str.c_str(),0);
//        cout << str << endl;
//        g_greyscale_image_left[0] = Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
//        g_greyscale_image_right[0] = Mat::zeros(HEIGHT,WIDTH,CV_8UC1);
//        memcpy( g_greyscale_image_left[0].data, img1->imageData, IMAGE_SIZE );
//        memcpy( g_greyscale_image_right[0].data, img2->imageData, IMAGE_SIZE );


//        PRINTF_LOG("continue\n");
//        sem_wait(&g_full);
        lock_which.enter();
        int which_cpy = which_old;
//        int which_cpy = 0;
        lock_which.leave();

        if(!g_greyscale_image_left[which_cpy].empty() && !g_greyscale_image_right[which_cpy].empty())
        {
//            PRINTF_LOG("show\n");
//            imshow(string("left_")+char('0'+sensor_id), g_greyscale_image_left[which_cpy]);
//            imshow(string("right_")+char('0'+sensor_id), g_greyscale_image_right[which_cpy]);

//            key =  cvWaitKey(1);

            //1. call elas add DWA; return Vx, Vy;
            //2. sem_post Vx, Vy; func main call sem_wait and get Vx, Vy;
            //elas + DWA

            struct timeval start, end;
            double timeuse;
            gettimeofday(&start, NULL);

            ElasCom(elas, (uint8_t*)g_greyscale_image_left[which_cpy].data, \
                        (uint8_t*)g_greyscale_image_right[which_cpy].data);
            gettimeofday(&end, NULL);
            timeuse = 1000000* (end.tv_sec-start.tv_sec) + end.tv_usec-start.tv_usec;
            printf("ElasCom : %fms\n", timeuse/1000);
//            cout<< "dis: "<< g_reliab <<", "<<g_distance<<endl;
//            usleep(100000);
            cout << ind << "gg index: " << GetOrSetIndex(0,0) << endl;
            cout << endl;
        ind++;
//        return 0;
            /*
                imwrite(name_left, g_greyscale_image_left[which_cpy]);
                name_left[2]++;
                imwrite(name_right, g_greyscale_image_right[which_cpy]);
                name_right[2]++;
if( name_right[2] > '9')
return 0;
*/
        /*
        char pic[50];
        sprintf(pic, "./image/left%02d.png", ind);
                imwrite(pic, g_greyscale_image_left[which_cpy]);
        sprintf(pic, "./image/right%02d.png", ind);
                imwrite(pic, g_greyscale_image_right[which_cpy]);
        key = getchar();
printf("stop");
*/
//        while(1);
//                key =  cvWaitKey(0);
//          return 0;
//usleep(1000000);
//PRINTF_LOG("sleep 1s\n");
//left[4]++;
//right[5]++;
            if('s' == key){
                imwrite("./left.png", g_greyscale_image_left[which_cpy]);
                name_left[2]++;
                imwrite("./right.png", g_greyscale_image_right[which_cpy]);
                name_right[2]++;
            }else if ('q' == key){
                _index_vola = -5;
                break;
            }else if (key == 'q' || key == 'w' || key == 'd' || key == 'x' || key == 'a' || key == 's'){// switch image direction
                destroyAllWindows();
                err_code = stop_transfer();
                RETURN_IF_ERR(err_code);
                reset_config();

                if (key == 'q') break;
                if (key == 'w') sensor_id = e_vbus1;
                if (key == 'd') sensor_id = e_vbus2;
                if (key == 'x') sensor_id = e_vbus3;
                if (key == 'a') sensor_id = e_vbus4;
                if (key == 's') sensor_id = e_vbus5;

                select_greyscale_image(sensor_id, true);
                select_greyscale_image(sensor_id, false);

                err_code = start_transfer();
                RETURN_IF_ERR(err_code);
                key = 0;
            }

//            cin>>key;
//            right << g_greyscale_image_left[which_cpy];

        }

//        key = waitKey(1);
//        which++;
//        if(which >= QUEUE_LEN)
//            which = 0;

//        sem_post(&g_empty);


    }
                  PRINTF_LOG("destroyAllWindows\n");
                destroyAllWindows();
    PRINTF_LOG("stop_transfe1\n");
    err_code = stop_transfer();
    RETURN_IF_ERR2( err_code );
    //make sure the ack packet from GUIDANCE is received
    usleep( 10000 );
    PRINTF_LOG("stop_transfe2\n");
    err_code = release_transfer();
    PRINTF_LOG("stop_transfe3\n");
    RETURN_IF_ERR2( err_code );
    PRINTF_LOG("stop_transfe4\n");
    return 0;
}


int ElasCom(const Elas &elas, uint8_t *I1, uint8_t *I2)
{

    struct timeval start, end;
    double timeuse;
    char key;
    //global
    struct obs_point obs_arr[320] = {0};
    vector<struct point> obstacle;



    gettimeofday(&start, NULL);
    elas.process(I1, I2);
    gettimeofday(&end, NULL);
    timeuse = 1000000* (end.tv_sec-start.tv_sec) + end.tv_usec-start.tv_usec;
    PRINTF_LOG("xxxxxxxxxxxxxxxxxxxxxxx elas process use : %fms\n", timeuse/1000);

    gettimeofday(&start, NULL);
    createObs(elas, obs_arr, &obstacle);
    gettimeofday(&end, NULL);
    timeuse = 1000000* (end.tv_sec-start.tv_sec) + end.tv_usec-start.tv_usec;
    PRINTF_LOG("createObs : %fms\n", timeuse/1000);

    gettimeofday(&start, NULL);
    DynamicWindowApproach( &obstacle[0], obstacle.size(), goal);
    gettimeofday(&end, NULL);
    timeuse = 1000000* (end.tv_sec-start.tv_sec) + end.tv_usec-start.tv_usec;
    PRINTF_LOG("DWA : %fms\n", timeuse/1000);
    return 1;
}


int DynamicWindowApproach(point* obstacle, int obs_nums,  point goal)
{
    //1. compute window
    //2. 模拟窗口内的运动轨迹 ,64条轨迹,存入traj向量
    //traj 存放64个移动4s后的飞行器位置
    GenerateTraj(obstacle, obs_nums);

//	PRINTF_LOG("NormalizeEval\n");
    // 各评价函数正则化
    NormalizeEval(goal);

    //3. 评价函数选择最优路线
    return  Evaluation();

    //4. move dt ms.初期设想，运行20ｍｓ
    // MoveDt();
}

int GenerateTraj(point* obstacle, int obs_nums)
{

    float Vx, Vy;
    float Px, Py;

//	PRINTF_LOG("predict point:\n");
    for(int i = 0; i < DIRECTIONS; i++) //DIRECTIONS= 8+1+8
    {
        Vx = abs(SpeedWindow[i].x);
        Vy = SpeedWindow[i].y;

        float obs_dis = 100;
        int flag = 0;
        //如果轨迹穿过障碍物，或很接近障碍物，则舍去该轨迹
        for(int t = 0; t < PREDICTT/DEL_DIS; t++) //t = (0, 11)
        {

            Px = Vx * (t * DEL_DIS);
            Py = Vy * (t * DEL_DIS);
            for(int k = 0; k < obs_nums; k++)
            {
                obs_dis = pow( pow(Px - obstacle[k].x, 2) + \
                        pow(Py - obstacle[k].y, 2), 0.5);

                if(obs_dis < SAFE_DIS ) //0.7
                {
//                    PRINTF_LOG("%d, %d, %d, (%f, %f), %f\n", i, t, k, \
//                           Px, Py, obs_dis);
                    flag = 1;
                    predict[i].x = 0;
                    predict[i].y = 0;
                    break;
                }
            }
            if(1 == flag)
                break;
        }
        if(1 == flag)
        {
//			PRINTF_LOG("xxxx\n");
            continue;
        }


        Px = Vx * PREDICTT;
        Py = Vy * PREDICTT;
        predict[i].x = Px;
        predict[i].y = Py;
        // predict[i].p.x = SpeedWindow[i].x * PREDICTT;
        // predict[i].p.y = SpeedWindow[i].y * PREDICTT;
//		PRINTF_LOG("%f, %f  \n",Px, Py);

    }
//	PRINTF_LOG("\n");
}

int NormalizeEval(point goal)
{
    for(int i = 0; i < DIRECTIONS; i ++)
    {
        if( 0 == predict[i].x && 0 == predict[i].y)
        {
//			PRINTF_LOG("xxxx\n");
            continue;
        }
        float del_x = goal.x-predict[i].x;
        if ( 0 == del_x)
        {
//			PRINTF_LOG("del_x == 0\n");
            while(1);
        }

        EvalDB[i].heading = (goal.y-predict[i].y) / (goal.x-predict[i].x)   ;
        EvalDB[i].dist = pow( pow(goal.x-predict[i].x, 2) + pow(goal.y-predict[i].y, 2), 0.5);
        EvalDB[i].Vx = SpeedWindow[i].x;

        if( 0 > EvalDB[i].heading)
            EvalDB[i].heading = -EvalDB[i].heading;
//		PRINTF_LOG(" %f\n", EvalDB[i].heading);
    }
    float heading_sum, dist_sum, Vx_sum;
    for(int i = 0; i < DIRECTIONS; i++)
    {
        if( 0 == predict[i].x && 0 == predict[i].y)
        {
            continue;
        }
        heading_sum += EvalDB[i].heading;
        dist_sum 	+= EvalDB[i].dist;
        Vx_sum		+= EvalDB[i].Vx;
    }
//	PRINTF_LOG("heading_sum: %f\n", heading_sum);
    // PRINTF_LOG("%f, %f, %f\n", heading_sum, dist_sum, Vx_sum);
    for(int i = 0; i < DIRECTIONS; i++)
    {
        if( 0 == predict[i].x && 0 == predict[i].y)
        {
            PRINTF_LOG("%d: xxxx\n", i);
            continue;
        }
        EvalDB_Nor[i].heading = EvalDB[i].heading / heading_sum;
        EvalDB_Nor[i].dist 	  = EvalDB[i].dist /dist_sum;
        EvalDB_Nor[i].Vx      = EvalDB[i].Vx / Vx_sum;
        PRINTF_LOG("%d: %f, %f  =  %f\n", i, EvalDB_Nor[i].heading, EvalDB_Nor[i].dist, \
               HEADING * EvalDB_Nor[i].heading + DIST * EvalDB_Nor[i].dist);
    }

}

int Evaluation()
{

    int tmp_index = -1;
    float eval = 100;
    /*
    for(int i = 0; i < DIRECTIONS; i++)
    {
        if( 0 == predict[i].x && 0 == predict[i].y)
            continue;
        float heading 	= EvalDB_Nor[i].heading;
        float dist 		= EvalDB_Nor[i].dist;
        float Vx 		= EvalDB_Nor[i].Vx;

//		float eval_tmp = HEADING * heading + DIST * dist + VEL * Vx;
        float eval_tmp = HEADING * heading + DIST * dist ;
        if(eval > eval_tmp)
        {
            eval = eval_tmp;
            tmp_index = i;
        }
    }
    */
    int optimize = 100;
/*
    for(int i = 0; i < DIRECTIONS; i++)
    {
        if( 0 == predict[i].x && 0 == predict[i].y)
            continue;
        if(abs(i - 8) < optimize) {
            optimize = abs(i - 8);
            tmp_index = i;
        }

    }
    GetOrSetIndex(1, tmp_index);
    */
    int best[DIRECTIONS];
    int best_ind = 0;
    for(int i = 0; i < DIRECTIONS; i++)
    {
        if( 0 == predict[i].x && 0 == predict[i].y)
            continue;
        best[best_ind] = i;
        best_ind++;
    }
    if(best_ind == 0)
        GetOrSetIndex(1, -1);
    else {
        GetOrSetIndex(1, best[(best_ind-1)/2]);
    }


//    PRINTF_LOG(" %d ", tmp_index);

//    sem_wait(&disTooSmall);
//    _index_vola = tmp_index;
//    sem_post(&disTooSmall);

    return 1;
}


void createObs(const Elas &elas, struct obs_point *obs_arr, \
               vector<struct point> *obstacle)
{
    struct timeval start, end;
    double timeuse;



    IplImage* img1f = cvCreateImage(cvSize(WIDTH, HEIGH), IPL_DEPTH_8U,1);
    IplImage* img2f = cvCreateImage(cvSize(WIDTH, HEIGH), IPL_DEPTH_8U,1);
    IplImage* img3f = cvCreateImage(cvSize(WIDTH, HEIGH), IPL_DEPTH_8U,1);

//    for(int32_t i = 100*HEIGH; i < 200*HEIGH; i++) {
//        cout << elas.cloud_c[i].z <<' ';
//        if(i % HEIGH == 0)
//            cout<<endl;
//    }

    for (int32_t i=0; i<WIDTH*HEIGH; i++){
        float dis_y = elas.cloud_c[i].y;
        if ( 1000 < dis_y)
            dis_y = 1000;
        if(dis_y <= -1000)
            dis_y = -1000;
        dis_y += 1000;
        dis_y /= 10;

            img1f->imageData[i] = (uint8_t)(dis_y);
    }


    //x axi is cloud_c.z
    for (int32_t i=0; i<WIDTH*HEIGH; i++){
        float dis = elas.cloud_c[i].z;
        if ( 10000 < dis)
            dis = 10000;
        if(dis <= 0)
            img2f->imageData[i] = 255;
        else
            img2f->imageData[i] = (uint8_t)(dis / 40);
    }
    gettimeofday(&start, NULL);

    //for(int v = 19; v < 220; v ++ )
    for(int v = 90; v < 130; v ++ )
    {
        for(int u = 3; u < 313; u ++)
        {
            int n = u + v * WIDTH;
            float dis_y = elas.cloud_c[n].y;
            float dis_z = elas.cloud_c[n].z;
            float dis_x = elas.cloud_c[n].x;
            img3f->imageData[n] = 255;

            if( dis_y > -200 && dis_y < 200 )   //gao du
            {
                if ( 10000 < dis_z)
                    dis_z = 10000;
                if(dis_z <= 0)
                    img3f->imageData[n] = 255;
                else
                    img3f->imageData[n] = (uint8_t)(dis_z / 40);
                if(dis_z < 6000 && dis_z > 0)
                {
                    //add this point to obstacle
                    struct obs_point *p = &obs_arr[u];

                    //if(124 == u)
                    //    PRINTF_LOG("124: (%f, %f) \n", dis_z, dis_x);
                    if(0 == p->y)
                    {
                        p->y = dis_x;
                    }else
                    {
                        p->y = (p->y + dis_x)/2;
                    }
                    if(0 == p->x )
                    {
                        p->x = dis_z;
                    }else
                    {
                        p->x = min(dis_z , p->x);
                    }
                    p->num++;
                }else
                    continue;

            }else
                continue;
        }

    }

//    gettimeofday(&end, NULL);
//    timeuse = 1000000* (end.tv_sec-start.tv_sec) + end.tv_usec-start.tv_usec;
//    PRINTF_LOG(" create Obs1---true : %fms\n", timeuse/1000);

//    gettimeofday(&start, NULL);
    //clear not use
    for(int i = 0; i < 320; i++)
    {
        if(obs_arr[i].num >= 5)
        {
//           PRINTF_LOG("(%f, %f) %d %d\n", obs_arr[i].x, obs_arr[i].y, obs_arr[i].num, i);

        }
        else //if(18 > obs_arr[i].num)
        {
            obs_arr[i].x = 0;
            obs_arr[i].y = 0;
            obs_arr[i].num = 0;
        }
    }

    for(int i = 5; i < 320 - 5; )
    {
        float base = obs_arr[i].x;
        int flag = 0;
        if(obs_arr[i].num == 0)
        {
            i ++;
            continue;
        }
        for(int j = i - 2; j < i + 2; j++)
        {
            float cha = abs(obs_arr[j].x - base);
            if(cha < 500)   //cha < 0.5m
            {
                flag++;
            }
        }
        if(flag > 3)        //flag =  4 or 5;
        {
            obs_arr[i].num = 1;
            struct point obsN;
            obsN.x = obs_arr[i].x / 1000;
            obsN.y = obs_arr[i].y / 1000;
            obstacle->push_back(obsN);
//show obstacle point
            PRINTF_LOG("{%f, %f},\n", obsN.x, obsN.y);
            i += 5;
        }
        else
        {
            obs_arr[i].num = 0;
            i ++;
        }
    }

    //struct point obsN;
    //obsN.x = 2;
    //obsN.y = 0;
    //obstacle.push_back(obsN);
//    gettimeofday(&end, NULL);
//    timeuse = 1000000* (end.tv_sec-start.tv_sec) + end.tv_usec-start.tv_usec;
//    PRINTF_LOG(" create Obs---true : %fms\n", timeuse/1000);

//    cvShowImage("y",img1f);
//    cvShowImage("consistency",img2f);
//    cvShowImage("high",img3f);
//    cvWaitKey(1);

}

void SetGoal(CoreAPI* api, float Xoff, float Yoff)
{
    PositionData curPosition = api->getBroadcastData().pos;
    PositionData goalPos = curPosition;
    goal_P.latitude = goalPos.latitude + Xoff / C_EARTH;
    goal_P.longitude = goalPos.longitude + Yoff / C_EARTH / cos(curPosition.latitude);

}

int index_num = 0;
int GetOrSetIndex(int flag, int index)
{
    sem_wait(&sem_Index);
    if(1 == flag){      //set index
        _index_vola = index;
        index_num = 0;
    }else if( 0 == flag) { //get index
        index_num++;
        if(index_num > 20){
		printf("index too long\n");
            index_num = 30;
            sem_post(&sem_Index);
            return -1;
        }
        sem_post(&sem_Index);
        return _index_vola;
    }
    sem_post(&sem_Index);
}
/*
int rel = 0;
int dis = 0;
int GetOrSetIndex(int flag, int rel, int dis)
{
    sem_wait(&sem_Index);
    if(1 == flag){      //set index
        _index_vola = index;
        index_num = 0;
    }else if( 0 == flag) { //get index
        index_num++;
        if(index_num > 10){
            index_num = 10;
            sem_post(&sem_Index);
            return -1;
        }
        sem_post(&sem_Index);
        return _index_vola;
    }
    sem_post(&sem_Index);
}
*/


int BitCount(unsigned int n)
{
    unsigned int c =0 ; // 计数器
    for (int i = 0; i < 10; i++)
    {
        if((n &1) ==1) // 当前位是1
            ++c ; // 计数器加1
        n >>=1 ; // 移位
    }
    return c ;
}
