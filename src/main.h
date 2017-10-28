#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <ctype.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <unistd.h>
#include <time.h>
#include <sys/times.h>

/* Include gettid() */
#include <sys/types.h>

/* Include the LITMUS^RT API.*/
//#include "litmus.h"

/* Include Elas head*/
#include "elas.h"

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
#include <DJI_Type.h>

using namespace std;

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "opencv/cv.h"
#include "opencv/cxmisc.h"
#include "opencv/highgui.h"
#include "opencv/cvaux.h"

#define PRI_CONTROL         10
#define PRI_CALLBACK        20
#define PRI_IMAGEPROCESS    40
#define PRI_READ            30

#define PERIOD_CONTROL          10
#define PERIOD_CALLBACK         15
#define PERIOD_IMAGEPROCESS     100
#define PERIOD_READ             20

#define RELATIVE_DEADLINE 100

#define EXEC_COST_CONTROL        2
#define EXEC_COST_CALLBACK       10
#define EXEC_COST_IMAGEPROCESS   30
#define EXEC_COST_READ           1

#define migrate_CPU_CONTROl         0
#define migrate_CPU_CALLBACK        1
#define migrate_CPU_IMAGEPROCESS    2
#define migrate_CPU_READ            3

#define WIDTH 320
#define HEIGHT 240
#define HEIGH 240

#define IMAGE_SIZE (HEIGHT * WIDTH)
#define QUEUE_LEN 10

#define USE_GUIDANCE_ASSISTANT_CONFIG 0 //use GUIDANCE ASSISTANT's configure
#define SELECT_DEPTH_DATA 1

using namespace cv;
Mat     g_greyscale_image_left[QUEUE_LEN];
Mat		g_greyscale_image_right[QUEUE_LEN];
Mat		g_depth;
Mat		g_disparity;


e_vbus_index sensor_id = e_vbus1;




DJI_lock    g_lock;
DJI_event   g_event;
sem_t   g_empty;
sem_t   g_full;
sem_t   disTooSmall;
sem_t   sem_Index;

DJI_lock    lock_which; //enter, leave
int volatile which = 0;
int volatile which_old = 0;
char		key = 0;


#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<\
    __FILE__<<std::endl; return -1;}}
#define RETURN_IF_ERR2(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return NULL;}}

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


/***
 * ELAS main.h
 * */

typedef struct
{
    float theta;
    float x;
    float y;
}speed;

struct obs_point
{
    float x;
    float y;
    int num;
};

struct point
{
    float x;
    float y;
};
typedef struct point point;


typedef struct
{
    point p;
}PointStat;

typedef struct
{
    float heading;
    float dist;
    float Vx;

}eval_t;


int DynamicWindowApproach(point* obstacle, int obs_nums,  point goal);
int GenerateTraj(point* obstacle, int obs_nums);
int NormalizeEval(point goal);
int Evaluation();

//制动距离
//65个方向
//预测四秒后的位置
#define STOPDIST 1
#define DIRECTIONS 17
#define PREDICTT 7      //predict time 6s;

#define DEL_DIS		0.5
#define SAFE_DIS	0.5    //M100: width 0.51; 1m is too far
//use ./picture3/left18 will no path

//eval func param
#define HEADING 0.05
#define DIST	0.2
#define VEL		0.1

int volatile _index_vola = -1;

//M100 speed is 0.5m/s
speed SpeedWindow[] =
{

{58, -0.169610, 0.105984},
{62, -0.176590, 0.093894},
{66, -0.182709, 0.081347},
{70, -0.187939, 0.068404},
{74, -0.192252, 0.055127},
{78, -0.195630, 0.041582},
{82, -0.198054, 0.027835},
{86, -0.199513, 0.013951},
{90, -0.200000, 0.000000},
{94, -0.199513, -0.013951},
{98, -0.198054, -0.027835},
{102, -0.195630, -0.041582},
{106, -0.192252, -0.055127},
{110, -0.187939, -0.068404},
{114, -0.182709, -0.081347},
{118, -0.176590, -0.093894},
{122, -0.169610, -0.105984}
/*
    {58, -0.42402, 0.26497}, // 0
    {62, -0.44147, 0.23475}, // 1
    {66, -0.45677, 0.20338}, // 2
    {70, -0.46984, 0.17103}, // 3
    {74, -0.48063, 0.13784}, // 4
    {78, -0.48907, 0.10398}, // 5
    {82, -0.49513, 0.06961}, // 6
    {86, -0.49878, 0.03490}, // 7
    {90, -0.50000, 0.00000}, // 8
    {94, -0.49878, -0.03485}, // 9
    {98, -0.49514, -0.06956}, // 10
    {102, -0.48908, -0.10393}, // 11
    {106, -0.48064, -0.13779}, // 12
    {110, -0.46986, -0.17098}, // 13
    {114, -0.45678, -0.20334}, // 14
    {118, -0.44149, -0.23471}, // 15
    {122, -0.42404, -0.26493} // 16
*/
};

speed SpeedWindowBack[] =
{
    {58, -0.42402, -0.26497}, // 0
    {62, -0.44147, -0.23475}, // 1
    {66, -0.45677, -0.20338}, // 2
    {70, -0.46984, -0.17103}, // 3
    {74, -0.48063, -0.13784}, // 4
    {78, -0.48907, -0.10398}, // 5
    {82, -0.49513, -0.06961}, // 6
    {86, -0.49878, -0.03490}, // 7
    {90, -0.50000,-0.00000}, // 8
    {94, -0.49878, 0.03485}, // 9
    {98, -0.49514, 0.06956}, // 10
    {102, -0.48908, 0.10393}, // 11
    {106, -0.48064, 0.13779}, // 12
    {110, -0.46986, 0.17098}, // 13
    {114, -0.45678, 0.20334}, // 14
    {118, -0.44149, 0.23471}, // 15
    {122, -0.42404, 0.26493} // 16

};

point predict[DIRECTIONS];
eval_t EvalDB[DIRECTIONS];
eval_t EvalDB_Nor[DIRECTIONS];


void createObs(const Elas &elas, struct obs_point *obs_arr, \
               vector<struct point> *obstacle);

extern void* HostMal(void **p, long size);
extern void initCudaMalloc();
extern void SetDeviceMap();
extern void allocFreeCount();



/***
 * over: ELAS main.h
 * */
