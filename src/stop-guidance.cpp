#include "main.h"



int main(int argc, char** argv)
{


    printf("stop_transfe1\n");
    int err_code = stop_transfer();
    RETURN_IF_ERR2( err_code );
    //make sure the ack packet from GUIDANCE is received
    usleep( 10000 );
    printf("stop_transfe2\n");
    err_code = release_transfer();
    printf("stop_transfe3\n");
    RETURN_IF_ERR2( err_code );
    printf("stop_transfe4\n");
    return 0;

}
