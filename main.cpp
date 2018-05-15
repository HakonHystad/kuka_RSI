


#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

#include "rsi.h"


int main(int argc, char *argv[])
{
    HH::RSI rsi( "4950" );
    rsi.start();
/*
    std::string msg = "Testing 123";

    rsi.receive();

    std::string ipoc = "";
    std::cout << rsi.extractTimestamp( ipoc ) << " = " << ipoc << std::endl;

    rsi.send(msg);
*/
    sleep(5);


    std::vector<double> pose;
    for (int i = 0; i < POSE_SZ; ++i)
	pose.push_back(i);

    rsi.setPose( pose );

    rsi.end();
    
    return 0;
}
