#include "rsi.h"

#include <iostream>
#include <vector>
#include <unistd.h>

int main( int argc, char** argv )
{
    HH::RSI rsi( "4950" );

    std::vector<double> start = {1.35092, 0, 1.784, 0, 0, 0, 0, -3.1415, 0, 0, 0, 0};

    
    rsi.setPose( start );
    
    rsi.start();

    sleep(2);

    double tmp = start.at(0);
    start.at(0) = start.at(1);
    start.at(1) = tmp;
    
    rsi.setPose( start );

    sleep(2);

    rsi.end();
    
    return 0;
}
