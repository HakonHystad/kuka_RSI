//#include "rsi.h"
#include "sphericalPendulum.h"

#include <iostream>
#include <vector>
#include <unistd.h>

int main( int argc, char** argv )
{

    std::vector<double> start = {1.35092, 0, 1.784, 0.01, 0, 0, -0.0015, 0.0274, 0.0549, 0.0873, -0.0873, 0};

    std::vector<double> base = {0,0,0,0,0,0};
    std::vector<double> tool = base;
    HH::SphericalPendulum rsi( start, HH::home_axis_kr120, base, tool, HH::Manip::KR120, "4950" );

    
    rsi.start();

    sleep(60);

    double tmp = start.at(0);
    start.at(0) = start.at(1);
    start.at(1) = tmp;
    
    rsi.setPose( start );

    sleep(2);

    rsi.end();
    
    return 0;
}
