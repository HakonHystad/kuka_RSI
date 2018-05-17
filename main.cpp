//#include "rsi.h"
#include "sphericalPendulum.h"

#include <iostream>
#include <vector>
#include <unistd.h>

int main( int argc, char** argv )
{

/* // default base
    std::vector<double> base = {0,0,0,0,0,0};
    std::vector<double> tool = {0,0,0,0,0,0};

    std::vector<double> start = {1.35092, 0, 1.784, 0.01, 0, 0, -0.0015, 0.0274, 0.0549, 0.0873, -0.0873, 0};
    HH::SphericalPendulum rsi( start, HH::home_axis_kr120, base, tool, HH::Manip::KR120, "4950" );
*/

// new base

    std::vector<double> base = {0,0,0,-3.141592/2,0,0};
    std::vector<double> tool = {0,0,0,0,0,0};

    std::vector<double> start = {1.35092, 0, 1.784, 0.01, 0, 0, 3.141592, 3.141592/2, 0, 0.0873, -0.0873, 0};
    auto home_axis = HH::home_axis_kr120;
    home_axis.at(0) -= 3.141592/2;
    HH::RSI rsi( start, home_axis, base, tool, HH::Manip::KR120, "4950" );
    
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
