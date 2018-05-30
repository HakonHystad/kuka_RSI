//#include "rsi.h"
#include "sphericalPendulum.h"

#include <iostream>
#include <vector>
#include <unistd.h>

#define N_STATES 12

void ramp( const std::vector<double> &pos1, const std::vector<double> &pos2, std::vector<double> &out, double t, double tf );

int main( int argc, char** argv )
{

  // default base
    std::vector<double> base = {0,0,0,0,0,0};
    std::vector<double> tool = {0,0,0,0,-3.141592/2,0};//{0,0,0,0,3.141592/2,0};
    /*
    std::vector<double> start = {1.35092, 0, 1.784, 0.01, 0, 0, -0.0015, 0.0274, 0.0549, 0.0873, -0.0873, 0};
    HH::SphericalPendulum rsi( start, HH::home_axis_kr120, base, tool, HH::Manip::KR120, "4950" );
*/

// new base
/*
    std::vector<double> base = {0,0,0,-3.141592/2,0,0};
    std::vector<double> tool = {0,0,0,0,0,0};
*/
    std::vector<double> start = {1.35, 0, 1.784, 0, 0, 0, 0, 0, 0, 0.017, 0, 0};//{1.350, 0, 1.568, 0, 0, 0, -3.141592, 0, -3.141592, 0, 0, 0};
    
    auto home_axis = HH::home_axis_kr120;

    
    HH::SphericalPendulum rsi( start, home_axis, base, tool, HH::Manip::KR120, "49001" );
    
    rsi.start();

    while( !rsi.isReady() )
      {}

    
    std::cout << "Ready!\n";

    
    
    sleep(5);
    rsi.end();

    sleep(1);
    
    return 0;
}
