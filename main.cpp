//#include "rsi.h"


#include <iostream>
#include <vector>
#include <unistd.h>

#define N_STATES 12
namespace PARAMETER
{
    const float LENGTH = 0.9;// meter
    const float THETA_ALPHA = 0;// rad
    const float THETA_BETA = 0;// rad
}

#include "sphericalPendulum.h"



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
    std::vector<double> start = {1.35, 0, 1.784, 0, 0.01, 0, 0, 0, 0, 0, 0, 0};//{1.350, 0, 1.568, 0, 0, 0, -3.141592, 0, -3.141592, 0, 0, 0};
    
    auto home_axis = HH::home_axis_kr120;
    home_axis[0] = 42*(3.141592/180);
    home_axis[0] = -95*(3.141592/180);
    home_axis[0] = 136*(3.141592/180);
    home_axis[0] = -62*(3.141592/180);
    home_axis[0] = -62*(3.141592/180);
    home_axis[0] = 112*(3.141592/180);
    

    
    HH::SphericalPendulum rsi( start, home_axis, base, tool, HH::Manip::KR120, "49001" );

    
    // add offset to local ref
    
    rsi[0] = -0.8;// x
    rsi[1] = 0;// y
    rsi[2] = -0.65;// z
    rsi[3] = 0;// alpha
    rsi[4] = 3.141592/2;// beta
    rsi[5] = 0;// gamma
    
    
    rsi.start();

    while( !rsi.isReady() )
      {}

    
    std::cout << "Ready!\n";

    
    
    sleep(15);
    rsi.end();
    std::cout << "Done!\n";
    sleep(5);
    
    return 0;
}
