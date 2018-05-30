//#include "rsi.h"
#include "sphericalPendulum.h"

#include <iostream>
#include <vector>
#include <unistd.h>

#define N_STATES 12

void ramp( const std::vector<double> &pos1, const std::vector<double> &pos2, std::vector<double> &out, double t, double tf );

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

    std::vector<double> start = {1.35092, 0, 1.784, 0, 0, 0, 3.141592, 3.141592/2, 0, 0, 0, 0};
    auto end = start;
    end.at(1) += 0.5;
    
    auto home_axis = HH::home_axis_kr120;
    home_axis.at(0) -= 3.141592/2;
    
    HH::RSI rsi( start, home_axis, base, tool, HH::Manip::KR120, "4950" );
    
    rsi.start();
    sleep(5);
    
    auto pose = start;

    double timeOfPath = 5;// s

    // send in intervals of 0.1s
    for (int i = 0; i <= timeOfPath/0.1; ++i)
    {
	ramp( start, end, pose, i*0.1, timeOfPath );

	rsi.setPose( pose );
	sleep(0.1);
	std::cout << "set pose: " << pose[0] << " " << pose[1] << " " << pose[2] << std::endl;
    }

    rsi.end();

    sleep(5);
    
    return 0;
}

double cubic( double a0, double qf, double t, double tf )
{
    double r = t/tf;
    double q = a0-qf;
    // a1 = 0
    //double a2 = -3*( a0 -qf )/(tf*tf);
    //double a3 = 2*( a0 - qf )/(tf*tf*tf);
    //a3*t*t*t + a2*t*t + a1*t + a0;
    
    return (2*q*r*r*r -3*q*r*r + a0);
}


void ramp( const std::vector<double> &pos1, const std::vector<double> &pos2, std::vector<double> &out, double t, double tf )
{
     for (int i = 0; i < N_STATES; ++i)
     {
	 out[i] = cubic( pos1[i], pos2[i], t, tf );
     }

}
