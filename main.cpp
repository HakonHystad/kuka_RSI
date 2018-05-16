#include "kinematics.h"

#include <vector>
#include <iostream>
 
int main( int argc, char** argv )
{

    std::vector<double> q_home = { 0, -1.57, 1.57, 0, 1.57, 3.1415 };
    
    
    HH::Kinematics kin( HH::KR120, q_home );

    double fk_pose[6];
    double pose[6] = {1.35, 0, 1.569,  0, 0, -3.1416};
    double q[6];

//////////////////////////////////////////////////////////////
// Forward kinematics
/////////////////////////////////////////////////////////////

    double jointAngles[6] = { 0, -1.57, 1.57, 0, 1.57, 3.1415 };
    if( !kin.fk( fk_pose/*, jointAngles*/ ) )
	return 1;

    std::cout << "Forward kinematics:\n";
    for (int i = 0; i < 6; ++i)
	std::cout << fk_pose[i] << " ";
    std::cout << std::endl;

//////////////////////////////////////////////////////////////
// Inverse kinematics
/////////////////////////////////////////////////////////////

    
    if( !kin.ik( pose, q ) )
	return 1;

    std::cout << "Inverse kinematics:\n";
    for (int i = 0; i < 6; ++i)
    {
	std::cout << q[i] << " ";
    }
    std::cout << std::endl;

    return 0;
}
