#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <stdio.h>
#include <iostream>
 
using namespace KDL;
 
 
int main( int argc, char** argv )
{
    //Definition of a kinematic chain & add segments to the chain
    KDL::Chain chain;
/*
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
    chain.addSegment(Segment(Joint(Joint::RotZ)));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
    chain.addSegment(Segment(Joint(Joint::RotZ)));
*/
    chain.addSegment(Segment("link1",Joint( "joint_a1", Vector(0,0,0.675), Vector(0,0,-1), Joint::RotAxis ), Frame( Vector( 0,0,0.675 ) ) ) );
    chain.addSegment(Segment("link2",Joint( "joint_a2", Vector(0.35,0,0), Vector(0,1,0), Joint::RotAxis ), Frame( Vector( 0.35,0,0 ) ) ) );
    chain.addSegment(Segment("link3",Joint( "joint_a3", Vector(1.15,0,0), Vector(0,1,0), Joint::RotAxis ), Frame( Vector( 1.15,0,0 ) ) ) );
    chain.addSegment(Segment("link4",Joint( "joint_a4", Vector(1,0,-0.041), Vector(-1,0,0),  Joint::RotAxis ), Frame( Vector( 1,0,-0.041 ) ) ) );
    chain.addSegment(Segment("link5",Joint( "joint_a5", Vector(0,0,0), Vector(0,1,0),  Joint::RotAxis ), Frame( Vector( 0,0,0 ) ) ) );
    chain.addSegment(Segment("link6",Joint( "joint_a6", Vector(0,0,0), Vector(-1,0,0), Joint::RotAxis ), Frame( Vector( 0,0,0 ) ) ) );
    // Create solver based on kinematic chain
//    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    int n_joints = chain.getNrOfJoints();
    JntArray q_lower( n_joints );
    JntArray q_upper( n_joints );

    const double lower[] = {-3.22885911619, -2.70526034059, -2.26892802759, -6.10865238198, -2.26892802759, -6.10865238198};
    const double upper[] = {3.22885911619, 0.610865238198, 2.68780704807, 6.10865238198, 2.26892802759, 6.10865238198};

    for (int i = 0; i < n_joints; ++i)
    {
	q_lower(i) = lower[i];
	q_upper(i) = upper[i];
    }

    
    //Creation of the solvers:
    ChainFkSolverPos_recursive fksolver(chain);//Forward position solver
    ChainIkSolverVel_pinv iksolver_v(chain);//Inverse velocity solver
    ChainIkSolverPos_NR_JL iksolver(chain, q_lower, q_upper, fksolver,iksolver_v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

    //Creation of jntarrays:
    JntArray q_dest(chain.getNrOfJoints());
    JntArray q_guess(chain.getNrOfJoints());

    const double home_axis[] = {0.0, -90.0, 90.0, 0.0, 90.0, 180.0};
    for( int i=0; i<6; i++ )
	q_guess(i) = (home_axis[i] + 5)*(3.141592/180);

    Frame T = Frame( Rotation::EulerZYX(0,0,-3.141592), Vector(1.35,0,1.78) );

    int ret = iksolver.CartToJnt( q_guess, T, q_dest );

    for (int i = 0; i < 6; ++i)
    {
	std::cout << q_dest(i) << " ";
    }
    std::cout << std::endl;

/*
    // Assign some values to the joint positions
    for(unsigned int i=0;i<nj;i++){
        float myinput;
        printf ("Enter the position of joint %i: ",i);
        scanf ("%e",&myinput);
        jointpositions(i)=(double)myinput;
    }
 
    // Create the frame that will contain the results
    KDL::Frame cartpos;    
 
    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos <<std::endl;
        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }
    */
}
