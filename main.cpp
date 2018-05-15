#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
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
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
 
    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);
 
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
}
