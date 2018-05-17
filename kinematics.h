#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <string>
#include <iostream>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>



namespace HH
{
    using namespace KDL;

    enum Manip{ KR120, KR6 };


class Kinematics
{

public:
    Kinematics()
	: n_joints(0)
	{
	    // empty
	}
    
    Kinematics( std::vector<double> base, std::vector<double> tool, const Manip manipulator, const std::vector<double> &q_home )
	: Kinematics()
	{
	    switch( manipulator )
	    {
	    case KR120:
		setKR120(base, tool);
		break;
	    case KR6:
		setKR6(base, tool);
		break;
	    }

	    setHome( q_home );

	    for (int i = 0; i < n_joints; ++i)
		q_guess(i) = this->q_home.at(i);

	}

	
    ~Kinematics()
	{

	    if( fksolver != NULL )
		delete fksolver;
	    if( iksolver_v != NULL )
		delete iksolver_v;
	    if( iksolver != NULL )
		delete iksolver;
	}

//////////////////////////////////////////////////////////////
// return the lastest joint config
/////////////////////////////////////////////////////////////

    void getJoints( double jointAngles[] )
	{
	    for (int i = 0; i < n_joints; ++i)
	    {
		jointAngles[i] = q_dest(i);
	    }
	}

//////////////////////////////////////////////////////////////
// perform inverse kinematis 
/////////////////////////////////////////////////////////////
// pose is x,y,z in meters and ZYX euler angles A,B,C in radians
    bool ik( const double pose[6], double jointAngles[] )
	{
	    Frame T = Frame( Rotation::EulerZYX( pose[3],pose[4],pose[5]), Vector(pose[0],pose[1],pose[2] ) );

	    if( iksolver->CartToJnt( q_guess, T, q_dest ) != 0 )
		return false;

	    getJoints( jointAngles );

	    q_guess = q_dest;
	    
	    return true;

	}
//////////////////////////////////////////////////////////////
// perform forward kinematics on the latest joint config or a given one
/////////////////////////////////////////////////////////////

    bool fk( double pose[6], const double jointAngles[] = NULL )
	{
	    JntArray joints = q_guess;

	    if( jointAngles != NULL )
	    {
		for (int i = 0; i < n_joints; ++i)
		    joints(i) = jointAngles[i];
	    }

	    Frame T;

	    if( fksolver->JntToCart( joints, T )<0 )	    
		return false;

	    pose[0] = T.p.x();
	    pose[1] = T.p.y();
	    pose[2] = T.p.z();

	    T.M.GetEulerZYX( pose[3], pose[4], pose[5] );

	    
	    
	    return true;
	}



//////////////////////////////////////////////////////////////
// specify home joint config
/////////////////////////////////////////////////////////////

    void setHome( const std::vector<double> &q_home )
    {
	this->q_home.clear();

	for (int i = 0; i < n_joints; ++i)
	{
	    if( q_home.at(i)<q_lower_lim(i) || q_home.at(i)>q_upper_lim(i) )
	    {
		std::cerr << "Home joint angles (radians) exceed the joint limits\n";
		break;
	    }
	    else
		this->q_home.push_back( q_home.at(i) );
	}
    }


//////////////////////////////////////////////////////////////
// use kinamatics for kuka kr120
/////////////////////////////////////////////////////////////

    void setKR120( std::vector<double> base, std::vector<double> tool )
	{
	    m_chain = Chain();
	    
	    m_chain.addSegment(Segment("link1",Joint( "joint_a1", Vector(0,0,0.675), Vector(0,0,-1), Joint::RotAxis ), Frame( Vector( 0,0,0.675 ) ) ) );
	    m_chain.addSegment(Segment("link2",Joint( "joint_a2", Vector(0.35,0,0), Vector(0,1,0), Joint::RotAxis ), Frame( Vector( 0.35,0,0 ) ) ) );
	    m_chain.addSegment(Segment("link3",Joint( "joint_a3", Vector(1.15,0,0), Vector(0,1,0), Joint::RotAxis ), Frame( Vector( 1.15,0,0 ) ) ) );
	    m_chain.addSegment(Segment("link4",Joint( "joint_a4", Vector(1,0,-0.041), Vector(-1,0,0),  Joint::RotAxis ), Frame( Vector( 1,0,-0.041 ) ) ) );
	    m_chain.addSegment(Segment("link5",Joint( "joint_a5", Vector(0,0,0), Vector(0,1,0),  Joint::RotAxis ), Frame( Vector( 0,0,0 ) ) ) );
	    m_chain.addSegment(Segment("link6",Joint( "joint_a6", Vector(0,0,0), Vector(-1,0,0), Joint::RotAxis ), Frame( Vector( 0,0,0 ) ) ) );

	    const double lower[6] = {-3.22885911619, -2.70526034059, -2.26892802759, -6.10865238198, -2.26892802759, -6.10865238198};
	    const double upper[6] = {3.22885911619, 0.610865238198, 2.68780704807, 6.10865238198, 2.26892802759, 6.10865238198};

	    buildSolvers( lower, upper );

	}

//////////////////////////////////////////////////////////////
// use kinamatics for kuka kr6
/////////////////////////////////////////////////////////////
    
    void setKR6( std::vector<double> base, std::vector<double> tool )
	{
	    m_chain = Chain();
	    
	    m_chain.addSegment(Segment("link1",Joint( "joint_a1", Vector(0,0,0.4), Vector(0,0,-1), Joint::RotAxis ), Frame( Vector( 0,0,0.4 ) ) ) );
	    m_chain.addSegment(Segment("link2",Joint( "joint_a2", Vector(0.025,0,0), Vector(0,1,0), Joint::RotAxis ), Frame( Vector( 0.025,0,0 ) ) ) );
	    m_chain.addSegment(Segment("link3",Joint( "joint_a3", Vector(0.455,0,0), Vector(0,1,0), Joint::RotAxis ), Frame( Vector( 0.455,0,0 ) ) ) );
	    m_chain.addSegment(Segment("link4",Joint( "joint_a4", Vector(0,0,0.035), Vector(-1,0,0),  Joint::RotAxis ), Frame( Vector( 0,0,0.035 ) ) ) );
	    m_chain.addSegment(Segment("link5",Joint( "joint_a5", Vector(0.42,0,0), Vector(0,1,0),  Joint::RotAxis ), Frame( Vector( 0.42,0,0 ) ) ) );
	    m_chain.addSegment(Segment("link6",Joint( "joint_a6", Vector(0.08,0,0), Vector(-1,0,0), Joint::RotAxis ), Frame( Vector( 0.08,0,0 ) ) ) );

	    const double lower[6] = {-2.96705972839, -3.31612557879, -2.09439510239, -3.22885911619, -2.09439510239, -6.10865238198};
	    const double upper[6] = {2.96705972839, 0.785398163397, 2.72271363311, 3.22885911619, 2.09439510239, 6.10865238198};

	    buildSolvers( lower, upper );
	}

    

protected:

private:
    Chain m_chain;
    JntArray q_dest;
    JntArray q_guess;

    std::vector<double> q_home;

    int n_joints;
    JntArray q_lower_lim;
    JntArray q_upper_lim;

    ChainFkSolverPos_recursive *fksolver = NULL;//Forward position solver
    ChainIkSolverVel_pinv *iksolver_v = NULL;//Inverse velocity solver
    ChainIkSolverPos_NR_JL *iksolver = NULL;

    void buildSolvers( const double lower[], const double upper[])
	{
	    n_joints = m_chain.getNrOfJoints();
	    
	    q_lower_lim = JntArray(n_joints);
	    q_upper_lim = JntArray(n_joints);

	    q_dest = JntArray(n_joints);
	    q_guess = JntArray(n_joints);
	    
	    for (int i = 0; i < n_joints; ++i)
	    {
		q_lower_lim(i) = lower[i];
		q_upper_lim(i) = upper[i];
	    }

	    fksolver = new ChainFkSolverPos_recursive( m_chain );
	    iksolver_v = new ChainIkSolverVel_pinv( m_chain );
	    iksolver = new ChainIkSolverPos_NR_JL(m_chain, q_lower_lim, q_upper_lim, *fksolver,*iksolver_v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
	    
	}

};
    
}// namespace HH



#endif /* _KINEMATICS_H_ */
