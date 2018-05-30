/*
 * A header class to control KUKA manipulators via the RSI interface.
 * The instance will keep updating the controller at the required 250Hz,
 * but the wanted poses can be sent sporadically by using setPose.
 * Overload the function update() to choose how the intermidiate poses
 * are chosen between setPose updates.
 */

// Written in May 2018 by Håkon Hystad: hakonhystad@gmail.com

/*
 * WARNING:
 * - The implementation has a minimal amount of safe guards and should be thoroughly tested.
 */

#ifndef _RSI_H_
#define _RSI_H_



//////////////////////////////////////////////////////////////
// dependencies
/////////////////////////////////////////////////////////////


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <iostream>
#include <stdexcept>
#include <regex>
#include <thread>
#include <atomic>
#include <sys/time.h>
#include <mutex>
#include <vector>
#include <iomanip> // setprecision
#include <sstream> // stringstream
#include <chrono>
#include <ctime>
#include <ratio>

#include "kinematics.h"// NB! dependency: orocos kdl library installed

//////////////////////////////////////////////////////////////
// config
/////////////////////////////////////////////////////////////

#define _DEBUG_RSI_
#define _SOFT_STOP_HH_ 2// continues n seconds after ending to slow down


namespace HH
{
    
    const int MAXBUFLEN = 1024;

    const std::vector<double> home_axis_kr120 = {0.0, -1.5707963268, 1.5707963268, 0.0, 1.5707963268, 0};//3.1415926};

}


//////////////////////////////////////////////////////////////
// helpers
/////////////////////////////////////////////////////////////

// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}


namespace HH
{

//////////////////////////////////////////////////////////////
// class definition
/////////////////////////////////////////////////////////////

    /* 
     * NOTES:
     * - RSI::start will throw std::out_of_range if a UDP session can not be established 
     */


class RSI
{

public:
    // startPose can have any format as long as update(..) is overridden accordigly
    // base and tool are x,y,z in meters and A,B,C in ZYX euler angles (radians)
    RSI( std::vector<double> startPose, std::vector<double> q_home, std::vector<double> base, std::vector<double> tool, HH::Manip manipulator, std::string port )
	: m_port( port ),
	  m_signal( false ),
	  m_end( true ),
	  m_error( false ),
          m_ready(false),
	  m_kin( base, tool, manipulator, q_home ),
          m_home( q_home ),
	  m_pose( startPose )
	{
	    this->n_joints = q_home.size();
	}


	
    ~RSI()
	{
	    if( !m_end )
		end();
	    close(m_sockfd);
	}
    
//////////////////////////////////////////////////////////////
// start communication with the controller
/////////////////////////////////////////////////////////////

    
    void start()
	{
	    
	    if( m_end )
	    {
		m_end = false;
		m_signal = false;
		m_thread = std::thread( [this] { RSIthreadFnc(); } );
	    }
	    else
		std::cerr << "End communication before starting a new session\n";
	}

//////////////////////////////////////////////////////////////
// stop communicatin with the controller
/////////////////////////////////////////////////////////////
    
    void end()
	{
	    m_end = true;
	    m_thread.join();
	}

//////////////////////////////////////////////////////////////
// refresh the desired pose of the manipulator
/////////////////////////////////////////////////////////////


    bool setPose( std::vector<double> pose )
	{
	    std::lock_guard<std::mutex> lock(m_mtx);

	    if( pose.size() == m_pose.size() )
		m_pose = pose;
	    else
	    {
		std::cerr << "Invalid pose format\n";
		return false;
	    }

	    m_signal = true;

	    return !m_error;
	}

    bool isReady()
    {
      return m_ready;
    }


    

protected:

//////////////////////////////////////////////////////////////
// integrate or otherwise update the current pose when requested by the KUKA controller
/////////////////////////////////////////////////////////////
    
    // Overload to get a custom implementation.
    // The format of currentPose is optional, but newPose must follow x,y,z,A,B,C
    // where x,y,z are in meters and A,B,C are kuka Euler angles (ZYX) in radians.
    // Remember to handle the case if interval<some s which means currentPose has been very recently refreshed. And update currentPose
    virtual void update( double newPose[6], std::vector<double> &currentPose, double interval )
	{
	  //	    std::cout << "Using default update" << std::endl;
	    //  if( interval < 1e-6 )
	    {
		// here it is assumed that currentPose has the linear and angular velocities as well:
		// x,y,z,x_d,y_d,z_d,A,B,C,A_d,B_d,C_d
		// The newPose is simply the same as the given pose
		for(int i = 0; i < 3; ++i)
		    newPose[i] = currentPose[i];
		for(int i = 0; i < 3; ++i)
		    newPose[i+3] = currentPose[i+6];
		return;
	    }

	    
	}


//////////////////////////////////////////////////////////////
// The continued communication with the KUKA controller is in a separate thread
/////////////////////////////////////////////////////////////
    
    void RSIthreadFnc( )
	{

	    //////////////////////////////////////////////////////////////
	    // set up communication
	    if( !connect( m_port ) )
		throw std::out_of_range( "Connection not established\n" );


	    //////////////////////////////////////////////////////////////
	    // initialize
	    m_error = false;
	    m_end = false;

	    auto newTime = std::chrono::steady_clock::now();
	    auto oldTime = newTime;
	    
	    std::chrono::duration<double> duration;
	    
	    double *axis = new double[ this->n_joints ];
	    double *prev_axis = new double[ this->n_joints ];

	    m_kin.getJoints( axis );
	    /*
	    double test[6];
	    m_kin.fk(test);
	    for(int i = 0; i<6; i++)
	      std::cout << test[i] << " ";
	    std::cout << std::endl;
	    */
	    std::string ipoc;

	    std::unique_lock<std::mutex> lock(m_mtx);
	    std::vector<double> currentPose = m_pose;
	    lock.unlock();

	    //////////////////////////////////////////////////////////////
	    // main loop
	    while( !m_end )
	    {

		m_kin.getJoints( prev_axis  );
		//////////////////////////////////////////////////////////////
		// wait for until controller needs new update
		if( receive() < 1 )
		{
		    m_error = true;
		    break;
		}
		m_ready = true;
		
		//////////////////////////////////////////////////////////////
		// extract ACK signal to send back
		ipoc.clear();

		if( !extractTimestamp( ipoc ) )
		{
		    std::cerr << "IPOC error\n";
		    m_error = true;
		    break;
		}


		//////////////////////////////////////////////////////////////
		// refresh with new pose if one is given
		newTime = std::chrono::steady_clock::now();
		
		
		if( m_signal )
		{	        
		    lock.lock();
		    currentPose = m_pose;
		    m_signal = false;
		    oldTime = newTime;
		    lock.unlock();
		}

		//////////////////////////////////////////////////////////////
		// get intermidiate pose
		double sendPose[6];// x,y,z,A,B,C
		duration = std::chrono::duration_cast<std::chrono::seconds>(newTime - oldTime);
		update( sendPose, currentPose, duration.count() );
		oldTime = newTime;
				

#ifdef _DEBUG_RSI_
		std::cout << "Time since last update: " << duration.count() << std::endl;
		
		std::cout << "\nNew pose:\n";
		for (int i = 0; i < 6; ++i)
		    std::cout << sendPose[i] << " ";
		std::cout << "\n\n";
#endif
		
	        //////////////////////////////////////////////////////////////
		// perform inverse kinematics
		if( !m_kin.ik( sendPose, axis ) )
		{
		    std::cerr << "No solution to inverse kinematics\n";
		    m_error = true;
		    break;
		}

		for( int i = 0; i<this->n_joints; ++i )		  
		  axis[i] -= m_home[i];
		  
		//////////////////////////////////////////////////////////////
		// send the new pose to the controller
		packXML( axis, ipoc );// make xml string with joint angles and IPOC

		if( send(ipoc) != ipoc.length() )
		{
		    std::cerr << "Incomplete send\n";
		    m_error = true;
		    break;
		}


	    }// while not ended
#ifdef _DEBUG_RSI_
	    std::cout << "Ended RSI, slowing down\n";
#endif
	    m_ready = false;

	    //////////////////////////////////////////////////////////////
	    // slow down the joints to a soft stop

	    if( !m_error )
	    {

		const double period = duration.count();
		
		for(int i = 0; i < this->n_joints; ++i)
		{
		    // find out how fast the joint is moving ca
		    prev_axis[i] = ( axis[i] - prev_axis[i] )/period;// rad/s
		}
		const int tf = _SOFT_STOP_HH_/period;
		double *new_axis = new double[ this->n_joints ];
		std::cout << period << " " << tf << std::endl;
		for (int i = 0; i < tf; ++i)
		{
		    if( receive()<1 )
			break;
		    extractTimestamp( ipoc );
		    // ramp down speed and acceleration by a cubic polynomial
		    ramp( axis, prev_axis, i*period, _SOFT_STOP_HH_, new_axis );
		    std::cout << axis[0] << std::endl;
		    
		    packXML( new_axis, ipoc );// make xml string with joint angles and IPOC
		    send(ipoc);
		}
		delete [] new_axis;
	    }

	    delete [] axis;
	    delete [] prev_axis;

	    m_end = true;
	}

    void ramp( const double axis[], const double speed[], double t, double tf, double out[] )
	{
	    double r = t/tf;
	    for(int i = 0; i < this->n_joints; ++i)
		out[i] = speed[i]*t*( (1/3)*r*r - r + 1 ) + axis[i];
	}


private:

    int m_sockfd;
    std::string m_port;
    char m_buffer[MAXBUFLEN];
    struct sockaddr_storage m_their_addr;
    socklen_t m_addr_len;

    std::thread m_thread;
    std::atomic<bool> m_signal;
    std::atomic<bool> m_end;
    std::atomic<bool> m_error;
    std::atomic<bool> m_ready;

    std::mutex m_mtx;
    std::vector<double> m_pose;
    const std::vector<double> m_home;
    int n_joints;

    
    HH::Kinematics m_kin;

//////////////////////////////////////////////////////////////
// format the xml for sending 
/////////////////////////////////////////////////////////////


    void packXML( const double axis[], std::string &IPOC )
	{
	    std::stringstream stream;
	    stream << std::fixed << std::setprecision(4);

	    // header
	    stream << "<Sen Type=\"ImFree\">\n<AK ";
	    // body
	    for(int i = 1; i <= n_joints; ++i)
		stream << "A" << i << "=\"" << axis[i-1]*57.295779513 << "\" ";

	    stream << "/>\n";

	    // add ipoc, ack/timestamp
	    stream << IPOC;

	    stream << "\n</Sen>";
	    
	    IPOC = stream.str();
	}

//////////////////////////////////////////////////////////////
// get the ACK to send back
/////////////////////////////////////////////////////////////
    
    bool extractTimestamp( std::string &IPOC )
	{

	    std::regex rgx("<IPOC>\\d+<\\/IPOC>");
	    std::cmatch match;

	    
	    if( std::regex_search( m_buffer, match, rgx ) )	   
	    {
		IPOC = match[0];
		return true;
	    }
	    else
		return false;

	}

//////////////////////////////////////////////////////////////
// receive cstring from calling client via UDP
/////////////////////////////////////////////////////////////
    
    int receive()
	{
	    int numbytes;
	    m_addr_len = sizeof( m_their_addr );

	    if ((numbytes = recvfrom(m_sockfd, m_buffer, MAXBUFLEN-1 , 0, (struct sockaddr *) &m_their_addr, &m_addr_len)) == -1)
		perror("recvfrom");
	    m_buffer[numbytes] = '\0';


	    
#ifdef _DEBUG_RSI_
	    char s[INET6_ADDRSTRLEN];

	    
	    std::cout << "Got packet from " << inet_ntop(m_their_addr.ss_family, get_in_addr((struct sockaddr *)&m_their_addr), s, sizeof( s )) << std::endl;
  
	    std::cout << "It is " << numbytes << " bytes and contains\n";
	    std::cout << " \" " << m_buffer << " \" " << std::endl;
#endif

	    
	    return numbytes;


	}

//////////////////////////////////////////////////////////////
// send cstring to calling client via UDP
/////////////////////////////////////////////////////////////

    
    int send( const std::string &msg )
	{
	    int numbytes;

	    int len = msg.length();

	    if ((numbytes = sendto(m_sockfd, msg.c_str(), len, 0, (struct sockaddr *)&m_their_addr, m_addr_len)) == -1) 
		perror("sendto");

#ifdef _DEBUG_RSI_
	    std::cout << "sent " << numbytes << " of " << len << " bytes:\n \" "
		      << msg.substr(0,numbytes) << " \" " << std::endl;
#endif
        
	    return numbytes;
	}

//////////////////////////////////////////////////////////////
// set up a UDP server on specified port 
/////////////////////////////////////////////////////////////


    bool connect( std::string port )
	{
	    struct addrinfo hints, *servinfo, *p;
	    memset(&hints, 0, sizeof hints);
	    hints.ai_family = AF_UNSPEC; // set to AF_INET to force IPv4
	    hints.ai_socktype = SOCK_DGRAM;
	    hints.ai_flags = AI_PASSIVE; // use my IP

	    int rv;
	    if ((rv = getaddrinfo(NULL, port.c_str(), &hints, &servinfo)) != 0)
	    {
		std::cerr <<  "getaddrinfo: " << gai_strerror(rv) << std::endl;
		return false;
	    }

	    // loop through all the results and bind to the first we can
	    for(p = servinfo; p != NULL; p = p->ai_next)
	    {
		if ((m_sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1)
		{
		    perror("socket");
		    continue;
		}
		if (bind(m_sockfd, p->ai_addr, p->ai_addrlen) == -1) {
		    close(m_sockfd);
		    perror("bind");
		    continue;
		}

		break;
	    }

	    if (p == NULL)
	    {
		std::cerr << "failed to bind socket\n";
		return false;
	    }

#ifdef _DEBUG_RSI_
	    std::cout << "Server running on port " << port << std::endl;
#endif
	    
	    freeaddrinfo(servinfo);
	    return true;
	    
	}
    
};

    

}// namespace HH



#endif /* _RSI_H_ */
