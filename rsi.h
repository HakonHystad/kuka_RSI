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
#include <ctime>
#include <mutex>
#include <vector>
#include <iomanip> // setprecision
#include <sstream> // stringstream

//////////////////////////////////////////////////////////////
// config
/////////////////////////////////////////////////////////////

//#define MYPORT "4950"    // the port users will be connecting to
#define _DEBUG_RSI_
#define MAXBUFLEN 1024
#define POSE_SZ 12
#define N_AXIS 6

namespace HH
{
    // start pose: x,y,z,x_d,y_d,z_d,A,B,C,A_dot, B_dot, C_dot
    const double home[POSE_SZ] = {1.35, 0, 1.569, 0, 0, 0, 0, 0, -3.1416, 0, 0, 0};

    const double home_axis[N_AXIS] = {0.0, -90.0, 90.0, 0.0, 90.0, 180.0};
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
     * - Constructor will throw std::out_of_range if a UDP session can not be established 
     */


class RSI
{

public:
    RSI( std::string port )
	: m_port( port ),
	  m_signal( false ),
	  m_end( false ),
	  m_error( false )
	{
	    
	}


	
    ~RSI()
	{
	    if( !m_end )
		end();
	    close(m_sockfd);
	}

    void start()
	{
	    m_thread = std::thread( [this] { RSIthreadFnc(); } );
	}
    void end()
	{
	    m_end = true;
	    m_thread.join();
	}


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
  
	    std::cout << "It is " << numbytes << " bytes and contains\n\t";
	    std::cout << " \" " << m_buffer << " \" " << std::endl;
#endif

	    
	    return numbytes;


	}

    int send( const std::string &msg )
	{
	    int numbytes;

	    int len = msg.length();

	    if ((numbytes = sendto(m_sockfd, msg.c_str(), len, 0, (struct sockaddr *)&m_their_addr, m_addr_len)) == -1) 
		perror("sendto");

#ifdef _DEBUG_RSI_
	    std::cout << "sent " << numbytes << " of " << len << " bytes:\n\t \" "
		      << msg.substr(0,numbytes) << " \" " << std::endl;
#endif
        
	    return numbytes;
	}


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
		std::cerr << "listener: failed to bind socket\n";
		return false;
	    }

#ifdef _DEBUG_RSI_
	    std::cout << "Server running on port " << port << std::endl;
#endif
	    
	    freeaddrinfo(servinfo);
	    return true;
	    
	}


    void extractTimestamp( std::string &IPOC )
	{

	    std::regex rgx("<IPOC>\\d+<\\/IPOC>");
	    std::cmatch match;

	    
	    if( std::regex_search( m_buffer, match, rgx ) )	   
		IPOC = match[0];

	}

    bool setPose( std::vector<double> pose )
	{
	    std::lock_guard<std::mutex> lock(m_mtx);

	    for (int i = 0; i < POSE_SZ; ++i)
		m_pose[i] = pose.at(i);			    		    

	    m_signal = true;

	    return !m_error;
	}
    

protected:

    void RSIthreadFnc( )
	{

	    //////////////////////////////////////////////////////////////
	    // set up communication
	    if( !connect( m_port ) )
		throw std::out_of_range( "Connection not established\n" );


	    double newTime = (double)clock()/CLOCKS_PER_SEC;
	    double oldTime = newTime;

	    double currentPose[POSE_SZ];
	    for (int i = 0; i < POSE_SZ; ++i)
		currentPose[i] = home[i];

	    double axis[N_AXIS];
	    for (int i = 0; i < N_AXIS; ++i)
		axis[i] = home_axis[i];

	    
	    while( !m_end )
	    {
		// wait for new RSI message
		int nbytes = receive();
		
		newTime = (double)clock()/CLOCKS_PER_SEC;

		if( nbytes < 1 )
		{
		    m_error = true;
		    break;
		}



		// handle the IPOC
		std::string ipoc = "";
		extractTimestamp( ipoc );

		if( ipoc.empty() )
		{
		    std::cerr << "IPOC error\n";
		    m_error = true;
		    break;
		}


		// check if pose has been updated
		if( m_signal )
		{	        
		    std::lock_guard<std::mutex> lock(m_mtx);
		    for (int i = 0; i < POSE_SZ; ++i)
			currentPose[i] = m_pose[i];			    		    
		}

		// TODO: run update function to get an intermidiate pose

		// TODO perform optional interpolation

		// TODO perform inverse kinematics

		// send xml string with joint angles and IPOC
		packXML( axis, ipoc );
		send(ipoc);

		std::cout << "Time: " << newTime - oldTime << std::endl;
		oldTime = newTime;
	    }


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

    std::mutex m_mtx;
    double m_pose[POSE_SZ];

    void packXML( const double axis[N_AXIS], std::string &IPOC )
	{
	    std::stringstream stream;
	    stream << std::fixed << std::setprecision(4);

	    // header
	    stream << "<Sen Type=\"ImFree\">\n<AK ";
	    // body
	    for(int i = 1; i <= N_AXIS; ++i)
		stream << "A" << i << "=\"" << axis[i-1] << "\" ";

	    stream << "/>\n";

	    // add ipoc, ack/timestamp
	    stream << IPOC;

	    stream << "\n</Sen>";
	    
	    IPOC = stream.str();
	}


    
};

    

}// namespace HH



#endif /* _RSI_H_ */
