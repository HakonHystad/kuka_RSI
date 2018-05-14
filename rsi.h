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

//////////////////////////////////////////////////////////////
// config
/////////////////////////////////////////////////////////////

//#define MYPORT "4950"    // the port users will be connecting to
#define _DEBUG_RSI_
#define MAXBUFLEN 1024

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
	: m_port( port )
	{
	    //////////////////////////////////////////////////////////////
	    // set up communication
	    if( !connect( m_port ) )
		throw std::out_of_range( "Connection not established\n" );

	    
	    
	}


	
    ~RSI()
	{
	    close(m_sockfd);
	}


    int receive()
	{
	    int numbytes;
	    m_addr_len = sizeof( m_their_addr );

	    if ((numbytes = recvfrom(m_sockfd, m_buffer, MAXBUFLEN-1 , 0, (struct sockaddr *) &m_their_addr, &m_addr_len)) == -1)
		perror("recvfrom");

	    
#ifdef _DEBUG_RSI_
	    char s[INET6_ADDRSTRLEN];

	    
	    std::cout << "Got packet from " << inet_ntop(m_their_addr.ss_family, get_in_addr((struct sockaddr *)&m_their_addr), s, sizeof( s )) << std::endl;
  
	    std::cout << "It is " << numbytes << " bytes and contains\n\t";
	    m_buffer[numbytes] = '\0';
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


    

protected:

private:

    int m_sockfd;
    std::string m_port;
    char m_buffer[MAXBUFLEN];
    struct sockaddr_storage m_their_addr;
    socklen_t m_addr_len;

    int extractTimestamp( std::string IPOC )
	{
	    
	}

};

}// namespace HH



#endif /* _RSI_H_ */
