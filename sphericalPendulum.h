#ifndef _SPHERICALPENDULUM_H_
#define _SPHERICALPENDULUM_H_

#include "rsi.h"
#include <cmath>
#include <valarray>


namespace PARAMETER
{
    const double LENGTH = 1;// temp
}


namespace HH
{

class SphericalPendulum : public RSI
{

public:
    SphericalPendulum( std::string port )
	: RSI( port )
	{}
    ~SphericalPendulum()
	{}

    void update( double newPose[6], double currentPose[POSE_SZ], double interval )
	{
	    double stepSz = 1e-4;
	    //////////////////////////////////////////////////////////////
	    // if new pose was given recently, just resend is
	    if( interval < stepSz )
		goto done;

	    //////////////////////////////////////////////////////////////
	    // else integrate previous pose with the dynamic equations

	    std::valarray<double> Y(POSE_SZ);

	    // initial values
	    for (int i = 0; i < POSE_SZ; ++i)
		Y[i] = currentPose[i];

	    int n_steps = interval/stepSz;

	    double x = 0;
	    for (int i = 0; i < n_steps; ++i)
	    {
		step( stepSz, x, Y ); 
	    }

	    for (int i = 0; i < POSE_SZ; ++i)
	    {
		currentPose[i] = Y[i];

		#ifdef _DEBUG_RSI_
		std::cout << currentPose[i] << " ";
		#endif
	    }
	    #ifdef _DEBUG_RSI_
	    std::cout << std::endl;
	    #endif
	done:
	    
	    for(int i = 0; i < 3; ++i)
		newPose[i] = currentPose[i];

	    XYZ_to_ZYX( &currentPose[6], &newPose[3] );
	}

    void XYZ_to_ZYX( const double in[3], double out[3] )
	{
	    double s1 = sin( in[0] );
	    double c1 = cos( in[0] );

	    double s2 = sin( in[1] );
	    double c2 = cos( in[1] );

	    double s3 = sin( in[2] );
	    double c3 = cos( in[2] );

	    double &A = out[0];
	    double &B = out[1];
	    double &C = out[2];

	    A = atan2( c1*s3+c3*s1*s2, c2*c3 );
	    double sA = sin( A );
	    double cA = cos( A );
	    
	    B = atan2( c1*c3*s2-s1*s3, c2*c3*cA + ( c1*s3 + c3*s1*s2 )*sA );
	    C = atan2( s2*sA+c2*s1*cA, (c1*c3-s1*s2*s3)*cA + c2*s3*sA );
	}

	    


protected:

private:

//////////////////////////////////////////////////////////////
// RK 4 step
/////////////////////////////////////////////////////////////
    // ndep= number of variables, dx= stepsize, x = fraction of interval, Y=input/output, F=derivative function
    void step( double dx, double &x, std::valarray<double> &Y )
	{
	    int ndep = Y.size();
	    std::valarray<double> dY1(ndep), dY2(ndep), dY3(ndep), dY4(ndep);

	    dY1 = F( x           , Y             ) * dx;
	    dY2 = F( x + 0.5 * dx, Y + 0.5 * dY1 ) * dx;
	    dY3 = F( x + 0.5 * dx, Y + 0.5 * dY2 ) * dx;
	    dY4 = F( x +       dx, Y       + dY3 ) * dx;
	    Y += ( dY1 + 2.0 * dY2 + 2.0 * dY3 + dY4 ) / 6.0;

	    x += dx;
	}

//////////////////////////////////////////////////////////////
// derivative function 
/////////////////////////////////////////////////////////////

        std::valarray<double> F( double x, std::valarray<double> Y )
	{
	    std::valarray<double> f( Y.size() );

#define LINEAR_PARAMETERS Y[0],Y[1],Y[2],Y[3],Y[4],Y[5]// (double x, double y, double z, double x_dot, double y_dot, double z_dot)
#define ANGULAR_PARAMETERS Y[6],Y[7],Y[8],Y[9],Y[10],Y[11]//(double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot)

	    f[0] = dynamicsX( LINEAR_PARAMETERS );
	    f[1] = dynamicsY( LINEAR_PARAMETERS );
	    f[2] = dynamicsY( LINEAR_PARAMETERS );

	    f[3] = dynamicsX_d( LINEAR_PARAMETERS );
	    f[4] = dynamicsY_d( LINEAR_PARAMETERS );
	    f[5] = dynamicsY_d( LINEAR_PARAMETERS );

	    f[6] = dynamicsAlpha( ANGULAR_PARAMETERS );
	    f[7] = dynamicsBeta( ANGULAR_PARAMETERS );
	    f[8] = dynamicsGamma( ANGULAR_PARAMETERS );

	    f[9] = dynamicsAlpha_d( ANGULAR_PARAMETERS );
	    f[10] = dynamicsBeta_d( ANGULAR_PARAMETERS );
	    f[11] = dynamicsGamma_d( ANGULAR_PARAMETERS );
	    
	    return f;
	}


    inline double dynamicsX( double x, double y, double z, double x_dot, double y_dot, double z_dot )
	{
	    return x_dot;
	}

    inline double dynamicsY( double x, double y, double z, double x_dot, double y_dot, double z_dot )
	{
	    return y_dot;
	}

    inline double dynamicsZ( double x, double y, double z, double x_dot, double y_dot, double z_dot )
	{
	    return z_dot;
	}

    inline double dynamicsX_d( double x, double y, double z, double x_dot, double y_dot, double z_dot )
	{
	    return 0;
	}

    inline double dynamicsY_d( double x, double y, double z, double x_dot, double y_dot, double z_dot )
	{
	    return 0;
	}

    inline double dynamicsZ_d( double x, double y, double z, double x_dot, double y_dot, double z_dot )
	{
	    return 0;
	}


    inline double dynamicsAlpha( double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot )
	{
	    return alpha_dot;
	}

    inline double dynamicsBeta( double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot )
	{
	    return beta_dot;
	}
    
    inline double dynamicsGamma( double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot )
	{
	    return gamma_dot;
	}


    // NB: uses XYZ euler angles in radians
    inline double dynamicsAlpha_d( double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot )
	{
	    double sinBeta = sin(beta),  cosBeta = cos(beta);
	    return ( alpha_dot*beta_dot*sinBeta - (9.81/PARAMETER::LENGTH)*sin(alpha) )/cosBeta;
	}

    inline double dynamicsBeta_d( double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot )
	{
	    double sinBeta = sin(beta),  cosBeta = cos(beta);
	    return -( 9.81/PARAMETER::LENGTH + alpha_dot*alpha_dot )*sinBeta*cosBeta;
	}

    inline double dynamicsGamma_d( double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot )
	{
	    return 0;
	}

};
    
}// namespace




#endif /* _SPHERICALPENDULUM_H_ */
