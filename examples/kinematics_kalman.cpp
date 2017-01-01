/*
Copyright (c) IoT Pipe 2016


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		Some useful tutorials and info on Kalman Filters

http://www.richelbilderbeek.nl/CppKalmanFilterExample4.htm
https://www.cl.cam.ac.uk/~rmf25/papers/Understanding%20the%20Basis%20of%20the%20Kalman%20Filter.pdf
http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/                                   
http://www.cs.unc.edu/~tracker/media/pdf/SIGGRAPH2001_CoursePack_08.pdf
http://greg.czerniak.info/guides/kalman1/

*/

#include <iostream>
#include <stack>
#include <string>
#include <vector>

#include "kalman.h"
#include "Plotter.h"
#include "WhiteNoiseSystem.h"

#include <boost/numeric/ublas/assignment.hpp>

template<typename T>
using vec = boost::numeric::ublas::vector<T>;
template<typename T>
using mat = boost::numeric::ublas::matrix<T>;

using namespace Plotter;

const double dt = 1; //Time between measurements, given in seconds 
const double dt2 = dt*dt; //dt^2 for convenience
const double a_sigma = 0.1;
const double measurement_sigma = 0.1;
const int N = 2; //The cardinality of the space.  Here it is 2 because we have both x and x dot as coordinates
const int M = 2; //The cardinality of our measurements.  We only observe position, so it is 1.
const int L = 1; //The cardinality of our control vector.

int main()
{

	WhiteNoiseSystem whiteNoise;

	mat<double> A(N,N);
	mat<double> B(N,L);
	vec<double> u(L);
	mat<double> H(M,N);
	mat<double> Q(N,N); 
	mat<double> R(M,M);
	mat<double> P_initial(N,N);
	vec<double> x_initial(N);
	vec<double> z(M,0);

	A <<= 	1,dt,
			0,1;

	B <<= 	dt2/2, dt;

	u <<=	0;
	
	Q <<=	dt2*dt2/4, dt*dt2/2,
		dt*dt2/2, dt2;
	Q = Q*a_sigma*a_sigma;

	H <<= 1,0;

	R <<= 	measurement_sigma*measurement_sigma, 0,
			0, measurement_sigma*measurement_sigma;		
	
	P_initial <<= 	0,0,
					0,0;

	x_initial <<= 	0,0;	

	Kalman<double> kalman(N, A, R, Q, H, B, u);
	kalman.SetInitialState(x_initial,P_initial);

	std::vector<double> measurements, x_state, xdot_state, iteration,P, control_vector;

	//std::cout << "Iteration, Measurements, Covariances" << std::endl;
	for(int i = 0; i <50; i++)
	{
		z = whiteNoise.Measure(z.size(), 0, measurement_sigma);
		u = whiteNoise.Measure(u.size(), 1, a_sigma);

		kalman.SetMeasurementNoise( z );
		kalman.SetControlVector( u );
		kalman.Update();

		x_state.push_back(kalman.GetState()[0]);
		xdot_state.push_back(kalman.GetState()[1]);
		measurements.push_back(kalman.GetMeasurement()[0]);
		P.push_back(kalman.GetCovariance()(0,0));		
		control_vector.push_back(kalman.GetControlVector()[0]);
		iteration.push_back((double)i);

		//std::cout << i << ", " << z[0] << ", " << kalman.GetCovariance()(0,0) << std::endl;		
	}

	//Compute exact path for comparison_error
	std::vector<double> exact_position, exact_velocity;
	exact_position.push_back(0);
	exact_velocity.push_back(0);
	double pos=0,vel=0,acc;
	std::cout << pos << ", " << vel << "," << control_vector[0] << std::endl;
	for(int i = 1; i < control_vector.size(); i++)
	{	
		acc = control_vector[i-1];		
		vel += acc*dt;
		pos += vel*dt;
		
		
		std::cout << pos << ", " << vel << "," << control_vector[i] << std::endl;
		exact_position.push_back(pos);
		exact_velocity.push_back(vel);
	}

    plot(iteration,measurements,"r-",iteration,x_state,"g-", iteration,exact_position,"k-");
		
}