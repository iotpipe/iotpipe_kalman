/*
Copyright (c) IoT Pipe 2016

This example makes use of a Kalman filter to filter a noise voltage signal.  It is a simple, one dimensional example of a Kalman Filter.  The 'true' voltage is -0.1V however the voltage meter has noisy measurements.  The Kalman filter will attempt to find the true voltage based on its understanding of the measurement noisiness.

Running this program will generate a plot, using GNUPlot, which shows the true voltage (black line), the measurements (green marks) and the estimated voltage (red line)

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

template<typename T>
using vec = boost::numeric::ublas::vector<T>;
template<typename T>
using mat = boost::numeric::ublas::matrix<T>;

using namespace Plotter;

const double true_voltage = 0.4;

int main()
{
	const int N = 1;
	WhiteNoiseSystem randomNoise;
	mat<double> A(N,N,1);
	mat<double> B(N,N,0);
	vec<double> u(N,0);
	mat<double> H(N,N,1);
	mat<double> Q(N,N,0.00001);
	mat<double> R(N,N,0.01);

	Kalman<double> kalman(N, A, R, Q, H, B, u);
	kalman.SetInitialState(vec<double>(N,0), mat<double>(N,N,1));

	std::vector<double> measurements, state, iteration,P;
	std::cout << "Iteration, Measurements, Covariances" << std::endl;

	vec<double> mn(1,0);
	
	mn(0) = true_voltage;
	kalman.SetMeasurement( mn );
	kalman.SetControlVector( u );
	kalman.Update();
	for(int i = 0; i < 50; i++)
	{
		mn = randomNoise.Measure(mn.size(),true_voltage,0.1);

		kalman.SetMeasurement( mn );
		kalman.SetControlVector( u );
		kalman.Update();

		state.push_back(kalman.GetState()[0]);
		measurements.push_back(kalman.GetMeasurement()[0]);
		P.push_back(kalman.GetCovariance()(0,0));
		iteration.push_back((double)i);
		
		std::cout << i << ", " << mn[0] << ", " << P[P.size()-1] << std::endl;
		
	}

	plot(iteration,measurements,"g2",iteration,state,"r-",std::vector<double>({0,49}), std::vector<double>({true_voltage,true_voltage}),"-");	
}
