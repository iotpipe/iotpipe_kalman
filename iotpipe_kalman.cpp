//g++ -std=c++14 iotpipe_kalman.cpp -lboost_iostreams -lboost_system -lboost_filesystem -o kalman

#include <iostream>
#include <vector>
#include <stack>
#include <string>

#include "kalman.h"
#include "Plotter.h"

#include <boost/math/distributions/normal.hpp>
#include <boost/random.hpp>

template<typename T>
using vec = boost::numeric::ublas::vector<T>;
template<typename T>
using mat = boost::numeric::ublas::matrix<T>;

using namespace Plotter;

int main()
{
	mat<double> A(1,1,1);
	mat<double> B(1,1,0);
	vec<double> u(1,0);
	mat<double> H(1,1,1);
	mat<double> Q(1,1,0.00001); 
	mat<double> R(1,1,0.01);

	Kalman<double> kalman(1, A, R, Q, H, B, u);
	kalman.SetInitialState(vec<double>(1,1), mat<double>(1,1,1));


	mat<double> G(1,1,1);
	mat<double> HH = trans(G);
	const mat<double>& HHH(trans(G));

	//Filter Loop
	typedef boost::mt19937 RNGType;
	RNGType gen;
	boost::normal_distribution<double> nd(-0.37727,0.1);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > measurement(gen, nd);
	vec<double> z(1,0);
	std::vector<double> measurements, state, iteration;
	for(int i = 0; i < 500; i++)
	{
		z[0] = measurement();

		kalman.SendMeasurement( z );

		state.push_back(kalman.GetState()[0]);

		measurements.push_back(z[0]);
		iteration.push_back((double)i);
	}

	plot(iteration,measurements,"g2",iteration,state,"r2",std::vector<double>({0,499}), std::vector<double>({-0.37727,-0.37727}),"-");
	
}

//http://www.richelbilderbeek.nl/CppKalmanFilterExample4.htm
//https://www.cl.cam.ac.uk/~rmf25/papers/Understanding%20the%20Basis%20of%20the%20Kalman%20Filter.pdf
//http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
//http://www.cs.unc.edu/~tracker/media/pdf/SIGGRAPH2001_CoursePack_08.pdf
//http://greg.czerniak.info/guides/kalman1/