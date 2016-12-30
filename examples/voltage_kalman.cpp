//g++ -std=c++14 iotpipe_kalman.cpp -lboost_iostreams -lboost_system -lboost_filesystem -o kalman
//http://www.richelbilderbeek.nl/CppKalmanFilterExample4.htm
//https://www.cl.cam.ac.uk/~rmf25/papers/Understanding%20the%20Basis%20of%20the%20Kalman%20Filter.pdf
//http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
//http://www.cs.unc.edu/~tracker/media/pdf/SIGGRAPH2001_CoursePack_08.pdf
//http://greg.czerniak.info/guides/kalman1/

#include <chrono>
#include <iostream>
#include <stack>
#include <string>
#include <vector>

#include "kalman.h"
#include "Plotter.h"

#include <boost/math/distributions/normal.hpp>
#include <boost/random.hpp>

template<typename T>
using vec = boost::numeric::ublas::vector<T>;
template<typename T>
using mat = boost::numeric::ublas::matrix<T>;

using namespace Plotter;
using namespace std::chrono;

int main()
{
	const int N = 1;
	mat<double> A(N,N,1);
	mat<double> B(N,N,0);
	vec<double> u(N,0);
	mat<double> H(N,N,1);
	mat<double> Q(N,N,0.00001); 
	mat<double> R(N,N,0.01);

	Kalman<double> kalman(1, A, R, Q, H, B, u);
	kalman.SetInitialState(vec<double>(N,0), mat<double>(N,N,1));

	mat<double> G(1,1,1);
	mat<double> HH = trans(G);
	const mat<double>& HHH(trans(G));

	//Filter Loop
	milliseconds ms = duration_cast< milliseconds >(
	    system_clock::now().time_since_epoch()
	);

	typedef boost::mt19937 RNGType;
	RNGType gen(ms.count());
	boost::normal_distribution<double> nd(-0.37727,0.1);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > measurement(gen, nd);
	vec<double> z(1,0);
	std::vector<double> measurements, state, iteration,P;
	for(int i = 0; i < 50; i++)
	{
		z[0] = measurement();
		kalman.SendMeasurement( z );

		state.push_back(kalman.GetState()[0]);
		measurements.push_back(z[0]);
		iteration.push_back((double)i);
		std::cout << z[0] << "," << kalman.GetCovariance()(0,0) << std::endl;
		P.push_back(kalman.GetCovariance()(0,0));
	}

	plot(iteration,measurements,"g2",iteration,state,"r-",std::vector<double>({0,49}), std::vector<double>({-0.37727,-0.37727}),"-");	
	//plot(iteration,P,"r-");
	std::cout<<P[P.size()-1]<<std::endl;
}
