#include <chrono>

#include "WhiteNoiseSystem.h"

#include <boost/math/distributions/normal.hpp>
#include "boost/numeric/ublas/vector.hpp"

boost::numeric::ublas::vector<double>
WhiteNoiseSystem::Measure(int n, double mean, double stdev)
{
	
	boost::normal_distribution<double> nd(mean,stdev);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > measurement(_gen, nd);
    
    boost::numeric::ublas::vector<double> vec(n);

    for(int i = 0; i < n; i++)
    {
        vec[i] = measurement();
    }
    return vec;
}

WhiteNoiseSystem::WhiteNoiseSystem()
{
	std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
	    std::chrono::system_clock::now().time_since_epoch()
	);
    //_gen.seed(ms.count());
    _gen.seed(0);
}