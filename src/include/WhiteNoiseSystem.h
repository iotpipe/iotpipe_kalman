#pragma once

#include "boost/numeric/ublas/fwd.hpp"
#include <boost/random.hpp>

class WhiteNoiseSystem {

    public:
        WhiteNoiseSystem();
        boost::numeric::ublas::vector<double> Measure(int num_measurements, double mean, double stdev);

    private:        
        boost::mt19937 _gen;
        
};