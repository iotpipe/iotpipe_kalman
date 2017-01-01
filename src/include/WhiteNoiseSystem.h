#pragma once

#include <Eigen/Dense>
#include <boost/random.hpp>

class WhiteNoiseSystem {

    public:
        WhiteNoiseSystem();
        Eigen::VectorXd Measure(int num_measurements, double mean, double stdev);

    private:        
        boost::mt19937 _gen;
        
};