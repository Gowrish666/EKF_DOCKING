#pragma once
#include <random>

class NoiseGenerator
{
    public:
       NoiseGenerator()
           :noiseX_(0.0,0.02),
            noiseY_(0.0,0.02),
            noiseTheta_(0.0, 0.01),
            probability_(0.0,1.0),
            outlierNoise_(-0.5, 0.5)
    {
    }
    
    double x()
    {
        return noiseX_(generator_);
    }

    double y()
    {
        return noiseY_(generator_);
    }

    double theta()
    {
        return noiseTheta_(generator_);
    }

    bool isOutlier()
    {
        return probability_(generator_)< 0.5;
    }
    double outlier()
    {
        return outlierNoise_(generator_);
    }

    bool isDropout()
    {
        return probability_(generator_)< 0.10;
    }

    private:
       
       std::default_random_engine generator_;

       std::normal_distribution<double> noiseX_;
       std::normal_distribution<double> noiseY_;
       std::normal_distribution<double> noiseTheta_;

       std::uniform_real_distribution<double> probability_;
       std::uniform_real_distribution<double> outlierNoise_;
};     