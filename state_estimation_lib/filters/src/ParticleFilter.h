#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include<Eigen/Dense>
#include<vector>
#include"Map.h"
#include"utility_functions.h"

namespace stateEstimation_pf
{
    struct Particle
    {
        int id;
        double x;
        double y;
        double theta;
        double weight;
    };

    class ParticleFilter
    {
        public:
            
            ParticleFilter(unsigned int numP);

            ~ParticleFilter(){};
            // initialize the particle filter given the mean
            void initialize(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance);

            bool isInitialized;

            //void runOneStep();
            void predict(const Eigen::VectorXd& controlVec,const Eigen::MatrixXd& covariance,const double& dt);

            void updateWeights(const std::vector<utility::observation>& noisy_meas, const Eigen::MatrixXd& covariance, const Map* map, const double& sensor_range);

            void resampleParticles();
            /**
            void dataAssociation();
            **/
            std::vector<Particle> particles;

            std::vector<double> weights;
            
            unsigned int numParticles;

        
    };


}
#endif