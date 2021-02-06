#include"ParticleFilter.h"
#include<random>
#include<algorithm>
#include<numeric>
#include<iostream>

namespace stateEstimation_pf
{
    ParticleFilter::ParticleFilter(unsigned int numP)
        {
            numParticles = numP;
            isInitialized = false;
        }
        
    void ParticleFilter:: initialize(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance)
        {
            
            weights.resize(numParticles);
            particles.resize(numParticles);

            std::normal_distribution<double> distrib_x(mean(0),covariance(0,0));
            std::normal_distribution<double> distrib_y(mean(1),covariance(1,1));
            std::normal_distribution<double> distrib_theta(mean(2),covariance(2,2));

            std::default_random_engine rnd_gen;
            
            for(int i=0; i<numParticles; i++)
            {
                Particle particle;
                particle.id = i;
                particle.x = distrib_x(rnd_gen);
                particle.y = distrib_y(rnd_gen);
                particle.theta = distrib_theta(rnd_gen);
                particle.weight = 1;

                particles[i] = particle;
                weights[i] = particle.weight;
            }
            
            isInitialized = true;
            //std::cout << "The size of initial particles: " << particles.size() << std::endl;
        }
    void ParticleFilter::predict(const Eigen::VectorXd& controlVec,const Eigen::MatrixXd& process_covariance,const double& dt)
        {

            std::default_random_engine rnd_gen;
            double velocity = controlVec(0);
            double yaw_rate = controlVec(1);
            for(int i=0; i<numParticles; i++)
            {
                Particle* par = &particles[i];
                
                // Computing the new robot location based on kinematics and control commands
                double x_new = par->x + (velocity/yaw_rate) * (sin(par->theta + yaw_rate*dt) - sin(par->theta)) ;
                double y_new = par->y + (velocity/yaw_rate) * (cos(par->theta) - cos(par->theta + yaw_rate*dt )) ;
                double theta_new = par->theta + (yaw_rate*dt);

                // Adding gaussian noise to the predicted position
                std::normal_distribution<double> distrib_x(x_new,process_covariance(0,0));
                std::normal_distribution<double> distrib_y(y_new,process_covariance(1,1));
                std::normal_distribution<double> distrib_theta(theta_new,process_covariance(2,2));   

                // Updating the particle fields
                par->x = distrib_x(rnd_gen);
                par->y = distrib_y(rnd_gen);
                par->theta = distrib_theta(rnd_gen);
            }

        }
    
    void ParticleFilter:: updateWeights(const std::vector<utility::observation>& noisy_meas, const Eigen::MatrixXd& meas_covariance, 
                        const Map* map, const double& sensor_range)
        {
            double weight_sum = 0;
            
            for(int i=0; i<numParticles; i++)
            {

                Particle* p = &particles[i];
                double weight = 1;
                // converting the observation from vehicle to map coordinate system
                for(int j=0; j<noisy_meas.size(); j++)
                {
                    utility::observation current_obs = noisy_meas[j];
                    utility::observation converted_obs;
                    Eigen::VectorXd conv_obs(2);
                    converted_obs.x = (current_obs.x*cos(p->theta)) - (current_obs.y*sin(p->theta)) + p->x;
                    converted_obs.y = (current_obs.x*sin(p->theta)) + (current_obs.y*cos(p->theta)) + p->y;
                    converted_obs.id = current_obs.id;

                    // find the predicted measurement which corresponds to this observation
                    // Then assign landmark to this measurement
                    //std::cout << "Noisy meas : " << noisy_meas[j].x << std::endl;
                    Map::landmark landmark;
                    double distance_min = std::numeric_limits<double>::max();
                    
                    for(int k=0; k<map->landmark_list.size();k++)
                    {
                        Map::landmark current_landmark = map->landmark_list[k];
                        Eigen::VectorXd curr_landmrk(2);
                        conv_obs << converted_obs.x , converted_obs.y ;
                        curr_landmrk << current_landmark.x , current_landmark.y ;
                        double distance = utility::distance(conv_obs,curr_landmrk);
                        
                        if(distance < distance_min)
                        {
                            distance_min = distance;
                            landmark = current_landmark;
                        }

                    }
                    //std::cout << "distance min: " << distance_min << std::endl;
                    // updating the weights of the particle using the observation
                    // we use the multi-gaussian pdf
                    Eigen::VectorXd landmrk_vector(2);
                    landmrk_vector << landmark.x , landmark.y;
                    double prob= utility::compute_pdf(conv_obs,landmrk_vector,meas_covariance);
                    weight*=prob;
                }

                weight_sum +=weight;
                p->weight = weight;
            }

            // normalizing the weights of the particles to bring them within (0,1)

            for(int i=0; i<numParticles; i++)
            {
                Particle* p = &particles[i];
                p->weight = p->weight / weight_sum;
                weights[i] = p->weight;
            }
            
        }
    
    void ParticleFilter::resampleParticles()
    {
        //std::cout << "Resampling Particles" << std::endl;
        std::default_random_engine rnd_gen;

        std::discrete_distribution<int> distribution(weights.begin(),weights.end());
        std::vector<Particle> resampledParticles;

        for(int i=0; i<numParticles; i++)
        {
            resampledParticles.push_back(particles[distribution(rnd_gen)]);
        }

        particles = resampledParticles;
    }
}