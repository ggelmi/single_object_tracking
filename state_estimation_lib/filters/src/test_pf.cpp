#include<iostream>
#include<iomanip>
#include<vector>
#include<fstream>
#include<random>
//#include<
#include"ParticleFilter.h"


using namespace utility;
using namespace stateEstimation_pf;


int main()
{
    // starting the clock
    int start = clock();

    std::ifstream input;
    Map* map = new Map();
    std::string map_filename ("/home/guuto/filtering/Filtering_Techniques/data/pfData/map_data.txt");
    // Loading the landmark data to the map
    bool ld = load_map_data(map_filename,map->landmark_list);
    //std::cout << "The size of the landmarks in the map : " << map->landmark_list.size()<< std::endl;
    // loading the control data
    std::vector<utility::control_command> controlVec;
    std::string cc_filename ("/home/guuto/filtering/Filtering_Techniques/data/pfData/control_data.txt");
    bool ld2 = load_control_data(cc_filename,controlVec);
    //std::cout << "The size of the control : " << controlVec.size()<< std::endl;
    // loading the groundtruth data
    std::vector<utility::ground_truth> gtVec;
    std::string gt_filename ("/home/guuto/filtering/Filtering_Techniques/data/pfData/gt_data.txt");
    bool ld3 = load_gt_data(gt_filename,gtVec);
    //std::cout << "The size of the ground_truth : " << gtVec.size()<< std::endl;
   
    double delta_t = 0.1;
    double sensor_range = 50;
    // GPS uncertainty x in meters, y in meters and theta in radians
    Eigen::MatrixXd sigma_process(3, 3);
    sigma_process <<   0.3, 0,0,
                    0, 0.3,0,
                    0,0, 0.01;
                
    Eigen::MatrixXd sigma_meas(2, 2);
    sigma_meas <<  0.3,0,
                    0, 0.3;
        
    
    std::default_random_engine rnd_gen;
    std::normal_distribution<double> rnd_x(0,sigma_process(0,0));
    std::normal_distribution<double> rnd_y(0,sigma_process(1,1));
    std::normal_distribution<double> rnd_theta(0,sigma_process(2,2));

    std::normal_distribution<double> rnd_obs_x(0,sigma_meas(0,0));
    std::normal_distribution<double> rnd_obs_y(0,sigma_meas(1,1));

    double rn_x, rn_y, rn_theta, rn_range, rn_heading;
    
    unsigned int num_particles = 729;
    unsigned int num_iterations = gtVec.size();
    //unsigned int num_iterations = 100;
    
    ParticleFilter* pf = new ParticleFilter(num_particles);

    Eigen::VectorXd total_error(3);
    Eigen::VectorXd cummulative_mean_error(3);
    total_error << 0.0,0.0,0.0;
    cummulative_mean_error << 0.0,0.0,0.0;

    // Parameters for pf runs
    int time_steps_before_lock_required = 100;
    double max_runtime = 45;
    double max_translation_error = 1;
    double max_yaw_error = 0.05;

    for(int i=0; i<num_iterations; i++)
    {
        std::cout << "Time step: " << i << std::endl;
        std::ostringstream file;
        file << "/home/guuto/filtering/Filtering_Techniques/data/pfData/observation/observations_"<<std::setfill('0')<<std::setw(6) << i+1 << ".txt";
        std::vector<utility::observation> obsVec;
        bool ld4 = load_obs_data(file.str(),obsVec);
        if(!ld4){std::cout << "Could not open the observation file : " <<i+1 << std::endl;}
        //std::cout << "The size of the obs vector : " << obsVec.size()<< std::endl;
        
        if(!pf->isInitialized)
        {
            rn_x = rnd_x(rnd_gen);
            rn_y = rnd_y(rnd_gen);
            rn_theta = rnd_theta(rnd_gen);

            //std::cout << "rn_x: " << rn_x << std::endl;
            Eigen::VectorXd rnd_vector(3);
            rnd_vector << rn_x, rn_y, rn_theta;
            Eigen::VectorXd gt_vector(3);
            gt_vector << gtVec[i].x, gtVec[i].y,gtVec[i].theta;

            Eigen::VectorXd prior = gt_vector + rnd_vector;

            pf->initialize(prior,sigma_process);
            //std::cout << "AFTER Initializing the PF" << std::endl;
        }
        else
        {
            //std::cout <<  "Calling the prediction step of the PF" << std::endl;
            Eigen::VectorXd control_vector(2);
            control_vector << controlVec[i-1].velocity, controlVec[i-1].yaw_rate;
            pf->predict(control_vector,sigma_process,delta_t);
        }

        // Given the noise measurement, we assume it was noiseless and we add noise to it
        std::vector<utility::observation> noisy_measurements;
        utility::observation obs;
        for(int j=0; j<obsVec.size(); j++)
        {
            rn_x = rnd_obs_x(rnd_gen);
            rn_y = rnd_obs_y(rnd_gen);
            obs = obsVec[j];
            obs.x = obs.x + rn_x;
            obs.y = obs.y + rn_y;
            noisy_measurements.push_back(obs);
        }
        //std::cout << "Right before updating the particles" << std::endl;
        // updating the weights of the samples
        pf->updateWeights(noisy_measurements, sigma_meas,map,sensor_range);
        //std::cout << "After particles updated" << std::endl;
        pf->resampleParticles();

        // get the particle with the highest weight

        std::vector<Particle> particles = pf->particles;
        Particle state_estimate;
        double highest_weight = 0.0;

        for(int k=0; k<particles.size(); k++)
        {
            if(particles[k].weight > highest_weight)
            {
                highest_weight = particles[k].weight;
                state_estimate = particles[k];
            }
        }

        // computing the error between the state estimate and the ground truth
        Eigen::VectorXd gt_vector(3);
        gt_vector << gtVec[i].x, gtVec[i].y, gtVec[i].theta; 
        Eigen::VectorXd state_estimate_vector(3);
        state_estimate_vector << state_estimate.x, state_estimate.y,state_estimate.theta;

        Eigen::VectorXd error = computeError(state_estimate_vector, gt_vector);

        std::cout << "Error: x " << error(0) << " y " << error(1) << " z "<< error(2) << std::endl;

        total_error += error;

        cummulative_mean_error = total_error/ double(i+1);

        std::cout << "Cummulative mean weight error: "<< cummulative_mean_error(0) << " y " << cummulative_mean_error(1) << " yaw "<< cummulative_mean_error(2) << std::endl;


        if(i >= time_steps_before_lock_required)
        {
            if(cummulative_mean_error(0) > max_translation_error ||cummulative_mean_error(1) > max_translation_error ||cummulative_mean_error(2) > max_yaw_error )
            {
                if(cummulative_mean_error(0) > max_translation_error )
                {
                    std::cout << "Cumm Error in x dimension: " << cummulative_mean_error(0) << " is greater than maximum allowable error " << max_translation_error << std::endl;
                }else if (cummulative_mean_error(1) > max_translation_error )
                {
                    std::cout << "Cumm Error in y dimension: " << cummulative_mean_error(1) << " is greater than maximum allowable error " << max_translation_error << std::endl;
                }else
                {
                    std::cout << "Cumm Error in theta dimension: " << cummulative_mean_error(3) << " is greater than maximum allowable error " << max_yaw_error << std::endl;
                }

                return -1;
            }
        }
        
    }

    int stop = clock();
    double runtime = (stop - start) / double(CLOCKS_PER_SEC);
    std::cout << "Runtime (sec): " << runtime << std::endl;

    // Printing the success rate
    if(runtime < max_runtime && pf->isInitialized)
    {
        std::cout << "SUCCESS!!! The Filter passed!" << std::endl;
    }else if(!pf->isInitialized)
    {
        std::cout << "The filter is not initialized" << std::endl;
    }else
    {
        std::cout << "The runtime " << runtime << " is greater than max allowable runtime " << max_runtime << std::endl;
        return -1;
    }

    return 0;

}