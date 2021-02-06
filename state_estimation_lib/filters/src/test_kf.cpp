#include<Eigen/Dense>
#include<iostream>
#include<cmath>

#include"KFAdapter.h"
#include"EKFAdapter.h"
#include"UKFAdapter.h"
#include"CvMeasurementModel.h"
#include"DualBearingMeasModel.h"
#include"CoordinatedTurnModel.h"
#include"ConstantVelocityModel.h"



using namespace stateEstimation;

int main()
{
    /**
    // Define the covariance matrix and the mean
    Eigen::MatrixXd sigma(4, 4);
    sigma <<   1, 0,0,0,
                0, 1,0,0,
                0,0,1,0,
                0,0,0,1;

    Eigen::VectorXd mean(4);
    mean << 0, 0,0,0;

    //creating the motion model
    const double delta = 1;
    const double sig = 0.5;
    unsigned int dim = 4;
    MotionModelInterface* cv_model = new ConstantVelocityModel(delta,sig,dim);

    // creating measurement model
    const double sigma_meas = 20;
    unsigned int sensor_dim = 2;

    MeasurementModelInterface* meas_model = new CvMeasurementModel(sigma_meas,sensor_dim);

    // creating the kf filter

    FilterInterface* kf_filter = new KFAdapter(cv_model,meas_model);

    std::cout << "mean before prediction: \n" << mean << std::endl;
    std::cout << "cov before prediction: \n"<< sigma << std::endl;

    kf_filter->predictState(mean,sigma);

    std::cout << "mean after prediction: \n" << mean << std::endl;
    std::cout << "cov mean prediction: \n"<< sigma << std::endl;

    Eigen::VectorXd measVector(2);
    measVector << 5,5;

    kf_filter->updateState(mean,sigma,measVector);

    std::cout << "mean after update: \n" << mean << std::endl;
    std::cout << "cov mean update: \n"<< sigma << std::endl;
    **/
    // Testing the EKF
    const double Delta = 1;
    const double sigmaV = 1;
    const double sigmaOmega = M_PI/180;
    unsigned int dimen = 5;

    MotionModelInterface* ct_model = new CoordinatedTurnModel(Delta,sigmaV,sigmaOmega,dimen);
    std::cout << "Q : \n" << ct_model->getProcessNoiseCovariance() << std::endl;
    Eigen::VectorXd sensor1(2);
    Eigen::VectorXd sensor2(2);
    
    const double sigm = 0.5*M_PI/180;
    sensor1 << -200,100;
    sensor2 << -200,-100;

    unsigned int sensorDimen = 2;
    MeasurementModelInterface* dbm_model = new DualBearingMeasModel(sigm,sensor1,sensor2,sensorDimen);
    std::cout << "R : \n" << dbm_model->getMeasurementNoiseCovariance() << std::endl;
    Eigen::VectorXd X(5);
    X << 0, 0,14,0,0;
    Eigen::MatrixXd P(dimen, dimen);
    P << 10,0,0,0,0,
        0,10,0,0,0,
        0,0,2,0,0,
        0,0,0,(M_PI/180),0,
        0,0,0,0,((5*M_PI)/180);
    P = P*P;
    //std::cout << "P : \n" << P<< std::endl;


    //FilterInterface* ekf_filter = new EKFAdapter(ct_model,dbm_model);
    FilterInterface* ukf_filter = new UKFAdapter(ct_model,dbm_model);

    std::cout << "X before prediction: \n" << X << std::endl;
    std::cout << "P before prediction: \n"<< P << std::endl;
    
    ukf_filter->predictState(X,P);
    
    std::cout << "X after prediction: \n" << X << std::endl;
    std::cout << "P after prediction: \n"<< P << std::endl;
    
    Eigen::VectorXd measVector1(2);
    measVector1 << -0.4721,
                   0.4239;
    
    ukf_filter->updateState(X,P,measVector1);

    std::cout << "X after update: \n" << X << std::endl;
    std::cout << "P after update: \n"<< P << std::endl;
   

    return 0;
}