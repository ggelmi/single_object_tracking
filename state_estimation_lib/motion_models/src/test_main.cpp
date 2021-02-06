#include<Eigen/Dense>
#include<iostream>
#include"ConstantVelocityModel.h"
#include"CoordinatedTurnModel.h"

using namespace stateEstimation;

int main()
{

const double delta = 1;
const double sigma = 0.5;
unsigned int dim = 4;

ConstantVelocityModel  cv_model(delta,sigma,dim);

std::cout << "The dimension is : " << cv_model.getDimension() << std::endl;

Eigen::VectorXd currState (dim);

currState << 1,1,0,0;


std::cout << "The stateTransition : \n " << cv_model.getStateTransitionMatrix(currState) << std::endl;
Eigen::MatrixXd tMatrix = cv_model.getStateTransitionMatrix(currState);

std::cout << "The 2by2 state : \n "<< tMatrix(Eigen::seqN(0,2), Eigen::seqN(0,2)) << std::endl;
std::cout << "The processNoise : \n " << cv_model.getProcessNoiseCovariance() << std::endl;


Eigen::VectorXd predState =  cv_model.predictState(currState);

std::cout << "The predicted state : \n " << predState << std::endl;


// testing the coordinated turn model

const double Delta = 1;
const double sigmaV = 0.01;
const double sigmaOmega = 0.2;
unsigned int dimen= 5;

CoordinatedTurnModel  ct_model(Delta,sigmaV,sigmaOmega,dimen);

std::cout << "The dimension is : " << ct_model.getDimension() << std::endl;

Eigen::VectorXd currState2 (dimen);

currState2 << 1,1,0.3,2.7,0;

std::cout << "currState2 is " << currState2 << std::endl;

Eigen::VectorXd predState2 =  ct_model.predictState(currState2);

std::cout << "The predicted state : \n " << predState2 << std::endl;

std::cout << "The processNoise : \n" << ct_model.getProcessNoiseCovariance() << std::endl;

std::cout << "The jacobian : \n" << ct_model.getJacobianMatrix(currState2) << std::endl;

return 0;
}
