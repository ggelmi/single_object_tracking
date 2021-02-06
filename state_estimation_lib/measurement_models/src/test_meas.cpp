#include<Eigen/Dense>
#include<iostream>
#include"CvMeasurementModel.h"
#include"CtMeasurementModel.h"
#include"BearingMeasurementModel.h"
#include"DualBearingMeasModel.h"
#include"RangeBearingMeasModel.h"

using namespace stateEstimation;

int main()
{

  const double sigma = 0.5;

  unsigned int dimen = 2;

  /**
  CvMeasurementModel cvMeas_model (sigma, dimen);

  std::cout << "The meas dimension: " << cvMeas_model.getDimension() <<std::endl;

  std::cout << "The obs Matrix : \n" << cvMeas_model.getObservationMatrix() << std::endl;

  std::cout << "The cov Matrix: \n" << cvMeas_model.getMeasurementNoiseCovariance() << std::endl;

 
  Eigen::VectorXd currState (4);

  currState << 1,2,0,0;


  Eigen::VectorXd meas =  cvMeas_model.getMeasurementVector(currState);


  std::cout << "The meas : \n " << meas << std::endl;

  

  CtMeasurementModel ctMeas_model (sigma, dimen);

  std::cout << "The meas dimension: " << ctMeas_model.getDimension() <<std::endl;

  
  std::cout << "The obs Matrix : \n" << ctMeas_model.getObservationMatrix() << std::endl;

  std::cout << "The cov Matrix: \n" << ctMeas_model.getMeasurementNoiseCovariance() << std::endl;


  Eigen::VectorXd currState (4);

  currState << 1,2,0,0;


  Eigen::VectorXd meas =  ctMeas_model.getMeasurementVector(currState);


  std::cout << "The meas : \n " << meas << std::endl;

 
  Eigen::VectorXd sensor(2);

  sensor << 3,2;

  BearingMeasurementModel bmmodel(sigma,sensor,dimen);

  std::cout << "The meas dimension: " << bmmodel.getDimension() <<std::endl;

  Eigen::VectorXd currState (4);
  currState << 1,2,0,0;

  std::cout << "The meas vector is " << bmmodel.getMeasurementVector(currState) << std::endl;

  std::cout << "The jacobian is :\n " << bmmodel.getJacobianMatrix(currState) << std::endl;

  std::cout << "The variance is " << bmmodel.getMeasurementNoiseCovariance() << std::endl;
  

  Eigen::VectorXd sensor1(2);
  Eigen::VectorXd sensor2(2);
  
  sensor1 << 3,2;
  sensor2 << 2,3;

  DualBearingMeasModel dbmmodel(sigma,sensor1,sensor2,dimen);

  std::cout << "The meas dimension: " << dbmmodel.getDimension() <<std::endl;

  Eigen::VectorXd currState (4);
  currState << 1,2,0,0;

  std::cout << "The meas vector is: \n " << dbmmodel.getMeasurementVector(currState) << std::endl;

  std::cout << "The jacobian is :\n " << dbmmodel.getJacobianMatrix(currState) << std::endl;

  std::cout << "The covariance is : \n " << dbmmodel.getMeasurementNoiseCovariance() << std::endl;

  **/

  const double sigma_r = 0.5;
  const double sigma_b = 0.8;

  unsigned int dimension= 2;

  Eigen::VectorXd sensor(2);

  sensor << 3,2;

  RangeBearingMeasModel rbmmodel(sigma_r, sigma_b,sensor,dimension);

  std::cout << "The meas dimension: " << rbmmodel.getDimension() <<std::endl;
  
  Eigen::VectorXd currState (4);
  currState << 1,2,0,0;

  std::cout << "The meas vector is : \n " << rbmmodel.getMeasurementVector(currState) << std::endl;
  
  std::cout << "The jacobian is :\n " <<rbmmodel.getJacobianMatrix(currState) << std::endl;

  std::cout << "The variance is : \n " << rbmmodel.getMeasurementNoiseCovariance() << std::endl;

	return 0;
}
