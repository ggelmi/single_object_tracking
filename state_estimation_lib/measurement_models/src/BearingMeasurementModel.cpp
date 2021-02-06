/* File BearingMeasurementModel.cpp
*//**
*     @file BearingMeasurementModel.cpp
*     @brief This file implements the bearing only measurement model
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/15/2020
*
*/

#include<cmath>
#include"BearingMeasurementModel.h"

namespace stateEstimation
{
  BearingMeasurementModel:: BearingMeasurementModel (const double& sig, const Eigen::VectorXd& sensorPos, unsigned int& dim)
    {
      sigma = sig;
      DIM = dim;
      sensorPosition = sensorPos;
    }
  
  unsigned int BearingMeasurementModel:: getDimension() const
    {
      return DIM;
    }
  
  Eigen::VectorXd  BearingMeasurementModel:: getMeasurementVector(const Eigen::VectorXd& currState)
    {
      Eigen::VectorXd measVector(getDimension()); 
      measVector <<  atan2((currState(1)-sensorPosition(1)),(currState(0)-sensorPosition(0)));
      return measVector;
    }
  
  Eigen::MatrixXd BearingMeasurementModel:: getObservationMatrix()
    {
      Eigen::MatrixXd observationMatrix (getDimension(),getDimension());
      return observationMatrix;
    }
  
  Eigen::MatrixXd BearingMeasurementModel:: getJacobianMatrix(const Eigen::VectorXd& currState)
    {
      Eigen::VectorXd x(2);
      x << currState(0),currState(1);
      
      Eigen::VectorXd diff = x-sensorPosition;
      double range = std::sqrt((std::pow(diff(0),2)) + (std::pow(diff(1),2)));
      
      Eigen::MatrixXd jacobianMatrix(getDimension(),4);
      jacobianMatrix << -((currState(1)-sensorPosition(1)) / std::pow(range,2)), ((currState(0)-sensorPosition(0)) / std::pow(range,2)), 0,0;
      return jacobianMatrix;	
    }
  
  Eigen::MatrixXd BearingMeasurementModel:: getMeasurementNoiseCovariance()
    {
      Eigen::MatrixXd measCovariance (getDimension(),getDimension());
      measCovariance << std::pow(sigma,2);	  
      return measCovariance;
  }
}