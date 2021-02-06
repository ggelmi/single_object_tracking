/* File DualBearingMeasModel.cpp
*//**
*     @file DualBearingMeasModel.cpp
*     @brief This file implements the dual bearing measurement model
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/15/2020
*
*/
#include<cmath>
#include<iostream>
#include"DualBearingMeasModel.h"

namespace stateEstimation
{
  DualBearingMeasModel:: DualBearingMeasModel (const double& sig, const Eigen::VectorXd& sensorOnePos,const Eigen::VectorXd& sensorTwoPos, unsigned int& dim)
    {
      sigma = sig;
      DIM = dim;
      sensorOnePosition = sensorOnePos;
      sensorTwoPosition = sensorTwoPos;
    }

  unsigned int DualBearingMeasModel:: getDimension() const
    {
      return DIM;
    }
  
  Eigen::VectorXd  DualBearingMeasModel:: getMeasurementVector(const Eigen::VectorXd& currState)
    {
      Eigen::VectorXd measVector(getDimension());
      measVector <<  atan2((currState(1)-sensorOnePosition(1)),(currState(0)-sensorOnePosition(0))),
                      atan2((currState(1)-sensorTwoPosition(1)),(currState(0)-sensorTwoPosition(0)));
      return measVector;
    }
  
  Eigen::MatrixXd DualBearingMeasModel:: getObservationMatrix()
    {
      Eigen::MatrixXd observationMatrix (getDimension(),getDimension());
      return observationMatrix;
    }
  
  Eigen::MatrixXd DualBearingMeasModel:: getJacobianMatrix(const Eigen::VectorXd& currState) const
    {
      Eigen::VectorXd x(2);
      x << currState(0),currState(1);
      
      //std::cout << "getDimension " << getDimension() << std::endl;
      Eigen::VectorXd diff1 = x-sensorOnePosition;
      double range1 = std::sqrt((std::pow(diff1(0),2)) + (std::pow(diff1(1),2)));
      Eigen::VectorXd diff2 = x-sensorTwoPosition;
      double range2 = std::sqrt((std::pow(diff2(0),2)) + (std::pow(diff2(1),2)));
      
      Eigen::MatrixXd jacobianMatrix(getDimension(),5);
      jacobianMatrix << -((currState(1)-sensorOnePosition(1)) / std::pow(range1,2)), ((currState(0)-sensorOnePosition(0)) / std::pow(range1,2)), 0,0,0,
                        -((currState(1)-sensorTwoPosition(1)) / std::pow(range2,2)), ((currState(0)-sensorTwoPosition(0)) / std::pow(range2,2)), 0,0,0;
      return jacobianMatrix;
    }
  
  Eigen::MatrixXd DualBearingMeasModel:: getMeasurementNoiseCovariance() 
    {
      Eigen::MatrixXd measCovariance (getDimension(),getDimension());
      measCovariance << std::pow(sigma,2), 0,
                        0, std::pow(sigma,2);
      return measCovariance;
    }
}
