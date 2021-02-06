
/* File RangeBearingMeasModel.cpp
*//**
*     @file RangeBearingMeasModel.cpp
*     @brief This file implements the range bearing measurement model
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/15/2020
*
*/

#include<cmath>
#include"RangeBearingMeasModel.h"

namespace stateEstimation
{
  RangeBearingMeasModel::RangeBearingMeasModel (const double& sig_r,const double& sig_b,
                                                const Eigen::VectorXd& sensorPos, unsigned int& dim)
    {
      sigma_r = sig_r;
      sigma_b = sig_b;
      DIM = dim;
      sensorPosition = sensorPos;
    }

  unsigned int RangeBearingMeasModel :: getDimension() const
    {
      return DIM;
    }
  
  Eigen::VectorXd  RangeBearingMeasModel:: getMeasurementVector(const Eigen::VectorXd& currState)
    {
      Eigen::VectorXd measVector(getDimension());
      double bearing = atan2((currState(1)-sensorPosition(1)),(currState(0)-sensorPosition(0)));
      
      Eigen::VectorXd x(2);
      x << currState(0),currState(1);
      Eigen::VectorXd diff = x-sensorPosition;
      double range = std::sqrt((std::pow(diff(0),2)) + (std::pow(diff(1),2)));
      
      measVector <<  range, bearing;
      return measVector;
    }
  
  Eigen::MatrixXd RangeBearingMeasModel:: getObservationMatrix()
    {
      Eigen::MatrixXd observationMatrix (getDimension(),getDimension());
      return observationMatrix;
    }
  
  Eigen::MatrixXd RangeBearingMeasModel:: getJacobianMatrix(const Eigen::VectorXd& currState) const
    {
        Eigen::VectorXd x(2);
        x << currState(0),currState(1);
        
        Eigen::VectorXd diff = x-sensorPosition;
        double range = std::sqrt((std::pow(diff(0),2)) + (std::pow(diff(1),2)));
        
        Eigen::MatrixXd jacobianMatrix(getDimension(),4);
        jacobianMatrix << ((currState(0)-sensorPosition(0)) / range),((currState(1)-sensorPosition(1)) / range), 0,0,
                          -((currState(1)-sensorPosition(1)) / std::pow(range,2)), ((currState(0)-sensorPosition(0)) / std::pow(range,2)), 0,0;
        return jacobianMatrix;
  }
  
  Eigen::MatrixXd RangeBearingMeasModel:: getMeasurementNoiseCovariance()
    {
      Eigen::MatrixXd measCovariance (getDimension(),getDimension());
      measCovariance << std::pow(sigma_r,2),0,
                        0,std::pow(sigma_b,2);
      return measCovariance;
    }
}
