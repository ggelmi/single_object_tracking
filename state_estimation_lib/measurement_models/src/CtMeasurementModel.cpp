/* File CtMeasurementModel.cpp
*//**
*     @file CtMeasurementModel.cpp
*     @brief This file implements the coordinated turn measurement model
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/14/2020
*
*/

#include<cmath>
#include"CtMeasurementModel.h"

namespace stateEstimation
{
  CtMeasurementModel:: CtMeasurementModel (const double& sig, unsigned int& dim)
    {
      sigma = sig;
      DIM = dim;
    }
  
  unsigned int CtMeasurementModel:: getDimension() const
    {
      return DIM;
    }
  
  Eigen::VectorXd CtMeasurementModel:: getMeasurementVector(const Eigen::VectorXd& currState)
    {
      Eigen::VectorXd meas_vector = getObservationMatrix()*currState;
      return meas_vector;
    }
  
  Eigen::MatrixXd CtMeasurementModel:: getObservationMatrix()
    {
      Eigen::MatrixXd observationMatrix (getDimension(),4);
      observationMatrix << 1,0,0,0,
                          0,1,0,0;
      return observationMatrix;
    }
  
  Eigen::MatrixXd CtMeasurementModel::getMeasurementNoiseCovariance()
    {
      Eigen::MatrixXd measCovariance (getDimension(),getDimension());
      measCovariance << 1,0,
                        0,1;
      measCovariance =  std::pow(sigma,2)*measCovariance;
      return measCovariance;
    }
}

