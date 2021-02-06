/* File MeasurementModelnterface.h
*//**
*     @file MeasurementModelInterface.h
*     @brief This file declares the  measurement model interface class
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/14/2020
*
*/

#ifndef MEASUREMENT_MODEL_INTERFACE
#define MEASUREMENT_MODEL_INTERFACE

#include<Eigen/Dense>

namespace stateEstimation
{
  class MeasurementModelInterface  
  {
    public:
    /**
    * @brief Returns the measurement dimension
    *
    * @return unsigned int
    */
      virtual unsigned int getDimension() const = 0;
    
      virtual ~MeasurementModelInterface(){ };
    /**
    * @brief Get the measurement vector
    *
    * @param currState : Current state.
    * @return Eigen::VectorXd
    */
      virtual Eigen::VectorXd getMeasurementVector(const Eigen::VectorXd& currState)=0;
    /**
    * @brief Get the measurement observation matrix
    *
    * @return Eigen::MatrixXd
    */
      virtual Eigen::MatrixXd getObservationMatrix()=0;
    /**
    * @brief Get the measurement noise covariance matrix
    *
    * @return Eigen::MatrixXd
    */
      virtual Eigen::MatrixXd getMeasurementNoiseCovariance()=0;
    /**
    * @brief Get the measurement jacobian matrix
    *
    * @param currState : Current state.
    * @return Eigen::MatrixXd
    */
      virtual Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd& currState) const = 0 ;
  };
}
#endif
