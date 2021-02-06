/* File MotionModelInterface.h
*//**
*     @file MotionModelInterface.h
*     @brief This file declares the  motion model interface class
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/14/2020
*
*/

#ifndef MOTION_MODEL_INTERFACE
#define MOTION_MODEL_INTERFACE

#include<Eigen/Core>

namespace stateEstimation
{
  class MotionModelInterface 
  {
    public:
    /**
    * @brief Returns the state dimension
    *
    * @return unsigned int
    */
      virtual const unsigned int& getDimension() const = 0;
 
      virtual ~MotionModelInterface(){ }
   /**
    * @brief Predicts the current state to the next timestamp
    *
    * @param currState : Current state.
    * @return Eigen::VectorXd
    */ 
      virtual Eigen::VectorXd predictState( const Eigen::VectorXd& currState ) const = 0;
   /**
    * @brief Get the state transition matrix
    *
    * @param currState : Current state.
    * @return Eigen::MatrixXd
    */ 
      virtual Eigen::MatrixXd getStateTransitionMatrix(const Eigen::VectorXd& currState) const =0;
    /**
    * @brief Get the state noise covariance matrix
    *
    * @return Eigen::MatrixXd
    */
      virtual Eigen::MatrixXd getProcessNoiseCovariance() const =0;
    /**
    * @brief Get the state jacobian matrix
    *
    * @param currState : Current state.
    * @return Eigen::MatrixXd
    */
      virtual Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd& currState) const =0;
  };
}
#endif
