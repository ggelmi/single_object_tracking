/* File ConstantVelocityMotionModel.h
*//**
*     @file ConstantVelocityMotionModel.h
*     @brief This file declares the constant velocity motion model
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/14/2020
*
*/

#ifndef CONSTANT_VELOCITY_MODEL_H
#define CONSTANT_VELOCITY_MODEL_H

#include"MotionModelInterface.h"

namespace stateEstimation
{
  class ConstantVelocityModel: public  MotionModelInterface
  {
    public:
    
      ConstantVelocityModel(const double& dtime, const double& sig, unsigned int& dimension);

      ~ConstantVelocityModel(){ };
      /**
      * @brief Returns the state dimension
      *
      * @return unsigned int
      */
      const unsigned int& getDimension() const ;
	    /**
      * @brief Predicts the current state to the next timestamp
      *
      * @param currState : Current state.
      * @return Eigen::VectorXd
      */ 
      Eigen::VectorXd predictState( const Eigen::VectorXd& currState) const ;
      /**
      * @brief Get the state transition matrix
      *
      * @param currState : Current state.
      * @return Eigen::MatrixXd
      */
      Eigen::MatrixXd getStateTransitionMatrix(const Eigen::VectorXd& currState) const;
      /**
      * @brief Get the state noise covariance matrix
      *
      * @return Eigen::MatrixXd
      */
      Eigen::MatrixXd  getProcessNoiseCovariance() const;
      /**
      * @brief Get the state jacobian matrix
      *
      * @param currState : Current state.
      * @return Eigen::MatrixXd
      */
      Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd& currState) const;
    private:
      double dt;         // sampling timestamp
      double sigma;      // standard deviation for the motion-model noise
      unsigned int DIM;  // Dimension of the state space
  };
}
#endif
