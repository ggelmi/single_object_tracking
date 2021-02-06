/* File CoordinatedTurnModel.h
*//**
*     @file CoordinatedTurnModel.h
*     @brief This file declares the coordinated turn motion model
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/14/2020
*
*/

#ifndef COORDINATED_TURN_MODEL_H
#define COORDINATED_TURN_MODEL_H

#include"MotionModelInterface.h"

namespace stateEstimation
{
  class CoordinatedTurnModel: public  MotionModelInterface
  {
    public:
      CoordinatedTurnModel(const double& dtime, const double& sigPhi, const double& sigOmega, unsigned int& dimension);
    
      ~CoordinatedTurnModel(){ };
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
      Eigen::VectorXd predictState( const Eigen::VectorXd& currState) const;
      /**
      * @brief Get the state transition matrix
      *
      * @param currState : Current state.
      * @return Eigen::MatrixXd
      */
      Eigen::MatrixXd getStateTransitionMatrix(const Eigen::VectorXd& currState) const ;
      /**
      * @brief Get the state noise covariance matrix
      *
      * @return Eigen::MatrixXd
      */
      Eigen::MatrixXd  getProcessNoiseCovariance() const ;
      /**
      * @brief Get the state jacobian matrix
      *
      * @param currState : Current state.
      * @return Eigen::MatrixXd
      */
      Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd& currState) const ;
    private:
      double dt;         // sampling timestamp
      double sigmaV;     // standard deviation for the velocity
      double sigmaOmega; // standard deviation for the angular rate
      unsigned int DIM;  // Dimension of the state space
  };
}
#endif 
