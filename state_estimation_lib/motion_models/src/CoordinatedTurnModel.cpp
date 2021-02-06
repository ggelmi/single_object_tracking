/* File CoordinatedTurnModel.cpp
*//**
*     @file CoordinatedTurnModel.cpp
*     @brief This file implements the coordinated turn motion model
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/14/2020
*
*/

#include<cmath>
#include"CoordinatedTurnModel.h"

namespace stateEstimation
{
   CoordinatedTurnModel::CoordinatedTurnModel(const double& dtime,const double& sigV,const double& sigOmega, unsigned int& dimension)
      { 
         dt = dtime;
         sigmaV= sigV;
         sigmaOmega = sigOmega;
         DIM = dimension;
      }

   const unsigned int& CoordinatedTurnModel:: getDimension() const
      {
         return DIM;
      }

   Eigen::VectorXd CoordinatedTurnModel:: predictState( const Eigen::VectorXd& currState) const
      {
         Eigen::VectorXd x(getDimension());
         x <<  dt*currState(2)*cos(currState(3)),
               dt*currState(2)*sin(currState(3)),
               0,
               dt*currState(4),
               0;
         Eigen::VectorXd predicted_state =  currState + x;
         return predicted_state;
      }

   Eigen::MatrixXd CoordinatedTurnModel:: getStateTransitionMatrix(const Eigen::VectorXd& currState) const
      {
         Eigen::MatrixXd stateTransitionMatrix (getDimension(),getDimension());
         return stateTransitionMatrix;
      }
      
   Eigen::MatrixXd CoordinatedTurnModel:: getJacobianMatrix(const Eigen::VectorXd& currState) const
      {
         Eigen::MatrixXd jacobianMatrix (getDimension(),getDimension());
         jacobianMatrix << 1,0,dt*cos(currState(3)),-dt*currState(2)*sin(currState(3)),0,
                           0,1,dt*sin(currState(3)),dt*currState(2)*cos(currState(3)),0,
                           0,0,1,0,0,
                           0,0,0,1,dt,
                           0,0,0,0,1;
         return jacobianMatrix;
      }

   Eigen::MatrixXd CoordinatedTurnModel:: getProcessNoiseCovariance() const
      {
         Eigen::MatrixXd G(getDimension(),2);
         G << 0, 0,
               0, 0,
               1, 0,
               0, 0,
               0, 1;
         Eigen::MatrixXd diagSigmas(2,2);
         diagSigmas << std::pow(sigmaV,2), 0,
                        0, std::pow(sigmaOmega,2);
         Eigen::MatrixXd processNoiseMatrix (getDimension(),getDimension());
         processNoiseMatrix = G*diagSigmas*G.transpose();
         return processNoiseMatrix;
      }
}