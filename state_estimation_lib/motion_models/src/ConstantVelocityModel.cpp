/* File ConstantVelocityMotionModel.cpp
*//**
*     @file ConstantVelocityMotionModel.cpp
*     @brief This file implements the constant velocity motion model
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/14/2020
*
*/

#include<cmath>
#include"ConstantVelocityModel.h"
#include<iostream>

namespace stateEstimation
{
   ConstantVelocityModel::ConstantVelocityModel(const double& dtime,const double& sig, unsigned int& dimension)
      { 
         dt = dtime;
         sigma = sig;
         DIM = dimension;
      }
 
   const unsigned int& ConstantVelocityModel:: getDimension() const
      {
         return DIM;
      }

   Eigen::VectorXd ConstantVelocityModel:: predictState( const Eigen::VectorXd& currState) const 
      {  
         Eigen::VectorXd predicted_state =  getStateTransitionMatrix(currState)*currState; 
         return predicted_state; 
      }

    Eigen::MatrixXd ConstantVelocityModel:: getStateTransitionMatrix(const Eigen::VectorXd& currState) const 
      {
         Eigen::MatrixXd stateTransitionMatrix (getDimension(),getDimension());
         stateTransitionMatrix << 1,0,dt,0,
                                  0,1,0,dt,
                                  0,0,1,0,
                                  0,0,0,1;
         return stateTransitionMatrix;	                  
      }

   Eigen::MatrixXd ConstantVelocityModel::  getProcessNoiseCovariance() const
      {
         Eigen::MatrixXd processNoiseMatrix(getDimension(),getDimension());
         processNoiseMatrix << std::pow(dt, 4)/4,  0, std::pow(dt, 3)/2,0,
                              0, std::pow(dt, 4)/4, 0, std::pow(dt, 3)/2,
                              std::pow(dt, 3)/2, 0,  std::pow(dt, 2), 0,
                              0,std::pow(dt, 3)/2, 0,    std::pow(dt, 2);   
         processNoiseMatrix = std::pow(sigma,2) * processNoiseMatrix;
         return processNoiseMatrix;
      }
   
   Eigen::MatrixXd ConstantVelocityModel:: getJacobianMatrix(const Eigen::VectorXd& currState) const
      {
         Eigen::MatrixXd jacobian(getDimension(),getDimension());
         return jacobian;
      }
}


