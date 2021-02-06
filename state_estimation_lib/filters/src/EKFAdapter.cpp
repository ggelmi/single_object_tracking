/* File EKFAdapter.cpp
*//**
*     @file EKFAdapter.h
*     @brief This file implements the extended kalman filter 
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/20/2020
*
*/
#include<cmath>
#include"EKFAdapter.h"
#include<iostream>

namespace stateEstimation
{
    EKFAdapter::EKFAdapter(MotionModelInterface*& motionMdl, MeasurementModelInterface*& measMdl)
        {
            motionModel = motionMdl;
            measModel = measMdl;
        }
    void EKFAdapter:: predictState( Eigen::VectorXd& state,Eigen::MatrixXd& covMatrix) const
        {
            state = motionModel->predictState(state);
            covMatrix = motionModel->getJacobianMatrix(state)*covMatrix*motionModel->getJacobianMatrix(state).transpose() 
                                                                                        + motionModel->getProcessNoiseCovariance();
        }
    
    void EKFAdapter:: updateState( Eigen::VectorXd& state, Eigen::MatrixXd& cov, 
                                    const Eigen::VectorXd& measurement) const
        {
            std::cout << "state : \n" << state << std::endl;
            std::cout << "measurement: \n" << measurement << std::endl;
            Eigen::VectorXd innovation = computeInnovation(state,measurement);
            std::cout << "Innovation : \n" << innovation << std::endl;
            Eigen::MatrixXd innovCovar = computeInnovCovariance(state,cov);
            std::cout << "InnovCovar :\n" << innovCovar << std::endl;
            Eigen::MatrixXd K = computeKalmanGain(state,cov,innovCovar);
            std::cout << "K : \n" << K << std::endl;
            state = state + K*innovation;
            cov = cov - K*innovCovar*K.transpose();
        }
    Eigen::VectorXd EKFAdapter:: computeInnovation(const Eigen::VectorXd& predState,
                                                    const Eigen::VectorXd& measVector) const
        {
            auto n = measVector.rows();
            std::cout << "n :" << n << std::endl;
            Eigen::VectorXd innovation(n);
            innovation << measVector - measModel->getMeasurementVector(predState);  
            return innovation;
        }
    
    Eigen::MatrixXd EKFAdapter:: computeInnovCovariance(const Eigen::VectorXd& predState,const Eigen::MatrixXd& predCov) const
        {
            Eigen::MatrixXd innovationCovar = (measModel->getJacobianMatrix(predState)*predCov*measModel->getJacobianMatrix(predState).transpose()) 
                                                                                            + measModel->getMeasurementNoiseCovariance();
            return innovationCovar;
        }
    
    Eigen::MatrixXd EKFAdapter:: computeKalmanGain(const Eigen::VectorXd& predState,const Eigen::MatrixXd& predCov, const Eigen::MatrixXd& innovCovariance) const
        {
            Eigen::MatrixXd kalmanGain = predCov*measModel->getJacobianMatrix(predState).transpose()*innovCovariance.inverse();
            return kalmanGain;
        }
}