/* File KFAdapter.cpp
*//**
*     @file KFAdapter.h
*     @brief This file implements the kalman filter 
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/20/2020
*
*/
#include<cmath>
#include"KFAdapter.h"
#include<iostream>

namespace stateEstimation
{
    KFAdapter::KFAdapter(MotionModelInterface*& motionMdl, MeasurementModelInterface*& measMdl)
        {
            motionModel = motionMdl;
            measModel = measMdl;
        }
    void KFAdapter:: predictState( Eigen::VectorXd& state,Eigen::MatrixXd& covMatrix) const
        {
            state = motionModel->predictState(state);
            covMatrix = motionModel->getStateTransitionMatrix(state)*covMatrix*motionModel->getStateTransitionMatrix(state).transpose() 
                                                                                        + motionModel->getProcessNoiseCovariance();
        }
    
    void KFAdapter:: updateState( Eigen::VectorXd& state, Eigen::MatrixXd& cov, 
                                    const Eigen::VectorXd& measurement) const
        {
            Eigen::VectorXd innovation = computeInnovation(state,measurement);
            Eigen::MatrixXd innovCovar = computeInnovCovariance(state,cov);
            Eigen::MatrixXd K = computeKalmanGain(state,cov,innovCovar);
            state = state + K*innovation;
            cov = cov - K*innovCovar*K.transpose();
        }
    Eigen::VectorXd KFAdapter:: computeInnovation(const Eigen::VectorXd& predState,
                                                    const Eigen::VectorXd& measVector) const
        {
            Eigen::VectorXd innovation = measVector - measModel->getObservationMatrix()*predState;
            return innovation;
        }
    
    Eigen::MatrixXd KFAdapter:: computeInnovCovariance(const Eigen::VectorXd& predState,const Eigen::MatrixXd& predCov) const
        {
            Eigen::MatrixXd innovationCovar = (measModel->getObservationMatrix()*predCov*measModel->getObservationMatrix().transpose()) 
                                                                                            + measModel->getMeasurementNoiseCovariance();
            return innovationCovar;
        }
    
    Eigen::MatrixXd KFAdapter:: computeKalmanGain(const Eigen::VectorXd& predState,const Eigen::MatrixXd& predCov, const Eigen::MatrixXd& innovCovariance) const
        {
            Eigen::MatrixXd kalmanGain = predCov*measModel->getObservationMatrix().transpose()*innovCovariance.inverse();
            return kalmanGain;
        }
}