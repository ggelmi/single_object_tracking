/* File UKFAdapter.h
*//**
*     @file UKFAdapter.h
*     @brief This file implements the unscented kalman filter 
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/25/2020
*
*/
#include<cmath>
#include"UKFAdapter.h"
#include<iostream>
#include<iomanip>
#include<Eigen/Cholesky>

namespace stateEstimation
{
    UKFAdapter::UKFAdapter(MotionModelInterface*& motionMdl, MeasurementModelInterface*& measMdl)
        {
            motionModel = motionMdl;
            measModel = measMdl;
        }
    void UKFAdapter:: predictState( Eigen::VectorXd& state,Eigen::MatrixXd& covMatrix) const
        {
            auto n = state.rows();
            unsigned int N = 2*n+1;
            Eigen::MatrixXd sigmaPoints(n,N);
            std::vector<double> sigmaPointWeights(N);
            computeSigmaPoints(state, covMatrix,sigmaPoints,sigmaPointWeights);
            Eigen::VectorXd X(n);
            X.setZero();
            for(int i=0; i<N; i++)
                {
                    
                    X  = X + (motionModel->predictState(sigmaPoints(Eigen::all,i)) *sigmaPointWeights[i] );
                }
            state = X;
            Eigen::MatrixXd P(n,n);
            P.setZero();
            for(int i=0; i<N; i++)
                {
                    Eigen::VectorXd diff = (motionModel->predictState(sigmaPoints(Eigen::all,i)) - state);
                    P  = P + (diff*diff.transpose())*sigmaPointWeights[i];
                }
            P = P + motionModel->getProcessNoiseCovariance();
            covMatrix = P;
            
        }
    void UKFAdapter:: updateState( Eigen::VectorXd& state, Eigen::MatrixXd& covMatrix, 
                                    const Eigen::VectorXd& measurement) const
        {
            auto n = state.rows();
            auto m = measurement.rows();
            unsigned int N = 2*n+1;
            Eigen::MatrixXd sigmaPoints(n,N);
            sigmaPoints.setZero();
            std::vector<double> sigmaPointWeights(N);
            computeSigmaPoints(state, covMatrix,sigmaPoints,sigmaPointWeights);
            std::cout << "sigmaPoints: \n" << sigmaPoints << std::endl;
            // computing predicted measurement using sigma points
            Eigen::VectorXd Yp(m);
            Yp.setZero();
            for(int i=0; i<N; i++)
                {
                    
                    Yp  = Yp + (measModel->getMeasurementVector(sigmaPoints(Eigen::all,i)) *sigmaPointWeights[i] );
                }
            std::cout << "Yp: \n" << Yp << std::endl;
            Eigen::MatrixXd S_K(m,m);
            S_K.setZero();
            std::cout << "S_K: \n" << S_K << std::endl;
            for(int i=0; i<N; i++)
                {
                    Eigen::VectorXd diff  = (measModel->getMeasurementVector(sigmaPoints(Eigen::all,i)) - Yp);
                    std::cout << "h(sigmaP): " << measModel->getMeasurementVector(sigmaPoints(Eigen::all,i))  << std::endl;
                    std::cout << "diff: " << diff << std::endl;
                    std::cout << "weight: " << sigmaPointWeights[i] << std::endl;
                    S_K  = S_K + (diff*diff.transpose()) *sigmaPointWeights[i] ;
                    std::cout << "S_K: \n" << S_K << std::endl;
                }
            S_K = S_K + measModel->getMeasurementNoiseCovariance();
            std::cout << "S_K: \n" << S_K << std::endl;
            Eigen::MatrixXd P_X_Y(n,m);
            P_X_Y.setZero();
            for(int i=0; i<N; i++)
                {
                    Eigen::VectorXd H_minus_Y  = (measModel->getMeasurementVector(sigmaPoints(Eigen::all,i)) - Yp);
                    Eigen::VectorXd SigMP_minus_X  = (sigmaPoints(Eigen::all,i) - state);
                    P_X_Y  = P_X_Y + ((SigMP_minus_X*H_minus_Y.transpose()) *sigmaPointWeights[i]);
                }
            std::cout << "P_X_Y: \n" << P_X_Y << std::endl;
            Eigen::VectorXd innovation = computeInnovation(Yp,measurement);
            state  = state + ( (P_X_Y*S_K.inverse()) * innovation);
            covMatrix = covMatrix - (P_X_Y*S_K.inverse()*P_X_Y.transpose());

        }
    Eigen::VectorXd UKFAdapter:: computeInnovation(const Eigen::VectorXd& predictedMeas,
                                                    const Eigen::VectorXd& measVector) const
        {
            auto n = measVector.rows();
            std::cout << "n :" << n << std::endl;
            Eigen::VectorXd innovation(n);
            innovation << measVector - predictedMeas;  
            return innovation;
        }    
    void UKFAdapter:: computeSigmaPoints(const Eigen::VectorXd& state,const Eigen::MatrixXd& covMatrix,
                                        Eigen::MatrixXd& sigmaPoints, std::vector<double>& sigmaPWeights) const
        {   
            
            auto n = state.rows();
            std::cout << "n :" << n << std::endl;
            double W_O = 1-(n/3.0);
            Eigen::MatrixXd P = covMatrix;
            Eigen::MatrixXd L( P.llt().matrixL() );
            double offset = sqrt(n/(1-W_O));
            double weight = (1-W_O)/(2*n);
            //std::cout << "weight :" << weight << std::endl;
            fill(sigmaPWeights.begin(), sigmaPWeights.end(),weight);
            sigmaPWeights[0] = W_O;
            sigmaPoints(Eigen::all,0) = state;
            for(int i=0; i<n; i++)
                {
                    sigmaPoints(Eigen::all,i+1) = state + offset*L.col(i);
                    sigmaPoints(Eigen::all,i+n+1) = state - offset*L.col(i);
                }

        }
}