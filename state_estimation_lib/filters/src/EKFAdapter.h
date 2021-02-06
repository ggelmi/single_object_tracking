/* File EKFAdapter.h
*//**
*     @file EKFAdapter.h
*     @brief This file declares the extended kalman filter 
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/20/2020
*
*/

#include"MotionModelInterface.h"
#include"MeasurementModelInterface.h"
#include"FilterInterface.h"

#ifndef EKF_ADAPTER_H
#define EKF_ADAPTER_H

namespace stateEstimation
{
    class EKFAdapter: public FilterInterface
    {
        public:
            EKFAdapter(MotionModelInterface*& motionMdl, MeasurementModelInterface*& measMdl);

            ~EKFAdapter(){};

            void predictState( Eigen::VectorXd& state,Eigen::MatrixXd& covMatrix) const;
            
            void updateState( Eigen::VectorXd& state, Eigen::MatrixXd& cov, const Eigen::VectorXd& measurement) const;
            
        private:
            
            Eigen::VectorXd computeInnovation(const Eigen::VectorXd& predState,const Eigen::VectorXd& measVector) const;
            
            Eigen::MatrixXd computeInnovCovariance(const Eigen::VectorXd& predState,const Eigen::MatrixXd& predCov) const;
            
            Eigen::MatrixXd computeKalmanGain(const Eigen::VectorXd& predState,const Eigen::MatrixXd& predCov, const Eigen::MatrixXd& innovCovariance) const;
            
            MotionModelInterface* motionModel;
            MeasurementModelInterface* measModel;
    };
}
#endif