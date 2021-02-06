/* File KFAdapter.h
*//**
*     @file KFAdapter.h
*     @brief This file declares the kalman filter 
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/20/2020
*
*/

#include"MotionModelInterface.h"
#include"MeasurementModelInterface.h"
#include"FilterInterface.h"

#ifndef KF_ADAPTER_H
#define KF_ADAPTER_H

namespace stateEstimation
{
    class KFAdapter: public FilterInterface
    {
        public:
            KFAdapter(MotionModelInterface*& motionMdl, MeasurementModelInterface*& measMdl);

            ~KFAdapter(){};

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