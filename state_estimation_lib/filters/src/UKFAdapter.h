/* File UKFAdapter.h
*//**
*     @file UKFAdapter.h
*     @brief This file declares the unscented kalman filter 
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/25/2020
*
*/

#include"MotionModelInterface.h"
#include"MeasurementModelInterface.h"
#include"FilterInterface.h"
#include<vector>

#ifndef UKF_ADAPTER_H
#define UKF_ADAPTER_H

namespace stateEstimation
{
    class UKFAdapter: public FilterInterface
    {
        public:
            UKFAdapter(MotionModelInterface*& motionMdl, MeasurementModelInterface*& measMdl);

            ~UKFAdapter(){};

            void predictState( Eigen::VectorXd& state,Eigen::MatrixXd& covMatrix) const;
            
            void updateState( Eigen::VectorXd& state, Eigen::MatrixXd& cov, const Eigen::VectorXd& measurement) const;
            
        private:
            
            Eigen::VectorXd computeInnovation(const Eigen::VectorXd& predState,const Eigen::VectorXd& measVector) const;
            
            void computeSigmaPoints(const Eigen::VectorXd& state,const Eigen::MatrixXd& covMatrix,Eigen::MatrixXd& sigmaPoints, std::vector<double>& sigmaPointweights) const;
            
            MotionModelInterface* motionModel;
            MeasurementModelInterface* measModel;
    };
}
#endif