/* File GaussianDensity.h
*//**
*     @file GaussianDensity.h
*     @brief This file declares the gaussian density function
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 02/08/2021
*
*/

#include"FilterInterface.h"
#include"MeasurementModelInterface.h"

#ifndef GAUSSIAN_DENSITY_H
#define GAUSSIAN_DENSITY_H

using namespace stateEstimation;

namespace sotTracker
{
    class GaussianDensity
    {
        public:
            GaussianDensity(FilterInterface*& filterIntfc, MeasurementModelInterface*& measMdl);

            void predict(Eigen::VectorXd& state,Eigen::MatrixXd& covMatrix);

            void update(Eigen::VectorXd& state,Eigen::MatrixXd& cov,const Eigen::VectorXd& measurement);

        private:
            FilterInterface* filter;
            MeasurementModelInterface* measModel;
    };
}
#endif