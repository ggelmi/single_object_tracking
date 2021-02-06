/* File CtMeasurementModel.h
*//**
*     @file CtMeasurementModel.h
*     @brief This file declares the coordinated turn measurement model
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/14/2020
*
*/

#ifndef CT_MEASURUMENT_MODEL_H
#define CT_MEASURUMENT_MODEL_H

#include"MeasurementModelInterface.h"

namespace stateEstimation
{
    class CtMeasurementModel: public  MeasurementModelInterface
    {
        public:
            CtMeasurementModel (const double& sig, unsigned int& dim);
            
            ~CtMeasurementModel(){ };
            /**
            * @brief Returns the measurement dimension
            *
            * @return unsigned int
            */
            unsigned int getDimension() const;
            /**
            * @brief Get the measurement vector
            *
            * @param currState : Current state.
            * @return Eigen::VectorXd
            */ 
            Eigen::VectorXd getMeasurementVector(const Eigen::VectorXd& currState);
            /**
            * @brief Get the measurement observation matrix
            *
            * @return Eigen::MatrixXd
            */
            Eigen::MatrixXd getObservationMatrix();
            /**
            * @brief Get the measurement noise covariance matrix
            *
            * @return Eigen::MatrixXd
            */
            Eigen::MatrixXd getMeasurementNoiseCovariance();
        private:
            double sigma;       // standard deviation for the sensor-model noise
            unsigned int DIM;   // Dimension of the measurement space 
    };
}
#endif
