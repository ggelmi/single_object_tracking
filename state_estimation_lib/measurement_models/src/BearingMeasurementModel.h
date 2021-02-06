/* File BearingMeasurementModel.h
*//**
*     @file BearingMeasurementModel.h
*     @brief This file declares the bearing only measurement model
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/15/2020
*
*/

#ifndef BEARING_MEASURUMENT_MODEL_H
#define BEARING_MEASURUMENT_MODEL_H

#include"MeasurementModelInterface.h"

namespace stateEstimation
{
    class BearingMeasurementModel: public  MeasurementModelInterface
    {
        public:
            BearingMeasurementModel (const double& sig, const Eigen::VectorXd& sensorPos, unsigned int& dim);
            
            ~BearingMeasurementModel(){ };
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
            * @brief Get the measurement jacobian matrix
            *
            * @param currState : Current state.
            * @return Eigen::MatrixXd
            */
            Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd& currState);
            /**
            * @brief Get the measurement noise covariance matrix
            *
            * @return Eigen::MatrixXd
            */
            Eigen::MatrixXd getMeasurementNoiseCovariance();
        private:
            double sigma;       // standard deviation for the sensor-model noise
            unsigned int DIM;   // Dimension of the measurement space
            Eigen::VectorXd sensorPosition;     // The sensor position
    };
}
#endif

