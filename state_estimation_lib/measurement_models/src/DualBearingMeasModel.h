/* File DualBearingMeasModel.h
*//**
*     @file DualBearingMeasModel.h
*     @brief This file declares the dual bearing measurement model
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/15/2020
*
*/

#ifndef DUAL_BEARING_MEAS_MODEL_H
#define DUAL_BEARING_MEAS_MODEL_H

#include"MeasurementModelInterface.h"

namespace stateEstimation
{
    class DualBearingMeasModel: public  MeasurementModelInterface
    {
        public:
            
            DualBearingMeasModel (const double& sig, const Eigen::VectorXd& sensorOnePos,const Eigen::VectorXd& sensorTwoPos, unsigned int& dim);
            
            ~DualBearingMeasModel(){ };
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
            Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd& currState) const;
            /**
            * @brief Get the measurement noise covariance matrix
            *
            * @return Eigen::MatrixXd
            */
            Eigen::MatrixXd getMeasurementNoiseCovariance();
        private:
            double sigma;       // standard deviation for the sensor-model noise
            unsigned int DIM;   // Dimension of the measurement space
            Eigen::VectorXd sensorOnePosition;  // The sensor one position
            Eigen::VectorXd sensorTwoPosition;  // The sensor two position
    };
}
#endif
