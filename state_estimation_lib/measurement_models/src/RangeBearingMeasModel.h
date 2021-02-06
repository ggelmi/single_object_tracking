/* File RangeBearingMeasModel.h
*//**
*     @file RangeBearingMeasModel.h
*     @brief This file declares the range bearing measurement model
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/15/2020
*
*/

#ifndef RANGE_BEARING_MEAS_MODEL_H
#define RANGE_BEARING_MEAS_MODEL_H

#include"MeasurementModelInterface.h"

namespace stateEstimation
{
    class RangeBearingMeasModel: public  MeasurementModelInterface
    {
        public:
            RangeBearingMeasModel (const double& sig_r,const double& sig_b, const Eigen::VectorXd& sensorPos, unsigned int& dim);
        
            ~RangeBearingMeasModel(){ };
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
            double sigma_r;     // standard deviation for the sensor-model noise - range
            double sigma_b;     // standard deviation for the sensor-model noise - bearing
            unsigned int DIM;   // Dimension of the measurement space
            Eigen::VectorXd sensorPosition; // The sensor position
    };
}
#endif

