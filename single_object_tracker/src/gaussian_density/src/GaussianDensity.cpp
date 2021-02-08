#include<cmath>
#include"GaussianDensity.h"
#include<iostream>

namespace sotTracker
{
    GaussianDensity::GaussianDensity(FilterInterface*& filterIntfc, MeasurementModelInterface*& measMdl)
        {
            std::cout << "Inside the gaussian density constructor"<< std::endl;
            filter = filterIntfc;
            measModel = measMdl;
        }
    void GaussianDensity:: predict(Eigen::VectorXd& state,Eigen::MatrixXd& covMatrix)
        {
            filter->predictState(state,covMatrix);
        }
    void GaussianDensity:: update(Eigen::VectorXd& state,Eigen::MatrixXd& covMatrix,const Eigen::VectorXd& measurement)
        {
            filter->updateState(state,covMatrix,measurement);
        }

}


