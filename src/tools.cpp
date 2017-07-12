#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size()
       || estimations.size() == 0){
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        VectorXd res = estimations[i] - ground_truth[i];

        res = res.array() * res.array();

        rmse += res;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

/**
* A helper method to calculate cartesian coordinates from polar.
*/
VectorXd Tools::PolarToCartesian(const float& ro, const float& phi)
{
    /**
    TODO:
      * Calculate cartesian coordinates here.
    */
    VectorXd coords(2);

    coords << ro * cos(phi), ro * sin(phi);

    return coords;
}

void Tools::NormAngle(double *angle)
{
    /**
    TODO:
      * Normalize angle here.
    */
    *angle = fmod(*angle, 2 * M_PI);
}

//void Tools::NormAngle(double *angle) {
//    /**
//    TODO:
//      * Normalize angle here.
//    */
//    while (*angle > M_PI) *angle -= 2. * M_PI;
//    while (*angle < -M_PI) *angle += 2. * M_PI;
//}

