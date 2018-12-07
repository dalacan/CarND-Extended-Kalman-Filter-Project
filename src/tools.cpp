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

    if(estimations.size() > 0 && estimations.size() == ground_truth.size()) {
        //accumulate squared residuals
        for(int i=0; i < estimations.size(); ++i){
            // ... your code here
            VectorXd diff = (estimations[i]-ground_truth[i]);
            diff = diff.array()*diff.array();
            rmse += diff;
        }

        //calculate the mean
        rmse = rmse/estimations.size();

        //calculate the squared root
        rmse = rmse.array().sqrt();
    } else {
        cout << "Invalid estimation or ground_truth data" << endl;
    }

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
    //recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    float c1 = std::pow(px,2) + std::pow(py,2);

    if(fabs(c1) < 0.0001){
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        // Increment px and py by a small amount
        px += .001;
        py += .001;

        // Re-evaluate c1
        c1 = std::pow(px,2) + std::pow(py,2);
    }

    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    Hj << (px/c2), (py/c2), 0, 0,
            -(py/c1), (px/c1), 0, 0,
            py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    return Hj;
}
