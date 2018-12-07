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
            VectorXd diff(4);
            diff = (estimations[i]-ground_truth[i]);
            rmse = rmse.array() + diff.array()*diff.array();
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
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float px2py2 = std::pow(px,2) + std::pow(py,2);

    //check division by zero
    if(px2py2 == 0) {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        // Set px2pxy2 to a very small number
        px2py2 = 0.0001;
    }

    float px_div_px2py2 = px/std::sqrt(px2py2);
    float py_div_px2py2 = py/std::sqrt(px2py2);

    //compute the Jacobian matrix
    Hj << px_div_px2py2, py_div_px2py2, 0, 0,
            -py/(px2py2), px/(px2py2), 0, 0,
            (py*(vx*py-vy*px))/std::pow(px2py2, 3/2), (px*(vy*px-vx*py))/std::pow(px2py2, 3/2), px_div_px2py2, py_div_px2py2;
    /*
//        Hj << px/std::sqrt((std::pow(px,2) + std::pow(py,2))), py/std::sqrt((std::pow(px,2) + std::pow(py,2))), 0, 0,
//              -py/(std::pow(px,2) + std::pow(py,2)), px/(std::pow(px,2) + std::pow(py,2)), 0, 0,
//              (py*(vx*py-vy*px))/std::pow((std::pow(px,2) + std::pow(py,2)), 3/2), (px*(vy*px-vx*py))/std::pow((std::pow(px,2) + std::pow(py,2)), 3/2), px/std::sqrt((std::pow(px,2) + std::pow(py,2))), py/std::sqrt((std::pow(px,2) + std::pow(py,2)));
    */

    return Hj;
}
