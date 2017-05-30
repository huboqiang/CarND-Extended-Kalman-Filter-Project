#include <math.h>
#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {
}

Tools::~Tools() {
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
        /**
           TODO:
         * Calculate the RMSE here.
         */
        VectorXd rmse(4);
        rmse << 0,0,0,0;
        //std::cerr << estimations.size() << ", " << ground_truth.size() << std::endl;
        if( (estimations.size() == 0)or (estimations.size() != ground_truth.size())) {
                std::cerr << "length of estimation is not valid!" << std::endl;
                return rmse;
        }

        for(int i=0; i<estimations.size(); ++i) {
                VectorXd delta = estimations[i] - ground_truth[i];
                delta = delta.array()*delta.array();
                rmse += delta;
        }
        rmse /= estimations.size();
        rmse = rmse.array().sqrt();
        return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
        /**
           TODO:
         * Calculate a Jacobian here.
         */
        MatrixXd Hj_(3, 4);
        //recover state parameters
        float px = x_state(0);
        float py = x_state(1);
        float vx = x_state(2);
        float vy = x_state(3);
    
        //pre-compute a set of terms to avoid repeated calculation
        float under2 = px * px + py * py;
        float under1 = sqrt(under2);
        float under3 = (under2 * under1);
    
        //check division by zero
        if (fabs(under2) < 0.0001) {
            cout << "CalculateJacobian () - Error - Division by Zero" << endl;
            return Hj_.setZero();
        }
    
        //compute the Jacobian matrix
        Hj_ << (px / under1), (py / under1), 0, 0,
            -(py / under2), (px / under2), 0, 0,
            py * (vx * py - vy * px) / under3, px * (px * vy - py * vx) / under3, px / under1, py / under1;

    
        return Hj_;
}
