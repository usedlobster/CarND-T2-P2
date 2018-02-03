#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {


    // start with 0,0,0,0
    VectorXd rmse(4);
    rmse.fill(0.0) ;

    // check we have same estimate to groundestimations[i] - ground_truth[i]
    if ( estimations.size() != ground_truth.size()  )
        std::cerr << "Estimation / Groud Truth Size Mismatch " ;
    else if ( estimations.size() < 1 )
        std::cerr  << "No Estimations " ;
    else {

        // calcualte the squared sum of the differences
        for(int i=0; i < estimations.size(); i++ )
            rmse = rmse +  (VectorXd)( estimations[i] - ground_truth[i] ).array().square() ;

        rmse = rmse / estimations.size()  ;
        // take sqrt of each element..
        rmse = rmse.array().sqrt() ;
    }

    return rmse ;

}
