#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;

    //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

    //
    is_initialized_ = false ;


}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {


    if ( !is_initialized_ )
    {

        P_ << 1 , 0 , 0 , 0 , 0 ,
              0 , 1 , 0 , 1 , 0 ,
              0 , 0 , 1 , 0 , 0 ,
              0 , 0 , 0 , 1 , 0 ,
              0 , 0 , 0 , 0 , 1 ;

        // std::cout << "P_ = " << std::endl << P_ << std::endl ;
        // we need to setup inital state vector x_

        x_.fill( 0.0 ) ;

        if ( use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR )
        {
           // get radar measurements
           double rho   = meas_package.raw_measurements_(0) ;
           double phi   = meas_package.raw_measurements_(1) ;
           double phi_d = meas_package.raw_measurements_(0) ;

           // convert into x , y , v ,  yaw , yaw_dot
           //

           // we have no way of calculating v , yaw or yaw_dot from just a single radar measure

           x_ << rho*cos(phi) , rho*sin(phi) , 0 , 0, 0 ;




        }
        else if ( use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER )
        {
            double px   = meas_package.raw_measurements_(0) ;
            double py   = meas_package.raw_measurements_(1) ;


            // we can only Know (px,py) from first measurement.
            x_ << px , py , 0 , 0 , 0 ;
        }
        else
            return ;

        // save this time step .
        time_us_ = meas_package.timestamp_ ;
        is_initialized_ = true ;
        return ;
    }

    double dt = ( meas_package.timestamp_ - time_us_) / 1.0e6 ; // convert us -> fractional seconds
    time_us_ = meas_package.timestamp_;

    Prediction( t ) ;



}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
    TODO:

    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the lidar NIS.
    */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the radar NIS.
    */
}
