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


    n_x_ = 5           ; // state vector size
    n_aug_ = 7         ; // augmented state size
    lambda_ = 3  - n_x_ ; // spreading value

    weights_ = VectorXd( 2*n_aug_ + 1 ) ;
    weights_(0) = lambda_ / ( lambda_ + n_aug_ ) ;
    for ( int i = 1 ; i < 2*n_aug_ + 1 ; i++ ) {
        weights_(i) = 1.0 / ( 2 *( lambda_ + n_aug_)) ;
    }

    Q_ = MatrixXd( 2, 2 ) ;
    Q_ << std_a_ * std_a_, 0,  0, std_yawdd_ * std_yawdd_ ;


}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {


    if ( !is_initialized_ ) {

        // set defualt estimate variance
        P_ << 1, 0, 0, 0, 0,
              0, 1, 0, 1, 0,
              0, 0, 1, 0, 0,
              0, 0, 0, 1, 0,
              0, 0, 0, 0, 1 ;

        // std::cout << "P_ = " << std::endl << P_ << std::endl ;
        // we need to setup inital state vector x_

        x_.fill( 0.0 ) ;

        if ( use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR ) {
            // get rada*r measurements
            double rho   = meas_package.raw_measurements_(0) ;
            double phi   = meas_package.raw_measurements_(1) ;

            // convert into x , y , v ,  yaw , yaw_do t
            //
            // we have no way of calculating v , yaw or yaw_dot from just a single radar measure
            //

            x_ << rho*cos(phi), rho*sin(phi), 0, 0, 0 ;

        } else if ( use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER ) {
            double px   = meas_package.raw_measurements_(0) ;
            double py   = meas_package.raw_measurements_(1) ;
            // we can only know (px,py) from first measurement.
            x_ << px, py, 0, 0, 0 ;
        } else
            return ;

        // save this time step .
        time_us_ = meas_package.timestamp_ ;
        is_initialized_ = true ;
        return ;
    }

    double dt = ( meas_package.timestamp_ - time_us_) / 1.0e6 ; // convert us -> fractional seconds
    time_us_ = meas_package.timestamp_;

    Prediction( dt ) ;

    if ( use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR )
        UpdateRadar( meas_package ) ;
    else if ( use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER  )
        UpdateLidar( meas_package ) ;

}


double static inline FIX_ANGLE( double a)
{
    while ( a > M_PI )
        a-=M_PI*2.0 ;
    while ( a < -M_PI )
        a+=M_PI*2.0 ;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt ) {
    /**
    TODO:

    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */

    // create augmented state .
    VectorXd x_aug = VectorXd( n_aug_ ) ;
    x_aug << x_, 0, 0 ;

    // create augment covariance matrix
    MatrixXd P_aug = MatrixXd( n_aug_, n_aug_ ) ;

    P_aug.block<5,5>( 0, 0 ) = P_ ;
    P_aug.block<2,2>( 5, 5 ) = Q_ ;


    // A = square-root of P_aug

    MatrixXd A = P_aug.llt().matrixL();


    // B = A * sqrt(lambda+n_aug)
    MatrixXd B = MatrixXd( n_aug_, n_aug_ ) ;


    B = A * sqrt( lambda_ + n_aug_ ) ;


    // create sigma points

    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    Xsig_aug.col(0) = x_aug ;

    for (int i = 0; i < n_aug_; i++) {
        Xsig_aug.col(i + 1)          = x_aug + B.col(i) ;
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - B.col(i) ;
    }


    Xsig_pred_ = MatrixXd( n_x_ , 2*n_aug_ + 1 ) ;

    // for each sigma point - predict
    //
    for ( int i = 0 ; i < 2*n_aug_ + 1 ; i++ ) {
        // extract for each columns elements
        double px            = Xsig_aug( 0, i ) ;
        double py            = Xsig_aug( 1, i ) ;
        double v             = Xsig_aug( 2, i ) ;
        double yaw           = Xsig_aug( 3, i ) ;
        double yawdot        = Xsig_aug( 4, i ) ;
        double nu_a          = Xsig_aug( 5, i ) ;
        double nu_yawdotdot  = Xsig_aug( 6, i ) ;

        // prediction

        double p_px, p_py, p_v, p_yaw, p_yawdot ;

        if ( fabs( yawdot) < 1e-4 ) {
            p_px = px + v * dt * cos(yaw) ;
            p_py = py + v * dt * sin(yaw) ;
        } else {
            p_px = px + v / yawdot * ( sin( yaw + yawdot * dt ) - sin( yaw )) ;
            p_py = py + v / yawdot * ( cos( yaw ) - cos( yaw + yawdot * dt )) ;

        }

        p_v = v                   ;
        p_yaw = yaw + yawdot * dt ;
        p_yawdot = yawdot         ;


        // add noise to prediction

        p_px     += 0.5 * nu_a * dt * dt * cos( yaw ) ;
        p_py     += 0.5 * nu_a * dt * dt * sin( yaw ) ;
        p_v      += nu_a * dt                         ;
        p_yaw    += 0.5 * nu_yawdotdot * dt * dt      ;
        p_yawdot += nu_yawdotdot * dt                 ;

        //

        Xsig_pred_.col(i) << p_px , p_py , p_v , p_yaw , p_yawdot ;


    }

    // create new state vector from weighted average of sigma points
    // to get state mean

    x_.fill( 0.0 ) ;
    for ( int i = 0 ; i < 2*n_aug_ + 1 ; i++ ) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i) ;

    }

    x_(3) = FIX_ANGLE( x_(3) ) ;

    // create new state covariance matrix estimate

    P_.fill( 0.0 ) ;
    for ( int i =0 ; i< 2*n_aug_ + 1 ; i++ ) {
        VectorXd x_d = Xsig_pred_.col(i) - x_ ;

        x_d( 3 ) = FIX_ANGLE( x_d(3) ) ;

        P_ = P_ + weights_( i ) * x_d * x_d.transpose() ;


    }


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
