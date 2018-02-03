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
    use_laser_ = false ;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true ;

    // initial state vector
    x_ = VectorXd( 5 );

    // initial covariance matrix
    P_ = MatrixXd( 5, 5 );

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 3.0  ;// 30;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.3 ;//30;

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


    // state vector size
    n_x_    = 5         ;
    // augmented state size
    n_aug_  = 7         ;
    // spreading value
    lambda_ = 3  - n_aug_ ; // => 3-7 = -4



    Xsig_pred_ = MatrixXd( n_x_, n_aug_ * 2 + 1 ) ;
    Xsig_pred_.fill(0.0)  ;

    weights_ = VectorXd( 2*n_aug_ + 1 ) ;
    weights_(0) = lambda_ / ( lambda_ + n_aug_ ) ;
    for ( int i = 1 ; i < 2*n_aug_ + 1 ; i++ ) {
        weights_(i) = 0.5 / ( lambda_ + n_aug_ ) ;
    }

    std::cout << "w" << weights_ << std::endl ;

    // precomputed noise matricies - fixed
    Q_ = MatrixXd( 2, 2 ) ;
    Q_ << ( std_a_ * std_a_ ) , 0 , 0 , ( std_yawdd_ * std_yawdd_ ) ;

    R_radar_ = MatrixXd( 3,  3 ) ;
    R_radar_ << std_radr_*std_radr_, 0,  0,
             0, std_radphi_*std_radphi_, 0,
             0, 0, std_radrd_*std_radrd_;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {


    if ( !is_initialized_ ) {

        // set defualt estimate variance
        P_ <<   1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1 ;

        // std::cout << "P_ = " << std::endl << P_ << std::endl ;
        // we need to setup inital state vector x_

        //

        P_ = P_ * 0.15 ;

        x_.fill( 0.0 ) ;


        if ( use_radar_ &&  meas_package.sensor_type_ == MeasurementPackage::RADAR ) {
            // get radar measurements
            double rho   = meas_package.raw_measurements_(0) ;
            double phi   = meas_package.raw_measurements_(1) ;
            x_ << rho*cos(phi), rho*sin(phi), 0.5, 0.5, 0.5 ;

        } else if ( use_laser_ &&  meas_package.sensor_type_ == MeasurementPackage::LASER ) {

            double px   = meas_package.raw_measurements_(0) ;
            double py   = meas_package.raw_measurements_(1) ;
            // we can only know (px,py) from first measurement.
            x_ << px, py, 0.5 , 0.5 , 0.5 ;
        }
        else
           return ; // we wait until we get an update we can use .



        std::cout << "[0]:" << x_.transpose() << std::endl ;
        // save this time step .
        time_us_ = meas_package.timestamp_ ;
        is_initialized_ = true ;
        return ;
    }


    double dt = ( meas_package.timestamp_ - time_us_ ) / 1.0e6 ; // convert us -> fractional seconds
    time_us_ = meas_package.timestamp_;

    Prediction( dt ) ;

    if ( use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR )
        UpdateRadar( meas_package ) ;
    /*
    else if ( use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER  )
        UpdateLidar( meas_package ) ;
*/
}


static void  FIX_ANGLE( double &a ) {
/*
    while ( a > M_PI )
        a -= M_PI*2.0 ;

    while ( a < -M_PI )
        a += M_PI*2.0 ;
        */

    a = atan2( sin(a) , cos(a) ) ;

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction( double dt ) {

    VectorXd x_aug = VectorXd( n_aug_ ) ;

    // create augmented state vector
    x_aug.head(5) = x_ ;
    x_aug(5) = 0.0     ;
    x_aug(6) = 0.0     ;

    // augment the covaranice matrix P -> P_aug
    MatrixXd P_aug = MatrixXd( n_aug_, n_aug_ )  ;

    P_aug.fill( 0.0 )                            ;
    P_aug.block<5,5>(0,0) = P_                   ;
    P_aug.block<2,2>(5,5) = Q_                   ;


    MatrixXd A = P_aug.llt().matrixL()  ;
    MatrixXd B = A * sqrt( lambda_ + n_aug_  ) ;

    MatrixXd Xsig_aug = MatrixXd( n_aug_ , 2 * n_aug_ + 1 ) ;

    // create augmented sigma points offset by 0 , +B , -B
    Xsig_aug.col( 0 ) = x_aug ;
    for ( int i = 0 ; i < n_aug_ ; i++  ) {
        Xsig_aug.col( i + 1 ) = x_aug + B.col( i ) ;
        Xsig_aug.col( i + n_aug_ + 1 ) = x_aug - B.col( i ) ;
    }


    //predict sigma points
    for ( int i = 0 ; i < 2*n_aug_ + 1 ; i++ ) {
        // extract augmented sigma points
        double px        = Xsig_aug(0,i)  ;
        double py        = Xsig_aug(1,i)  ;
        double v         = Xsig_aug(2,i)  ;
        double yaw       = Xsig_aug(3,i)  ;
        double yaw_d     = Xsig_aug(4,i)  ;
        double nu_a      = Xsig_aug(5,i)  ;
        double nu_yaw_dd = Xsig_aug(6,i)  ;

        // predicted
        double p_px,p_py,p_v,p_yaw,p_yaw_d ;

        if ( fabs(yaw_d) < 1.0e-4 )
        {
            p_px = px + v*dt*cos(yaw) ;
            p_py = py + v*dt*sin(yaw) ;

        }
        else
        {
            p_px = px + v/yaw_d * ( sin(yaw + yaw_d*dt) - sin(yaw)) ;
            p_py = py + v/yaw_d * ( - cos(yaw + yaw_d*dt) + cos(yaw)) ;
        }

        p_v     = v ;
        p_yaw   = yaw + yaw_d*dt ;
        p_yaw_d = yaw_d ;

        //add noise

        p_px    += 0.5*nu_a*dt*dt*cos(yaw) ;
        p_py    += 0.5*nu_a*dt*dt*sin(yaw) ;
        p_v     += nu_a*dt ;
        p_yaw   += 0.5*nu_yaw_dd*dt*dt ;
        FIX_ANGLE( p_yaw ) ;
        p_yaw_d += nu_yaw_dd*dt ;


        Xsig_pred_(0,i) = p_px    ;
        Xsig_pred_(1,i) = p_py    ;
        Xsig_pred_(2,i) = p_v     ;
        Xsig_pred_(3,i) = p_yaw   ;
        Xsig_pred_(4,i) = p_yaw_d ;


    }


    x_.fill( 0.0 )   ;
    for ( int i = 0 ; i < n_aug_*2 + 1 ; i++  ) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i) ;
    }

    FIX_ANGLE( x_(3) );

    //predict state covariance matrix
    P_.fill( 0.0 ) ;
    for ( int i = 0 ; i < n_aug_*2 +1 ; i++ ) {
        VectorXd xd = Xsig_pred_.col(i) - x_ ;
        FIX_ANGLE( xd(3) ) ;
        P_ = P_ + weights_(i) * xd * xd.transpose() ;
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
    **/




}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

    MatrixXd Zsig = MatrixXd( 3 , 2 * n_aug_ + 1  )   ;

    VectorXd z_pred = VectorXd( 3 ) ;
    z_pred.fill( 0.0 ) ;

    for ( int i=0; i< 2*n_aug_+1 ; i++ ) {
        // convert to radar space using these 4
        double px  = Xsig_pred_( 0, i ) ;
        double py  = Xsig_pred_( 1, i ) ;
        double v   = Xsig_pred_( 2, i ) ;
        double yaw = Xsig_pred_( 3, i ) ;

        //
        double rho = sqrt( px*px + py*py);

        Zsig( 0, i ) = rho              ;
        Zsig( 1, i ) = atan2(py,px)     ;
        Zsig( 2, i ) = ( px * cos(yaw) * v  + py * sin(yaw) * v ) / rho ;

        z_pred += weights_(i) * Zsig.col(i) ;

    }


    MatrixXd S  = MatrixXd( 3 , 3 ) ;
    S.fill(0.0) ;
    MatrixXd Tc = MatrixXd( n_x_ , 3 ) ;
    Tc.fill(0.0) ;


    for ( int i=0; i<2*n_aug_+1; i++) {

        VectorXd zd = Zsig.col(i) - z_pred  ;
        FIX_ANGLE(zd(1)) ;

        S =  S + weights_(i) * zd * zd.transpose() ;

        VectorXd xd = Xsig_pred_.col(i) - x_ ;
        FIX_ANGLE(xd(3)) ;

        Tc = Tc + weights_(i) * xd * zd.transpose() ;

    }

    // we should put this in constructor - as constant

    S = S + R_radar_ ;

    // Kalman gain
    MatrixXd K = Tc * S.inverse();

    // get radar measurement
    VectorXd z = VectorXd( 3 ) ;

    z << meas_package.raw_measurements_[0] ,
         meas_package.raw_measurements_[1] ,
         meas_package.raw_measurements_[2] ;


    VectorXd z_res = z - z_pred ;

    FIX_ANGLE( z_res(1) ) ;

    x_ = x_ +  K * z_res ;
    P_ = P_ - K * S * K.transpose() ;

}
