#include <eigen3/Eigen/Core>
#include <iostream>
#include "z_estimator.h"

namespace reef_estimator
{
    ZEstimator::ZEstimator() : Estimator()
    {
        // Move P R and x_0 beta vector from yaml
        dt = 0.005;

        //Initial state
        F = Eigen::MatrixXd(3, 3);
        F << 1, dt, 0,
             0, 1,  -dt,
             0, 0,  1;
        B = Eigen::MatrixXd(3,1);
        B <<    0,
                1*dt,
                0;
        G = Eigen::MatrixXd(3,2);
        G <<    0, 0,
                1, 0,
                0, 1;
        K = Eigen::MatrixXd(3,1);
        K << 0,
             0,
             0;
        u = Eigen::MatrixXd(1,1);
        u << 0.0;
        z = Eigen::MatrixXd(1,1);
        z << 0.0;
        I = Eigen::MatrixXd(3,3);
        I.setIdentity();
        H = Eigen::MatrixXd(1,3);
        H << 1, 0, 0;

        xHat = Eigen::MatrixXd(3,1);

        //P0 is created to save the initial covariance values. It keeps its value forever.
        P0 = Eigen::MatrixXd(3,3);
        P0forFlying = Eigen::MatrixXd(3,3);
        P = Eigen::MatrixXd(3,3);
        Q = Eigen::MatrixXd(2,2);
        Q0 = Eigen::MatrixXd(2,2);
        xHat0 = Eigen::MatrixXd(3,1);

        //R0 represents a pseudo covariance that we use to initiate the propagations,
        //once we are in the air, we need to switch the actual R.
        R0 = Eigen::MatrixXd(1,1);
        RforFlying = Eigen::MatrixXd(1,1);
        R = Eigen::MatrixXd(1,1);

        //Beta values for partial update
        betaVector = Eigen::MatrixXd(3,1);
    }

    ZEstimator::~ZEstimator() {}

    void ZEstimator::resetLandingState()
    {
        //Reset covariance P
        P = P0;

        //Reset velocity estimate
        xHat(1) = xHat0(1);
    }

    void ZEstimator::updateLinearModel()
    {
        //Initial state
        F << 1, dt, 0,
                0, 1,  -dt,
                0, 0,  1;
        B <<    0,
                1*dt,
                0;
        G <<    0, 0,
                1, 0,
                0, 1;
        Q = Q0*(dt);
    }

    void ZEstimator::setTakeoffState(bool takeoff)
    {
        if (takeoff) {
            //If flying, latch in estimator takeoff matrices
            R = RforFlying;
            P = P0forFlying;
        } else {
            //If landed, latch in estimator landing measurement coavariance matrix.
            R = R0; //Standard deviation computed from rosbag.
            resetLandingState();
            xHat = xHat0;
        }
    }

    /* Implementation of propagate and update is not here, but
     * in the base class.
    */
}