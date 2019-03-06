//
// Created by humberto on 6/7/18.
//
#include "ros/ros.h"
#include "xy_estimator.h"
#include <eigen3/Eigen/Core>
#include <iostream>

namespace reef_estimator
{
    XYEstimator::XYEstimator() : Estimator()
    {
        dt = 0.002;
        
        //Initial state
        F = Eigen::MatrixXd(6,6);
        F.setZero();

        G = Eigen::MatrixXd(6,6);
        G.setZero();

        //P0 is created to save the initial covariance values. It keeps its value forever.
        P0 = Eigen::MatrixXd(6,6);

        //P is the covariance that the filter propagates.
        P = Eigen::MatrixXd(6,6);
        I = Eigen::MatrixXd(6,6);
        I.setIdentity();
        K = Eigen::MatrixXd(6,2);
        K.setZero();
        z = Eigen::MatrixXd(2,1);
        z << 0.0,
             0.0;
        H = Eigen::MatrixXd(2,6);
        H <<  1, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0;
        xHat0 = Eigen::MatrixXd(6,1);
        xHat = Eigen::MatrixXd(6,1);
        Q = Eigen::MatrixXd(6,6);

        //R0 represents a pseudo covariance that we use to initiate the propagations,
        //once we are in the air, we need to switch the actual R.
        R0 = Eigen::MatrixXd(2,2);
        R = Eigen::MatrixXd(2,2);

        //Beta values for partial update
        betaVector = Eigen::MatrixXd(6,1);
    }

    XYEstimator::~XYEstimator(){}

    void XYEstimator::nonlinearPropagation(Eigen::Matrix3d &C_NED_to_body_frame, double initialAccMagnitude,Eigen::Vector3d accelxyz_in_body_frame, float bias_z_in_NED_component)
    {
        reef_msgs::roll_pitch_yaw_from_rotation321(C_NED_to_body_frame, roll, pitch, yaw);

        pitch_bias = xHat(2);
        roll_bias = xHat(3);
        pitch_est = pitch - pitch_bias;
        roll_est = roll - roll_bias;

        C_body_level_to_body_frame << cos(pitch_est),               0,              -sin(pitch_est),
                                      sin(roll_est)*sin(pitch_est), cos(roll_est),  sin(roll_est)*cos(pitch_est),
                                      cos(roll_est)*sin(pitch_est), -sin(roll_est), cos(roll_est)*cos(pitch_est);

        Eigen::MatrixXd C_body_level_to_body_frame_2x2(2,2);
        C_body_level_to_body_frame_2x2 = C_body_level_to_body_frame.block<2,2>(0,0);

        //Non-linear dynamics.
        Eigen::Vector3d biasAccel_in_body_frame;
        Eigen::Vector3d bias_z_in_body_frame;
        Eigen::Vector3d bias_z_in_NED;

        bias_z_in_NED << 0, 0, bias_z_in_NED_component;

        bias_z_in_body_frame = C_NED_to_body_frame*bias_z_in_NED;
        biasAccel_in_body_frame << xHat(4), xHat(5),bias_z_in_body_frame(2);

        Eigen::Vector3d gravity_in_NED;
        gravity_in_NED << 0, 0, initialAccMagnitude;

        Eigen::Vector3d gravity_in_body_frame;
        gravity_in_body_frame = C_NED_to_body_frame*gravity_in_NED;

        Eigen::Vector3d input_to_system_in_body_frame;
        input_to_system_in_body_frame = accelxyz_in_body_frame + gravity_in_body_frame + biasAccel_in_body_frame;

        // head(n) means take n elements from the vector starting from index 0. head(2) takes element 0 and 1 from the vector.
       /*
        * C_NED_to_body_frame*gravity_in_NED is included in the nonLinearDynamics to remove the gravity reading.
        *
        */
        nonLinearDynamics = C_body_level_to_body_frame.transpose()*(input_to_system_in_body_frame);
        Eigen::Vector3d xy_time_update;
        xy_time_update = nonLinearDynamics*dt;

        xHat << xHat(0) + xy_time_update(0),
                xHat(1) + xy_time_update(1),
                xHat(2),
                xHat(3),
                xHat(4),
                xHat(5);

        //Now compute th Jacobian F for the non-linear dynamics. After this, we compute the predict the covariance.
        Eigen::Matrix2d zeros2x2;
        zeros2x2.setZero();

        //xc and yc are just coefficients used in the Jacobian for clarity.
        xc = input_to_system_in_body_frame(0);
        yc = input_to_system_in_body_frame(1);
        zc = input_to_system_in_body_frame(2);

        Eigen::Matrix2d partialC_partialBiasAttitude;
        partialC_partialBiasAttitude << sin(pitch_est)*xc - sin(roll_est)*cos(pitch_est)*yc -cos(roll_est)*cos(pitch_est)*zc, -cos(roll_est)*sin(pitch_est)*yc + sin(roll_est)*sin(pitch_est)*zc,
                                        0,                                                                                    sin(roll_est)*yc + cos(roll_est)*zc;

        F << zeros2x2, partialC_partialBiasAttitude, C_body_level_to_body_frame_2x2.transpose(),
             zeros2x2, zeros2x2,                     zeros2x2,
             zeros2x2, zeros2x2,                     zeros2x2;

        G << C_body_level_to_body_frame_2x2.transpose(), zeros2x2,            zeros2x2,
                zeros2x2,                                Id.setIdentity(2,2), zeros2x2,
                zeros2x2,                                zeros2x2,            Id.setIdentity(2,2);

        P = P + (F*P.transpose() + P*F.transpose() + G*Q*G.transpose())*dt;
//        P = F*P*F.transpose() + G*Q*G.transpose();
    }

    void XYEstimator::resetLandingState()
    {
        //Reset covariance P
        P = P0;

        //Reset velocity estimate
        xHat = xHat0;
    }

    /* Implementation of propagate and update is not here, but
     * in the base class.
    */
}
