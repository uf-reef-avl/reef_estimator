//
// Created by humberto on 6/7/18.
//

#ifndef REEF_ESTIMATOR_XY_ESTIMATOR_H
#define REEF_ESTIMATOR_XY_ESTIMATOR_H

#include "estimator.h"
#include "../../reef_msgs/include/reef_msgs/dynamics.h"

namespace reef_estimator
{
    class XYEstimator : public Estimator
    {
    public:
        XYEstimator();
        ~XYEstimator();
        /*This class is inherited from Estimator, thus
            it has update and propagate methods already.
         */

        Eigen::Matrix3d C_body_to_body_fixed_frame;
        void nonlinearPropagation(Eigen::Matrix3d &C,double initialAcc, Eigen::Vector3d accel_in_body, float bias_z_in_NED_component);
        void resetLandingState();

        Eigen::Matrix3d C_body_level_to_body_frame ;
        Eigen::Vector3d nonLinearDynamics;
        Eigen::Matrix2d Id;
        double pitch;
        double roll;
        double yaw;
        double pitch_bias;
        double roll_bias;
        double yaw_bias;
        double pitch_est;
        double roll_est;
        double xc;
        double yc;
        double zc;
    };
}



#endif //REEF_ESTIMATOR_XY_ESTIMATOR_H
