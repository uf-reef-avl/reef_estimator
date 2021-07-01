//
// Created by humberto on 6/4/18.
//
#ifndef REEF_ESTIMATOR_XYZ_ESTIMATOR_H
#define REEF_ESTIMATOR_XYZ_ESTIMATOR_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Core>
#include <tf2_eigen/tf2_eigen.h>

//Messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <reef_msgs/XYZDebugEstimate.h>
#include <reef_msgs/XYZEstimate.h>
#include <reef_msgs/DeltaToVel.h>

#include "z_estimator.h"
#include "xy_estimator.h"
#include <reef_msgs/matrix_operation.h>
#include <reef_msgs/dynamics.h>

#include <iostream>

#define ACC_SAMPLE_SIZE 20
#define ACC_TAKEOFF_VARIANCE 0.2
#define SONAR_SAMPLE_SIZE 40
#define SONAR_TAKEOFF_VARIANCE 0.000002
#define SONAR_OFFSET 0.010

namespace reef_estimator
{
    class XYZEstimator
    {
    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;
        ros::Publisher state_publisher_;
        ros::Publisher debug_state_publisher_;
        ros::Publisher is_flying_publisher_;

        //Estimator enable/disable variables
        bool enableXY;
        bool enableZ;
        bool imuIsFromPixhawk;

        //Instantiate a ZEstimator
        ZEstimator zEst;

        //Instantiate an XYEstimator
        XYEstimator xyEst;

        //Message holders
        reef_msgs::XYZEstimate xyzState;
        reef_msgs::XYZDebugEstimate xyzDebugState;

        int numberOfPropagations;

        //Accelerometer calibration variables
        bool accInitialized;
        int accInitSampleCount;
        double initialAccMagnitude;
        geometry_msgs::Vector3 accSampleAverage;
        double last_time_stamp;
        double mahalanobis_distance_sonar;
        double mahalanobis_distance_rgbd_xy_;
        double mahalanobis_distance_mocap_z;
        double mahalanobis_distance_mocap_xy_;
        double dt;
        bool enable_partial_update;
        int sigma_mulitplier;


      //Takeoff detection variables
        std_msgs::Bool takeoffState;
        bool accTakeoffState, sonarTakeoffState;
        int numAccSamples, numSonarSamples;
        double accSamples[ACC_SAMPLE_SIZE], sonarSamples[SONAR_SAMPLE_SIZE];
        double accMean, accVariance, sonarMean, sonarVariance;
        bool newRgbdMeasurement, newSonarMeasurement;
        int rgbdCounter;

        Eigen::Matrix3d C_NED_to_body_frame;
        Eigen::Vector3d accelxyz_in_body_frame ;
        Eigen::Vector3d accelxyz_in_NED_frame ;
        Eigen::VectorXd range;
        Eigen::VectorXd expected_measurement;
        Eigen::Vector3d Mahalanobis_D_hat_square;
        Eigen::Vector3d Mahalanobis_D_hat;
        Eigen::Vector2d measurement;
        Eigen::Vector2d expected_rgbd;

        double beta_0;
        Eigen::MatrixXd xySigma;
        Eigen::Vector3d zSigma;
        
        void checkTakeoffState(double accMagnitude);
        void publishEstimates();
        void saveMinusState();

        void initializeAcc(geometry_msgs::Vector3 acc);
        bool chi2Accept(float range_measurement);
        bool chi2AcceptRgbd(geometry_msgs::TwistWithCovarianceStamped twist_msg);

        bool chi2AcceptMocapXY(geometry_msgs::TwistWithCovarianceStamped twist_msg);
        bool chi2AcceptMocapZ(float z_mocap_ned);
        
        bool hypothesisAccept(float range_measurement);
        float correctRange(float range_measurement);

    public:
        XYZEstimator();
        ~XYZEstimator();

        bool enableMocapXY, enableMocapZ;
        bool enableRGBD, enableSonar;
        bool useMocapXY, useMocapZ;
        bool debug_mode_;

        void sensorUpdate(sensor_msgs::Imu imu);
        void sensorUpdate(sensor_msgs::Range range_msg);
        void mocapUpdate(geometry_msgs::PoseStamped pose_msg);
        void mocapUpdate(geometry_msgs::TwistWithCovarianceStamped twist_msg);
        void rgbdUpdate(reef_msgs::DeltaToVel twist_msg);

        sensor_msgs::Imu transformImuToNed(sensor_msgs::Imu imu);
    };

    double getVectorMagnitude(double x, double y, double z);
}

#endif //REEF_ESTIMATOR_XYZ_ESTIMATOR_H
