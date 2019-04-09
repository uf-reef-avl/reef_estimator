/**
 * \class SensorManager
 *
 * \ingroup reef_estimator
 *
 * This class is meant to have all the ROS callbacks and recieve all the sensor readings
 * This class also publishes the sonar reading in the NED frame to verify it's measurements
 *
 * \author $Author: bv Humberto Ramos, William Warke, Prashant Ganesh
 *
 * \version $Revision: 1.0 $
 *
 * \date $Date: 2019/03/05 $
 *
 * Contact: prashant.ganesh@ufl.edu
 *
 */


#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <rosflight_msgs/RCRaw.h>

#include <reef_msgs/DeltaToVel.h>

#include "xyz_estimator.h"
#include "z_estimator.h"

namespace reef_estimator {
    class SensorManager {
    private:

        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        ros::Subscriber imu_subscriber_;
        ros::Subscriber altimeter_subscriber_;
        ros::Subscriber rc_subscriber_;
        ros::Subscriber mocap_pose_subscriber_;
        ros::Subscriber mocap_twist_subscriber_;
        ros::Subscriber rgbd_twist_subscriber_;
        ros::Subscriber rgbd_sigmas_subscriber_;

        ros::Publisher range_ned_publisher_;

        std::string mocapPoseTopic, mocapTwistTopic;
        std::string rgbdTwistTopic;

        XYZEstimator xyzEst;
        sensor_msgs::Imu imu_msg;
        sensor_msgs::Range range_msg;
        sensor_msgs::Range range_msg_ned; //used in debug mode

        void isFlyingCallback(const std_msgs::BoolConstPtr &msg);
        void imuCallback(const sensor_msgs::ImuConstPtr &msg);
        void altimeterCallback(const sensor_msgs::RangeConstPtr &msg);
        void rcRawCallback(const rosflight_msgs::RCRawConstPtr &msg);
        void mocapPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        void mocapTwistCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg);
        void rgbdTwistCallback(const reef_msgs::DeltaToVelConstPtr &msg);

        bool initialized_;
        bool imuCalibrated;
        bool get_measurements;

        //Mocap override variables
        bool mocapSwitchOn;
        int mocapOverrideChannel;

    public:
        SensorManager();
        ~SensorManager();



        };
}
#endif
