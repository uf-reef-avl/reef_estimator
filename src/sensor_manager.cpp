#include "sensor_manager.h"

namespace reef_estimator
{
    SensorManager::SensorManager() : private_nh_("~"), nh_(""), initialized_(false), mocapSwitchOn(false)
    {
        //Mocap override RC channel parameter
        bool enableMocapSwitch = false;
        private_nh_.param<bool>("enable_mocap_switch", enableMocapSwitch, false); ///< Enable switching between sensor readings midflight
        private_nh_.param<int>("mocap_override_channel", mocapOverrideChannel, 4); ///< RC Channel to enable switching

        if (enableMocapSwitch) {
            ROS_WARN_STREAM("Mocap override RC switch enabled on channel " << mocapOverrideChannel);
            rc_subscriber_ = nh_.subscribe("rc_raw", 1, &SensorManager::rcRawCallback, this);
        }

        if (xyzEst.enableMocapXY)
        {
            private_nh_.param<std::string>("mocap_twist_topic", mocapTwistTopic, "mocap_velocity/body_level_frame");
            mocap_twist_subscriber_ = nh_.subscribe(mocapTwistTopic, 1, &SensorManager::mocapTwistCallback, this);
        }

        if (xyzEst.enableMocapZ)
        {
            private_nh_.param<std::string>("mocap_pose_topic", mocapPoseTopic, "mocap_ned");
            mocap_pose_subscriber_ = nh_.subscribe(mocapPoseTopic, 1, &SensorManager::mocapPoseCallback, this);
        }

        if (xyzEst.enableRGBD) {
            private_nh_.param<std::string>("rgbd_twist_topic", rgbdTwistTopic, "rgbd_velocity_body_frame");
            rgbd_twist_subscriber_ = nh_.subscribe(rgbdTwistTopic, 1, &SensorManager::rgbdTwistCallback, this);

        }

        if (xyzEst.enableSonar)
        {
            altimeter_subscriber_ = nh_.subscribe("sonar", 1, &SensorManager::altimeterCallback, this);
            if(xyzEst.debug_mode_)
                range_ned_publisher_ = nh_.advertise<sensor_msgs::Range>("sonar_ned", 1);
        }

        //Initialize subscribers with corresponding callbacks.
        imu_subscriber_ = nh_.subscribe("imu/data", 10, &SensorManager::imuCallback, this);

    }

    SensorManager::~SensorManager(){}

    void SensorManager::imuCallback(const sensor_msgs::ImuConstPtr &msg)
    {
        //Pass the imu message to estimator.
        xyzEst.sensorUpdate(*msg);
    }

    void SensorManager::rcRawCallback(const rosflight_msgs::RCRawConstPtr &msg) {
        //Check for toggled mocap RC switch
        if (!mocapSwitchOn && msg->values[mocapOverrideChannel] > 1500)
        {
            if (xyzEst.enableMocapXY)
            {
                xyzEst.useMocapXY = true;
                ROS_WARN("Mocap XY feedback enabled");
            }

            if (xyzEst.enableMocapZ)
            {
                xyzEst.useMocapZ = true;
                ROS_WARN("Mocap Z feedback enabled");
            }

            mocapSwitchOn = true;
        }
        else if (mocapSwitchOn && msg->values[mocapOverrideChannel] <= 1500)
        {
            if (xyzEst.enableRGBD)
            {
                xyzEst.useMocapXY = false;
                ROS_WARN("Mocap XY feedback disabled");
            }

            if (xyzEst.enableSonar)
            {
                xyzEst.useMocapZ =  false;
                ROS_WARN("Mocap Z feedback disabled");
            }

            mocapSwitchOn = false;
        }
    }

    void SensorManager::mocapPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        xyzEst.mocapUpdate(*msg);
    }

    void SensorManager::mocapTwistCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg)
    {
        xyzEst.mocapUpdate(*msg);
    }

    void SensorManager::rgbdTwistCallback(const reef_msgs::DeltaToVelConstPtr &msg)
    {
      private_nh_.param<bool>("enable_measurements", get_measurements, true);
        if(get_measurements)
          xyzEst.rgbdUpdate(*msg);
        else
          ROS_WARN_STREAM("You stopped the measurements! Why?? ");
    }

void SensorManager::altimeterCallback(const sensor_msgs::RangeConstPtr &msg)
{
    xyzEst.sensorUpdate(*msg);

    if (xyzEst.debug_mode_)
    {
        //Publish the negative range measurement
        range_msg_ned = *msg;
        range_msg_ned.header.stamp = range_msg_ned.header.stamp;
        range_msg_ned.range = -range_msg_ned.range;
        range_ned_publisher_.publish(range_msg_ned);
    }
}



}
