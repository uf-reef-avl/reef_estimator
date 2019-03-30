#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from message_filters import ApproximateTimeSynchronizer, Subscriber
from reef_msgs.msg import DeltaToVel
from reef_msgs.msg import XYZDebugEstimate
from reef_msgs.msg import SyncVerifyEstimates
from reef_msgs.msg import SyncEstimateError

class verifyEstimates():
    def __init__(self):

        print("Verifying Estimates")
        self.syn_pub = rospy.Publisher('sync_estimates', SyncVerifyEstimates, queue_size=10)
        self.error_pub = rospy.Publisher('estimate_error', SyncEstimateError, queue_size=10)

        estimate_subs = Subscriber("xyz_debug_estimates", XYZDebugEstimate)
        truth_subs = Subscriber("mocap/velocity/body_level_frame", TwistWithCovarianceStamped)
        rgbd_subs = Subscriber("rgbd_velocity/body_level_frame", DeltaToVel)
        mocap_pose_subs = Subscriber("pose_stamped", PoseStamped)

        self.sync_msg = SyncVerifyEstimates()
        self.estimate_error = SyncEstimateError()

        self.multiplier = 1
        self.sonar_offset = 0
        use_sonar = False
        self.multiplier = rospy.get_param("SD_Multiplied")
        self.sonar_offset = rospy.get_param("Z_offset")
        use_sonar = rospy.get_param("use_sonar")
        print(use_sonar)

        if use_sonar:
            sonar_subs = Subscriber("sonar_ned", Range)
            approx_sync_sonar = ApproximateTimeSynchronizer([estimate_subs, truth_subs, rgbd_subs, mocap_pose_subs,sonar_subs], queue_size=5, slop=0.1)
            approx_sync_sonar.registerCallback(self.callbackSonar)
        else:
            approx_sync_wo_sonar = ApproximateTimeSynchronizer([estimate_subs, truth_subs, rgbd_subs, mocap_pose_subs], queue_size=5, slop=0.1)
            approx_sync_wo_sonar.registerCallback(self.callbackNOSonar)


    def callbackSonar(self, est_msg, mocap_vel, rgbd_msg, mocap_pose, sonar_msg):

        # print("in Callback")

        node_id = 1

        self.sync_msg.header = est_msg.header
        self.sync_msg.node_id = node_id

        self.sync_msg.estimatedVelocity.x = est_msg.xy_plus.x_dot
        self.sync_msg.estimatedVelocity.y = est_msg.xy_plus.y_dot
        self.sync_msg.estimatedVelocity.z = est_msg.z_plus.z_dot

        self.sync_msg.velocity_sigma_plus.x = est_msg.xy_plus.sigma_plus[0]
        self.sync_msg.velocity_sigma_plus.y = est_msg.xy_plus.sigma_plus[1]
        self.sync_msg.velocity_sigma_plus.z = est_msg.z_plus.sigma_plus[1]

        self.sync_msg.velocity_sigma_minus.x = est_msg.xy_plus.sigma_minus[0]
        self.sync_msg.velocity_sigma_minus.y = est_msg.xy_plus.sigma_minus[1]
        self.sync_msg.velocity_sigma_minus.z = est_msg.z_plus.sigma_minus[1]

        self.sync_msg.trueVelocity.x = mocap_vel.twist.twist.linear.x
        self.sync_msg.trueVelocity.y = mocap_vel.twist.twist.linear.y
        self.sync_msg.trueVelocity.z = mocap_vel.twist.twist.linear.z - self.sonar_offset

        self.sync_msg.measuredVelocity.x = rgbd_msg.vel.twist.twist.linear.x
        self.sync_msg.measuredVelocity.y = rgbd_msg.vel.twist.twist.linear.y
        self.sync_msg.measuredVelocity.z = rgbd_msg.vel.twist.twist.linear.z

        self.sync_msg.zEstimate = est_msg.z_plus.z
        self.sync_msg.zTrue = mocap_pose.pose.position.z
        self.sync_msg.zMeasured = sonar_msg.range
        self.sync_msg.z_sigma_plus = est_msg.z_plus.sigma_plus[0]
        self.sync_msg.z_sigma_minus = est_msg.z_plus.sigma_minus[0]
        self.syn_pub.publish(self.sync_msg)

        estimate_error = SyncEstimateError()
        estimate_error.header = est_msg.header
        estimate_error.node_id = node_id
        estimate_error.velocityError.x = (est_msg.xy_plus.x_dot - mocap_vel.twist.twist.linear.x)
        estimate_error.velocityError.y = est_msg.xy_plus.y_dot - mocap_vel.twist.twist.linear.y
        estimate_error.velocityError.z = est_msg.z_plus.z_dot - mocap_vel.twist.twist.linear.z

        estimate_error.velocitySDPlus.x = (est_msg.xy_plus.sigma_plus[0] - est_msg.xy_plus.x_dot)
        estimate_error.velocitySDPlus.y = (est_msg.xy_plus.sigma_plus[1] - est_msg.xy_plus.y_dot)
        estimate_error.velocitySDPlus.z = (est_msg.z_plus.sigma_plus[1] - est_msg.z_plus.z_dot)

        estimate_error.velocitySDMinus.x = -1 * (est_msg.xy_plus.sigma_plus[0] - est_msg.xy_plus.x_dot)
        estimate_error.velocitySDMinus.y = -1 * (est_msg.xy_plus.sigma_plus[1] - est_msg.xy_plus.y_dot)
        estimate_error.velocitySDMinus.z = -1 * (est_msg.z_plus.sigma_plus[1] - est_msg.z_plus.z_dot)

        estimate_error.zError = est_msg.z_plus.z - mocap_pose.pose.position.z + self.sonar_offset
        estimate_error.zSDPlus = 1 * (est_msg.z_plus.sigma_plus[0] - est_msg.z_plus.z )
        estimate_error.zSDMinus = -1 * (est_msg.z_plus.sigma_plus[0] - est_msg.z_plus.z)

        self.error_pub.publish(estimate_error)
        node_id = node_id +1

    def callbackNOSonar(self, est_msg, mocap_vel, rgbd_msg, mocap_pose):

        # print("in Callback")

        node_id = 1

        self.sync_msg.header = est_msg.header
        self.sync_msg.node_id = node_id

        self.sync_msg.estimatedVelocity.x = est_msg.xy_plus.x_dot
        self.sync_msg.estimatedVelocity.y = est_msg.xy_plus.y_dot
        self.sync_msg.estimatedVelocity.z = est_msg.z_plus.z_dot

        self.sync_msg.velocity_sigma_plus.x = est_msg.xy_plus.sigma_plus[0]
        self.sync_msg.velocity_sigma_plus.y = est_msg.xy_plus.sigma_plus[1]
        self.sync_msg.velocity_sigma_plus.z = est_msg.z_plus.sigma_plus[1]

        self.sync_msg.velocity_sigma_minus.x = est_msg.xy_plus.sigma_minus[0]
        self.sync_msg.velocity_sigma_minus.y = est_msg.xy_plus.sigma_minus[1]
        self.sync_msg.velocity_sigma_minus.z = est_msg.z_plus.sigma_minus[1]

        self.sync_msg.trueVelocity.x = mocap_vel.twist.twist.linear.x
        self.sync_msg.trueVelocity.y = mocap_vel.twist.twist.linear.y
        self.sync_msg.trueVelocity.z = mocap_vel.twist.twist.linear.z

        self.sync_msg.measuredVelocity.x = rgbd_msg.vel.twist.twist.linear.x
        self.sync_msg.measuredVelocity.y = rgbd_msg.vel.twist.twist.linear.y
        self.sync_msg.measuredVelocity.z = rgbd_msg.vel.twist.twist.linear.z

        self.sync_msg.zEstimate = est_msg.z_plus.z
        self.sync_msg.zTrue = mocap_pose.pose.position.z
        self.sync_msg.zMeasured = mocap_pose.pose.position.z
        self.sync_msg.z_sigma_plus = est_msg.z_plus.sigma_plus[0]
        self.sync_msg.z_sigma_minus = est_msg.z_plus.sigma_minus[0]
        self.syn_pub.publish(self.sync_msg)

        estimate_error = SyncEstimateError()
        estimate_error.header = est_msg.header
        estimate_error.node_id = node_id
        estimate_error.velocityError.x = (est_msg.xy_plus.x_dot - mocap_vel.twist.twist.linear.x)
        estimate_error.velocityError.y = est_msg.xy_plus.y_dot - mocap_vel.twist.twist.linear.y
        estimate_error.velocityError.z = est_msg.z_plus.z_dot - mocap_vel.twist.twist.linear.z

        estimate_error.velocitySDPlus.x = 1 * (est_msg.xy_plus.sigma_plus[0] - est_msg.xy_plus.x_dot)
        estimate_error.velocitySDPlus.y = 1 * (est_msg.xy_plus.sigma_plus[1] - est_msg.xy_plus.y_dot)
        estimate_error.velocitySDPlus.z = 1 * (est_msg.z_plus.sigma_plus[1] - est_msg.z_plus.z_dot)

        estimate_error.velocitySDMinus.x = -1 * (est_msg.xy_plus.sigma_plus[0] - est_msg.xy_plus.x_dot)
        estimate_error.velocitySDMinus.y = -1 * (est_msg.xy_plus.sigma_plus[1] - est_msg.xy_plus.y_dot)
        estimate_error.velocitySDMinus.z = -1 * (est_msg.z_plus.sigma_plus[1] - est_msg.z_plus.z_dot)

        estimate_error.zError = est_msg.z_plus.z - mocap_pose.pose.position.z
        estimate_error.zSDPlus = 1 * (est_msg.z_plus.sigma_plus[0] - est_msg.z_plus.z )
        estimate_error.zSDMinus = -1 * (est_msg.z_plus.sigma_plus[0] - est_msg.z_plus.z)

        self.error_pub.publish(estimate_error)
        node_id = node_id +1




if __name__ == '__main__':
    rospy.init_node("verify_estimates", anonymous=False)
    try:
        verifyObj = verifyEstimates()
    except rospy.ROSInterruptException: pass
    rospy.spin()






