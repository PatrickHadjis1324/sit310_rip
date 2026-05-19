#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray


class Target_Follower:
    def __init__(self):

        # Initialize ROS node
        rospy.init_node("target_follower_node", anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)

        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher(
            "/mybota002409/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1
        )
        rospy.Subscriber(
            "/mybota002409/apriltag_detector_node/detections",
            AprilTagDetectionArray,
            self.tag_callback,
            queue_size=1,
        )
        ################################################################

        rospy.spin()  # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        self.move_robot(msg.detections)

    # Stop Robot before node has shut down. This ensures the robot keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0  # No forward movement

        if len(detections) == 0:
            # SEEK: Spin in place to find an object
            cmd_msg.omega = 0.5
            rospy.loginfo("Seeking object: omega=%.2f", cmd_msg.omega)
        else:
            # LOOK: Turn to face the first detected object
            y = detections[0].transform.translation.y
            cmd_msg.omega = (
                -1.5 * y
            )  # proportional control, adjust -1.5 if too fast/slow
            rospy.loginfo("Tracking object at y=%.3f, omega=%.2f", y, cmd_msg.omega)

        self.cmd_vel_pub.publish(cmd_msg)


if __name__ == "__main__":
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
