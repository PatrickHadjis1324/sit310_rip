#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState
from std_msgs.msg import Int32  # Change if your encoder uses a different type


class ClosedLoopDrive:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("closed_loop_drive_node", anonymous=True)

        # Publisher for velocity commands
        self.cmd_pub = rospy.Publisher(
            "/mybota002409/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1
        )

        # Subscribers for FSM state and encoders
        rospy.Subscriber(
            "/mybota002409/fsm_node/mode", FSMState, self.fsm_callback, queue_size=1
        )
        rospy.Subscriber(
            "/mybota002409/left_wheel_encoder", Int32, self.left_encoder_callback
        )
        rospy.Subscriber(
            "/mybota002409/right_wheel_encoder", Int32, self.right_encoder_callback
        )

        # Encoder state
        self.left_ticks = 0
        self.right_ticks = 0
        self.initial_left = None
        self.initial_right = None

        # Robot parameters (adjust these!)
        self.ticks_per_rev = 135  # Example value, check your robot
        self.wheel_circumference = 0.21  # Example in meters, check your robot

        # To prevent multiple triggers
        self.has_run = False

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "LANE_FOLLOWING" and not self.has_run:
            rospy.sleep(1)  # Wait for node to be ready
            # Demonstrate at two speeds
            self.drive_straight(1.0, 0.2)  # 1 meter at 0.2 m/s
            rospy.sleep(2)
            self.drive_straight(1.0, -0.2)  # 1 meter backward at 0.2 m/s
            self.has_run = True

    def stop_robot(self):
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = 0.0
        cmd.omega = 0.0
        self.cmd_pub.publish(cmd)

    def drive_straight(self, target_distance, speed):
        # Record initial encoder values
        self.initial_left = self.left_ticks
        self.initial_right = self.right_ticks

        rate = rospy.Rate(10)
        cmd = Twist2DStamped()
        cmd.v = speed
        cmd.omega = 0.0

        rospy.loginfo(
            f"Driving {'forward' if speed > 0 else 'backward'} for {target_distance} meters at {abs(speed)} m/s"
        )

        while not rospy.is_shutdown():
            # Calculate average distance traveled
            left_dist = (self.left_ticks - self.initial_left) * (
                self.wheel_circumference / self.ticks_per_rev
            )
            right_dist = (self.right_ticks - self.initial_right) * (
                self.wheel_circumference / self.ticks_per_rev
            )
            avg_dist = (left_dist + right_dist) / 2.0

            if abs(avg_dist) >= abs(target_distance):
                break

            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.stop_robot()
        rospy.loginfo("Target distance reached. Robot stopped.")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ClosedLoopDrive()
        node.run()
    except rospy.ROSInterruptException:
        pass
