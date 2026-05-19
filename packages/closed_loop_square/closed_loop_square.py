#!/usr/bin/env python3

import rospy
import time
import math
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from std_msgs.msg import Int32

TICKS_PER_METER = 350.0


class Drive_Square:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        self.left_ticks = 0
        self.right_ticks = 0
        self.left_ticks_start = 0
        self.right_ticks_start = 0
        self.encoder_received = False

        rospy.init_node("drive_square_node", anonymous=True)

        self.pub = rospy.Publisher(
            "/mybota002409/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1
        )
        rospy.Subscriber(
            "/mybota002409/fsm_node/mode", FSMState, self.fsm_callback, queue_size=1
        )
        rospy.Subscriber(
            "/left_wheel_encoder", Int32, self.left_encoder_callback, queue_size=1
        )
        rospy.Subscriber(
            "/right_wheel_encoder", Int32, self.right_encoder_callback, queue_size=1
        )

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data
        self.encoder_received = True

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data
        self.encoder_received = True

    def wait_for_encoders(self):
        rospy.loginfo("Waiting for encoder data...")
        while not rospy.is_shutdown() and not self.encoder_received:
            rospy.sleep(0.1)

    def get_average_ticks(self):
        return (self.left_ticks + self.right_ticks) / 2.0

    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)
            self.draw_square(side_length=1.0, speed=0.2, angular_speed=1.0)

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def run(self):
        rospy.spin()

    def move_straight(self, distance, speed):
        """Moves the robot straight for a given distance at a given speed using encoder ticks."""
        self.wait_for_encoders()
        start_ticks = self.get_average_ticks()
        direction = 1 if distance >= 0 else -1
        speed = abs(speed) * direction
        target_ticks = abs(distance) * TICKS_PER_METER

        rospy.loginfo(
            f"[move_straight] Requested distance: {distance} m, speed: {speed} m/s, target_ticks: {target_ticks}"
        )

        self.cmd_msg.v = speed
        self.cmd_msg.omega = 0.0

        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_msg)
            current_ticks = self.get_average_ticks()
            traveled_ticks = abs(current_ticks - start_ticks)
            rospy.loginfo_throttle(
                1,
                f"[move_straight] Traveled: {traveled_ticks:.1f} ticks / {target_ticks:.1f} ticks",
            )
            if traveled_ticks >= target_ticks:
                break
            rate.sleep()

        rospy.loginfo("[move_straight] Target distance reached. Stopping robot.")
        self.stop_robot()

    def rotate_in_place(self, angle, angular_speed):
        """
        Rotates the robot in place for a given angle at a given angular speed using encoder ticks.
        Assumes both wheels move in opposite directions for in-place rotation.
        """
        self.wait_for_encoders()
        # Calculate the ticks needed for the desired rotation
        # For a differential drive, the arc length for each wheel is: L = (angle * wheel_base) / 2
        # You need to know your robot's wheel_base (distance between wheels)
        WHEEL_BASE = 0.1  # <-- Set this to your robot's wheel base in meters!
        arc_length = (abs(angle) * WHEEL_BASE) / 2.0
        target_ticks = arc_length * TICKS_PER_METER

        direction = 1 if angle >= 0 else -1
        angular_speed = abs(angular_speed) * direction

        rospy.loginfo(
            f"[rotate_in_place] Requested angle: {angle} rad, angular_speed: {angular_speed} rad/s, target_ticks: {target_ticks}"
        )

        left_start = self.left_ticks
        right_start = self.right_ticks

        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = angular_speed

        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_msg)
            left_delta = abs(self.left_ticks - left_start)
            right_delta = abs(self.right_ticks - right_start)
            avg_delta = (left_delta + right_delta) / 2.0
            rospy.loginfo_throttle(
                1,
                f"[rotate_in_place] Rotated: {avg_delta:.1f} ticks / {target_ticks:.1f} ticks",
            )
            if avg_delta >= target_ticks:
                break
            rate.sleep()

        rospy.loginfo("[rotate_in_place] Target angle reached. Stopping robot.")
        self.stop_robot()

    def draw_square(self, side_length=1.0, speed=0.2, angular_speed=1.0):
        for i in range(4):
            rospy.loginfo(f"[draw_square] Side {i+1}/4: Moving straight.")
            self.move_straight(side_length, speed)
            rospy.sleep(1)
            rospy.loginfo(f"[draw_square] Side {i+1}/4: Rotating 90 degrees.")
            self.rotate_in_place(math.pi / 2, angular_speed)
            rospy.sleep(1)


if __name__ == "__main__":
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
