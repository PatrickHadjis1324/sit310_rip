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
            # self.draw_square(side_length=1.0, speed=0.2, angular_speed=1.0)
            self.move_straight(distance=1.0, speed=0.2)
            rospy.sleep(1)
            self.rotate_in_place(math.pi, 1.0)
            rospy.sleep(1)

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def run(self):
        rospy.spin()

    def move_straight(self, distance, speed):
        start_ticks = self.get_average_ticks()
        target_ticks = distance * TICKS_PER_METER
        self.cmd_msg.v = speed
        self.cmd_msg.omega = 0.0
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.cmd_msg)
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
        self.stop_robot()

    def rotate_in_place(self, angle, angular_speed, wheel_base=0.1):
        arc_length = (wheel_base / 2.0) * abs(angle)
        target_ticks = arc_length * TICKS_PER_METER

        left_start = self.left_ticks
        right_start = self.right_ticks

        direction = 1 if angle > 0 else -1
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = direction * abs(angular_speed)
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.cmd_msg)

        while not rospy.is_shutdown():
            left_delta = abs(self.left_ticks - left_start)
            right_delta = abs(self.right_ticks - right_start)
            # For in-place rotation, sum of deltas should reach 2 * target_ticks
            if (left_delta + right_delta) >= 2 * target_ticks:
                break
            rospy.sleep(0.01)
        self.stop_robot()

    def draw_square(self, side_length, speed, angular_speed):
        for _ in range(4):
            self.move_straight(side_length, speed)
            rospy.sleep(0.5)  # brief pause between actions
            self.rotate_in_place(math.pi / 2, angular_speed)
            rospy.sleep(0.5)


if __name__ == "__main__":
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
