#!/usr/bin/env python3

import rospy
import math
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState


class ClosedLoopSquareNode:

    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        self.left_encoder_data = 0
        self.right_encoder_data = 0

        rospy.init_node("drive_square_node", anonymous=True)

        self.pub = rospy.Publisher(
            "/mybota002409/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1
        )
        rospy.Subscriber(
            "/mybota002409/fsm_node/mode", FSMState, self.fsm_callback, queue_size=1
        )

        rospy.Subscriber(
            "/mybota002409/left_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self.left_encoder_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/mybota002409/right_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self.right_encoder_callback,
            queue_size=1,
        )

    def left_encoder_callback(self, msg):
        # Process left encoder data if needed
        rospy.loginfo(f"Left encoder callback: {msg.data}")
        self.left_encoder_data = msg.data

    def right_encoder_callback(self, msg):
        # Process right encoder data if needed
        rospy.loginfo(f"Right encoder callback: {msg.data}")
        self.right_encoder_data = msg.data

    def fsm_callback(self, msg):
        rospy.loginfo(f"Received FSM state: {msg.state}")
        if msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # Sleep for a moment to ensure the system is ready
            # Move func
            self.move_robot()  # Move forward 1 meter at 0.5 m/s
            rospy.sleep(1)  # Sleep for a moment before the next command

    def distance_to_ticks(self, distance):
        tickes_per_meter = 660  # Example conversion factor, adjust as needed
        return int(distance * tickes_per_meter)

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def run(self):
        rospy.spin()

    def move_straight(self, distance, speed):
        ticks_needed = self.distance_to_ticks(distance)
        start_left = self.left_encoder_data
        start_right = self.right_encoder_data

        slow_down = 0.9
        min_speed = 0.2

        
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = speed
        self.cmd_msg.omega = 0.0
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            self.pub.publish(self.cmd_msg)
            left_travel = abs(self.left_encoder_data - start_left)
            right_travel = abs(self.right_encoder_data - start_right)
            average_travel = (left_travel + right_travel) / 2

            if average_travel >= slow_down * ticks_needed:
                current_speed = min_speed
            else:
                current_speed = speed

            if average_travel >= ticks_needed:
                self.stop_robot()
                break
            rate.sleep()
        self.stop_robot()


    def rotate_in_place(self, angle_deg, omega):

        TICKS_PER_DEG = 0.47

        start_left = self.left_encoder_data
        start_right = self.right_encoder_data

        target = abs(angle_deg) * TICKS_PER_DEG

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            delta_left = abs(self.left_encoder_data - start_left)
            delta_right = abs(self.right_encoder_data - start_right)

            delta = (delta_left + delta_right) / 2

            if delta >= target:
                break

            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = omega
            self.pub.publish(self.cmd_msg)

            rate.sleep()

    def move_robot(self):

        rospy.loginfo("Starting CLOSED LOOP square")

        for i in range(4):

            rospy.loginfo("Side %d", i + 1)

            # Forward
            self.move_straight(1.0, 0.3)
            #test self.move_straight(1.0, 0.6)

            # Rotate
            self.rotate_in_place(90, 1.5)
            #test self.rotate_in_place(90, 2.0)

        rospy.loginfo("DONE")
        self.stop_robot()


if __name__ == "__main__":
    try:
        duckiebot_movement = ClosedLoopSquareNode()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass

