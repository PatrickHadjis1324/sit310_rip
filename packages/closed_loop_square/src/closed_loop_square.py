#!/usr/bin/env python3

import rospy
import math
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
            EncoderMsgType,
            self.left_encoder_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/mybota002409/right_wheel_encoder_node/tick",
            EncoderMsgType,
            self.right_encoder_callback,
            queue_size=1,
        )

    def left_encoder_callback(self, msg):
        # Process left encoder data if needed
        self.left_encoder_data = msg.data

    def right_encoder_callback(self, msg):
        # Process right encoder data if needed
        self.right_encoder_data = msg.data

    def fsm_callback(self, msg):
        rospy.loginfo(f"Received FSM state: {msg.state}")
        if msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # Sleep for a moment to ensure the system is ready
            # Move func
            self.move_straight(1.0, 0.5)  # Move forward 1 meter at 0.5 m/s
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

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 1.0
        self.cmd_msg.omega = 0.0
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            self.pub.publish(self.cmd_msg)
            left_travel = abs(self.left_encoder_data - start_left)
            right_travel = abs(self.right_encoder_data - start_right)
            average_travel = (left_travel + right_travel) / 2
            if average_travel >= ticks_needed:
                self.stop_robot()
                break
            rate.sleep()
        self.stop_robot()


if __name__ == "__main__":
    try:
        duckiebot_movement = ClosedLoopSquareNode()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
