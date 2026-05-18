#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped


class ClosedLoopSquare:
    def __init__(self, robot_name="mybota002409"):
        self.robot_name = robot_name
        self.ticks_per_meter = 660.7  # <-- Calibrate for your robot!
        self.ticks_per_degree = 2.0  # <-- Calibrate for your robot!
        self.left_tick_start = None
        self.right_tick_start = None
        self.target_ticks = None

        self.action = None  # 'straight' or 'rotate'
        self.direction = 1  # +1 or -1 (forward/backward or CCW/CW)
        self.sides_completed = 0
        self.state = "IDLE"

        self.pub_cmd = rospy.Publisher(
            f"/{robot_name}/wheels_driver_node/car_cmd", Twist2DStamped, queue_size=1
        )
        rospy.Subscriber(
            f"/{robot_name}/left_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self.left_encoder_callback,
        )
        rospy.Subscriber(
            f"/{robot_name}/right_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self.right_encoder_callback,
        )

    def start_square(self, side_length=1.0, speed=0.2, turn_speed=2):
        self.sides_completed = 0
        self.state = "MOVING"
        self.move_straight(side_length, speed)

    def move_straight(self, distance, speed):
        self.action = "straight"
        self.direction = 1 if distance >= 0 else -1
        self.target_ticks = abs(distance) * self.ticks_per_meter
        self.left_tick_start = None
        self.right_tick_start = None
        self.send_cmd(speed * self.direction, 0)
        rospy.loginfo(f"Moving straight - Target: {distance}m @ speed {speed}")

    def rotate_in_place(self, angle_deg, angular_speed):
        if self.ticks_per_degree is None:
            rospy.logwarn("ticks_per_degree not set! Calibrate rotation first!")
            return
        self.action = "rotate"
        self.direction = 1 if angle_deg >= 0 else -1
        self.target_ticks = abs(angle_deg) * self.ticks_per_degree
        self.left_tick_start = None
        self.right_tick_start = None
        self.send_cmd(0, angular_speed * self.direction)
        rospy.loginfo(
            f"Rotating in place - Target: {angle_deg}deg @ angular speed {angular_speed}"
        )

    def left_encoder_callback(self, msg):
        if self.action is None:
            return
        if self.left_tick_start is None:
            self.left_tick_start = msg.data
        self.check_stop_condition(msg.data, wheel="left")

    def right_encoder_callback(self, msg):
        if self.action is None:
            return
        if self.right_tick_start is None:
            self.right_tick_start = msg.data
        self.check_stop_condition(msg.data, wheel="right")

    def check_stop_condition(self, current_tick, wheel):
        start_tick = self.left_tick_start if wheel == "left" else self.right_tick_start
        if start_tick is None:
            return

        ticks_moved = abs(current_tick - start_tick)
        if ticks_moved >= self.target_ticks:
            self.send_cmd(0, 0)
            rospy.loginfo(
                f"{self.action.capitalize()} done using {wheel} encoder: {ticks_moved} ticks."
            )
            if self.action == "straight":
                self.sides_completed += 1
                if self.sides_completed < 4:
                    self.state = "TURNING"
                    rospy.sleep(1)
                    self.rotate_in_place(90, 2)  # 90 deg turn
                else:
                    rospy.loginfo("Square complete!")
                    self.state = "IDLE"
            elif self.action == "rotate":
                self.state = "MOVING"
                rospy.sleep(1)
                self.move_straight(1.0, 0.2)  # Next side
            self.action = None

    def send_cmd(self, v, omega):
        cmd = Twist2DStamped()
        cmd.v = v
        cmd.omega = omega
        self.pub_cmd.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("closed_loop_square")
    robot_name = rospy.get_param("~robot_name", "duckiebot1")
    ctl = ClosedLoopSquare(robot_name)
    rospy.sleep(1.0)
    ctl.start_square(side_length=1.0, speed=0.2, turn_speed=2)
    rospy.spin()
