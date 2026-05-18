#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped, FSMState


class ClosedLoopSquare:
    def __init__(self, robot_name="mybota002409"):
        self.robot_name = robot_name

        # Calibration (tune these on your bot)
        self.ticks_per_meter = 660.7
        self.ticks_per_degree = 2.0

        # Runtime state
        self.left_tick_start = None
        self.right_tick_start = None
        self.target_ticks = 0.0
        self.action = None  # "straight" or "rotate"
        self.sides_completed = 0
        self.square_active = False

        # Store current command settings for consistent segments
        self.side_length = 1.0
        self.linear_speed = 0.2
        self.turn_angle_deg = 90.0
        self.turn_speed = 2.0

        # Publish to the same command path as your working open-loop node
        self.pub_cmd = rospy.Publisher(
            f"/{robot_name}/car_cmd_switch_node/cmd",
            Twist2DStamped,
            queue_size=1,
        )

        # Encoder feedback
        rospy.Subscriber(
            f"/{robot_name}/left_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self.left_encoder_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            f"/{robot_name}/right_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self.right_encoder_callback,
            queue_size=1,
        )

        # Trigger like open-loop: only run in lane following mode
        rospy.Subscriber(
            f"/{robot_name}/fsm_node/mode",
            FSMState,
            self.fsm_callback,
            queue_size=1,
        )

    def fsm_callback(self, msg):
        rospy.loginfo("FSM State: %s", msg.state)

        if msg.state == "LANE_FOLLOWING" and not self.square_active:
            rospy.sleep(1.0)
            self.start_square(side_length=1.0, speed=0.2, turn_speed=2.0)

        if msg.state != "LANE_FOLLOWING" and self.square_active:
            rospy.loginfo("Leaving LANE_FOLLOWING, stopping.")
            self.square_active = False
            self.action = None
            self.stop_robot()

    def start_square(self, side_length=1.0, speed=0.2, turn_speed=2.0):
        self.side_length = side_length
        self.linear_speed = speed
        self.turn_speed = turn_speed

        self.sides_completed = 0
        self.square_active = True

        rospy.loginfo("Starting closed-loop square.")
        self.move_straight(self.side_length, self.linear_speed)

    def move_straight(self, distance_m, speed):
        self.action = "straight"
        self.target_ticks = abs(distance_m) * self.ticks_per_meter
        self.left_tick_start = None
        self.right_tick_start = None

        v = speed if distance_m >= 0 else -speed
        self.send_cmd(v, 0.0)
        rospy.loginfo(
            "Moving straight: target=%.3f m (%.1f ticks)", distance_m, self.target_ticks
        )

    def rotate_in_place(self, angle_deg, angular_speed):
        if self.ticks_per_degree is None:
            rospy.logwarn("ticks_per_degree is not set.")
            self.square_active = False
            self.stop_robot()
            return

        self.action = "rotate"
        self.target_ticks = abs(angle_deg) * self.ticks_per_degree
        self.left_tick_start = None
        self.right_tick_start = None

        omega = angular_speed if angle_deg >= 0 else -angular_speed
        self.send_cmd(0.0, omega)
        rospy.loginfo(
            "Rotating: target=%.1f deg (%.1f ticks)", angle_deg, self.target_ticks
        )

    def left_encoder_callback(self, msg):
        if not self.square_active or self.action is None:
            return
        if self.left_tick_start is None:
            self.left_tick_start = msg.data
        self.check_stop_condition(msg.data, wheel="left")

    def right_encoder_callback(self, msg):
        if not self.square_active or self.action is None:
            return
        if self.right_tick_start is None:
            self.right_tick_start = msg.data
        self.check_stop_condition(msg.data, wheel="right")

    def check_stop_condition(self, current_tick, wheel):
        if self.action is None:
            return

        start_tick = self.left_tick_start if wheel == "left" else self.right_tick_start
        if start_tick is None:
            return

        ticks_moved = abs(current_tick - start_tick)
        if ticks_moved < self.target_ticks:
            return

        completed_action = self.action
        self.stop_robot()
        rospy.loginfo(
            "%s complete (%s wheel): %.1f ticks", completed_action, wheel, ticks_moved
        )

        if completed_action == "straight":
            self.action = "rotate"
            rospy.sleep(0.5)
            self.rotate_in_place(self.turn_angle_deg, self.turn_speed)
            return

        if completed_action == "rotate":
            self.sides_completed += 1
            if self.sides_completed >= 4:
                rospy.loginfo("Square complete.")
                self.square_active = False
                self.action = None
                self.stop_robot()
                return

            self.action = "straight"
            rospy.sleep(0.5)
            self.move_straight(self.side_length, self.linear_speed)

    def send_cmd(self, v, omega):
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = v
        cmd.omega = omega
        self.pub_cmd.publish(cmd)

    def stop_robot(self):
        self.send_cmd(0.0, 0.0)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("closed_loop_square")
    robot_name = rospy.get_param("~robot_name", "mybota002409")
    controller = ClosedLoopSquare(robot_name)
    controller.run()
