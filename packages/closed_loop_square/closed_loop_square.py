#!/usr/bin/env python3

import rospy
import time
import math
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from nav_msgs.msg import Odometry


class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        # Initialize ROS node
        rospy.init_node("drive_square_node", anonymous=True)

        # Initialize Pub/Subs
        self.pub = rospy.Publisher(
            "/mybota002409/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1
        )
        rospy.Subscriber(
            "/mybota002409/fsm_node/mode", FSMState, self.fsm_callback, queue_size=1
        )
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (
            orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y
        )
        cosy_cosp = 1 - 2 * (
            orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z
        )
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    # robot only moves when lane following is selected on the duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # Wait for a sec for the node to be ready
            self.draw_square(side_length=1.0, speed=0.2, angular_speed=0.5)

    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    # Spin forever but listen to message callbacks
    def run(self):
        rospy.spin()  # keeps node from exiting until node has shutdown

    def wait_for_odom(self):
        rospy.loginfo("Waiting for odometry data...")
        while not rospy.is_shutdown() and not self.odom_received:
            rospy.sleep(0.1)

    def move_straight(self, distance, speed):
        """Moves the robot straight for a given distance at a given speed using odometry."""
        self.wait_for_odom()
        start_x, start_y = self.x, self.y
        direction = 1 if distance >= 0 else -1
        speed = abs(speed) * direction

        rospy.loginfo(
            f"[move_straight] Requested distance: {distance} m, speed: {speed} m/s"
        )

        self.cmd_msg.v = speed
        self.cmd_msg.omega = 0.0

        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_msg)
            dx = self.x - start_x
            dy = self.y - start_y
            traveled = math.sqrt(dx * dx + dy * dy)
            rospy.loginfo_throttle(
                1, f"[move_straight] Traveled: {traveled:.3f} m / {abs(distance):.3f} m"
            )
            if traveled >= abs(distance):
                break
            rate.sleep()

        rospy.loginfo("[move_straight] Target distance reached. Stopping robot.")
        self.stop_robot()

    def rotate_in_place(self, angle, angular_speed):
        """
        Rotates the robot in place for a given angle at a given angular speed using odometry.
        Handles both clockwise and counterclockwise motion.
        """
        self.wait_for_odom()
        start_yaw = self.yaw
        direction = 1 if angle >= 0 else -1
        angular_speed = abs(angular_speed) * direction
        target_angle = abs(angle)

        rospy.loginfo(
            f"[rotate_in_place] Requested angle: {angle} rad, angular_speed: {angular_speed} rad/s"
        )

        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = angular_speed

        rate = rospy.Rate(10)  # 10 Hz

        def angle_diff(a, b):
            d = a - b
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            return d

        while not rospy.is_shutdown():
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_msg)
            delta_yaw = abs(angle_diff(self.yaw, start_yaw))
            rospy.loginfo_throttle(
                1,
                f"[rotate_in_place] Rotated: {delta_yaw:.3f} rad / {target_angle:.3f} rad",
            )
            if delta_yaw >= target_angle:
                break
            rate.sleep()

        rospy.loginfo("[rotate_in_place] Target angle reached. Stopping robot.")
        self.stop_robot()

    def draw_square(self, side_length=1.0, speed=0.2, angular_speed=0.5):
        """
        Draws a square with the given side length, speed, and angular speed.
        """
        for i in range(4):
            rospy.loginfo(f"[draw_square] Side {i+1}/4: Moving straight.")
            self.move_straight(side_length, speed)
            rospy.sleep(1)  # Small pause
            rospy.loginfo(f"[draw_square] Side {i+1}/4: Rotating 90 degrees.")
            self.rotate_in_place(math.pi / 2, angular_speed)
            rospy.sleep(1)


if __name__ == "__main__":
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        passass
