#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState


class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node("drive_square_node", anonymous=True)

        # Initialize Pub/Subs
        self.pub = rospy.Publisher(
            "/mybota002409/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1
        )
        rospy.Subscriber(
            "/mybota002409/fsm_node/mode", FSMState, self.fsm_callback, queue_size=1
        )

    # robot only moves when lane following is selected on the duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # Wait for a sec for the node to be ready
            self.move_straight(1.0, 0.2)
            rospy.sleep(1)  # Short pause before next command
            self.move_straight(-1.0, 0.1)

    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    # Spin forever but listen to message callbacks
    def run(self):
        rospy.spin()  # keeps node from exiting until node has shutdown

    # Move robot in a straight line.

    def move_straight(self, distance, speed):
        """Moves the robot straight (given a distance and speed)"""

        direction = 1 if distance >= 0 else -1
        speed = abs(speed) * direction
        duration = abs(distance / speed)

        rospy.loginfo(
            f"[move_straight] Requested distance: {distance} m, speed: {speed} m/s"
        )
        rospy.loginfo(
            f"[move_straight] Calculated duration: {duration:.2f} s, direction: {'forward' if direction == 1 else 'backward'}"
        )

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = speed
        self.cmd_msg.omega = 0.0

        start_time = time.time()
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown() and (time.time() - start_time) < duration:
            elapsed = time.time() - start_time
            rospy.loginfo_throttle(
                1,
                f"[move_straight] Moving... elapsed: {elapsed:.2f} s / {duration:.2f} s",
            )
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_msg)
            rate.sleep()

        rospy.loginfo("[move_straight] Target distance reached. Stopping robot.")
        self.stop_robot()


if __name__ == "__main__":
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
