#!/usr/bin/env python3
from math import inf
from time import sleep

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# =========================================================
# P controller class
# =========================================================
class PController:
    """
    Generates control action taking into account instantaneous error (proportional action).
    """

    def __init__(self, kP, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        ######### Your code starts here #########
        self.kP = kP
        self.u_min = u_min
        self.u_max = u_max
        self.t_prev = 0.0
        ######### Your code ends here #########

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0.0

        ######### Your code starts here #########
        u = self.kP * err

        # clamp output
        if u > self.u_max:
            u = self.u_max
        elif u < self.u_min:
            u = self.u_min

        self.t_prev = t
        return u
        ######### Your code ends here #########


# =========================================================
# Robot Controller
# =========================================================
class RobotController:
    def __init__(self, desired_distance: float):
        print("\nMake the robot follow the wall on its LEFT using LIDAR.\n")

        # ROS1 infrastructure
        rospy.init_node("robot_controller", anonymous=True)
        self.laserscan_sub = rospy.Subscriber(
            "/scan", LaserScan, self.robot_laserscan_callback
        )
        self.robot_ctrl_pub = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=10
        )

        ######### Your code starts here #########
        # Tuned, STABLE gains for sim
        kP = 0.10
        u_min = -0.30
        u_max = 0.30

        self.p_controller = PController(kP, u_min, u_max)
        ######### Your code ends here #########

        self.desired_distance = desired_distance
        self.ir_distance = None

    def robot_laserscan_callback(self, lscan: LaserScan):
        # Front-left LIDAR slice (given by starter code — DO NOT CHANGE)
        left = lscan.ranges[80:100]
        left = [x for x in left if x != inf]

        if len(left) > 0:
            self.ir_distance = sum(left) / len(left)

    def control_loop(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.ir_distance is None:
                print("Waiting for LIDAR readings...")
                sleep(0.1)
                continue

            ctrl_msg = Twist()

            ######### Your code starts here #########
            # Slow, stable forward velocity
            ctrl_msg.linear.x = 0.06

            # ERROR DEFINITION (front-left geometry!)
            err = self.desired_distance - self.ir_distance

            t = rospy.get_time()
            u = self.p_controller.control(err, t)

            # IMPORTANT: sign flip REQUIRED because sensor is front-left
            ctrl_msg.angular.z = -u
            ######### Your code ends here #########

            self.robot_ctrl_pub.publish(ctrl_msg)

            print(
                f"dist: {self.ir_distance:.3f}  "
                f"target: {self.desired_distance:.3f}  "
                f"u: {u:.3f}"
            )

            rate.sleep()


# =========================================================
# Main
# =========================================================
if __name__ == "__main__":
    desired_distance = 0.5
    controller = RobotController(desired_distance)

    try:
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
