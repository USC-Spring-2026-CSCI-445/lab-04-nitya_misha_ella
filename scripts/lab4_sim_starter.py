#!/usr/bin/env python3
from math import inf
import queue
from time import sleep, time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# P controller class
class PController:
    """
    Generates control action taking into account instantaneous error (proportional action).
    """

    def __init__(self, kP, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initialize variables here
        ######### Your code starts here #########
        self.t_prev = rospy.get_time()
        self.kP = kP
        self.u_min = u_min
        self.u_max = u_max

        ######### Your code ends here #########

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        # Compute control action here
        ######### Your code starts here #########
        #err is current error (desired distance minus actual distance)
        u = self.kP*err
        u = max(self.u_min, min(u, self.u_max))

        self.t_prev = t

        return u
        ######### Your code ends here #########


# PD controller class
class PDController:
    """
    Generates control action taking into account instantaneous error (proportional action)
    and rate of change of error (derivative action).
    """

    def __init__(self, kP, kD, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initialize variables here
        ######### Your code starts here #########
        self.t_prev = rospy.get_time()
        self.err_prev = 0
        self.kP = kP
        self.kD = kD
        self.u_min = u_min
        self.u_max = u_max

        ######### Your code ends here #########

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        # Compute control action here
        ######### Your code starts here #########
        d_err = err - self.err_prev

        u = self.kP*err + self.kD*(d_err/dt)
        u = max(self.u_min, min(u, self.u_max))

        self.t_prev = t
        self.err_prev = err

        return u
        ######### Your code ends here #########


class RobotController:
    def __init__(self, desired_distance: float):
        print("\nMake the robot follow the wall on its left by maintaining the distance from it using LIDAR.\n")

        # ROS1 infrastructure
        rospy.init_node("robot_controller", anonymous=True)
        self.laserscan_sub = rospy.Subscriber("/scan", LaserScan, self.robot_laserscan_callback)
        self.robot_ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Define PD controller for wall-following here
        ######### Your code starts here #########
        
        # SET THESE VALUES IN LAB !!!!
        kP = 1.0
        u_min = -1.0
        u_max = 1.0

        #kD = 0.5 # -- for when we make pDcontroller

        self.p_controller = PController(kP, u_min, u_max)

        ######### Your code ends here #########

        self.desired_distance = desired_distance  # Desired distance from the wall
        self.ir_distance = None

    def robot_laserscan_callback(self, lscan: LaserScan):
        left = lscan.ranges[80:100]
        left = [x for x in left if x != inf]
        if len(left) > 0:
            self.ir_distance = sum(left) / len(left)


    def control_loop(self):

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.ir_distance is None:
                print("Waiting for IR sensor readings")
                sleep(0.1)
                continue

            ctrl_msg = Twist()

            # using PD controller, compute and send motor commands
            ######### Your code starts here #########

            ctrl_msg.linear.x = 0.1 ## random initial speed set  CHANGE IN LAB IF NEEDED

            err = self.desired_distance - self.ir_distance
            t = rospy.get_time()
            u = self.p_controller.control(err, t)

            ctrl_msg.angular.z = -u ## is this right? 

            ######### Your code ends here #########

            self.robot_ctrl_pub.publish(ctrl_msg)
            print(f"dist: {round(self.ir_distance, 4)}\ttgt: {round(self.desired_distance, 4)}\tu: {round(u, 4)}")
            rate.sleep()


if __name__ == "__main__":
    desired_distance = 0.5
    controller = RobotController(desired_distance)
    try:
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
