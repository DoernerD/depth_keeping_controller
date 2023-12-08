#!/usr/bin/python3
# Copyright 2023 David Doerner (ddorner@kth.se)

from __future__ import division, print_function

import numpy as np

import rospy

from tf.transformations import euler_from_quaternion

from std_msgs.msg import Float64, Bool, Empty
from sam_msgs.msg import PercentStamped
from nav_msgs.msg import Odometry
from depth_keeping.msg import ControlState, ControlInput, ControlError, ControlReference


class DepthKeepingController(object):
    """
    Depth keeping controller for the SMARC AUV.
    """

    # Main loop
    def __init__(self, name):

        # Launch File Parameters
        self.loop_freq = rospy.get_param("~loop_freq", 20)
        verbose = rospy.get_param("~verbose", True)

        # Init
        self.current_state = np.array([0., 0., 0., 0., 0., 0.])
        self.z_ref = 0.
        self.pitch_ref = 0.
        self.ref = np.array([self.z_ref, self.pitch_ref])

        # Control Gains
        self.vbs_Kp = 40
        self.vbs_Ki = 1
        self.vbs_Kd = 0.7
        self.vbs_Kaw = 1

        self.lcg_Kp = 100
        self.lcg_Ki = 10
        self.lcg_Kd = 1
        self.lcg_Kaw = 1

        self.eps_depth = 0.6 # offset for depth control
        self.eps_pitch = 0.3 # offset for pitch control

        self.error = np.array([0., 0.])
        self.error_prev = np.array([0., 0.])
        self.integral = np.array([0., 0.])

        # Neutral actuator inputs
        self.vbs_neutral = 50.
        self.lcg_neutral = 50.

        self.u_neutral = np.array([self.vbs_neutral,
                                  self.lcg_neutral])

        self.anti_windup_diff = np.array([0., 0.])
        self.anti_windup_diff_integral = np.array([0., 0.])

        self.limit_output_cnt = 0

        self.abort = False

        # Topics for feedback and actuators
        vbs_topic = rospy.get_param("~vbs_topic", "/sam/core/vbs_cmd")
        lcg_topic = rospy.get_param("~lcg_topic", "/sam/core/lcg_cmd")

        ref_pose_topic = rospy.get_param("~ref_pose_topic")
        state_estimate_topic = rospy.get_param("~state_estimation_topic")

        state_topic = rospy.get_param("~state_topic")
        ref_topic = rospy.get_param("~ref_topic")
        error_topic = rospy.get_param("~error_topic")
        control_input_topic = rospy.get_param("~control_input_topic")
        control_neutral_topic = rospy.get_param("~control_neutral_topic")

        abort_topic = rospy.get_param("~abort_topic")

        # Subscribers to state feedback, setpoints and enable flags
        rospy.Subscriber(abort_topic, Empty, self.abort_callback, queue_size=1)
        rospy.Subscriber(ref_pose_topic, ControlReference, self.ref_callback, queue_size=1)
        rospy.Subscriber(state_estimate_topic, Odometry, self.estimation_callback, queue_size=1)

        # Publisher to actuators
        self.vbs_pub = rospy.Publisher(vbs_topic, PercentStamped, queue_size=10)
        self.lcg_pub = rospy.Publisher(lcg_topic, PercentStamped, queue_size=10)

        # Convenience topics
        self.state_pub = rospy.Publisher(state_topic, ControlState, queue_size=1)
        self.ref_pub = rospy.Publisher(ref_topic, ControlState, queue_size=1)
        self.error_pub = rospy.Publisher(error_topic, ControlError, queue_size=1)
        self.control_input_pub = rospy.Publisher(control_input_topic, ControlInput, queue_size=1)
        self.control_neutral_pub = rospy.Publisher(control_neutral_topic, ControlInput, queue_size=1)

        rate = rospy.Rate(self.loop_freq)

        # if verbose:
        #     self.console = curses.initscr()  # initialize is our playground

        # Run
        while not rospy.is_shutdown():

            u = self.compute_control_action()

            u_limited = self.limit_control_action(u)

            self.compute_anti_windup(u, u_limited)

            if not self.abort:
                self.publish_control_action(u_limited)

                self.publish_convenience_topics(u_limited)

            if verbose:
                self.print_states(u, u_limited)

            rate.sleep()

        # if verbose:
        #     curses.endwin()  # return control back to the console


    #region Call-backs
    def abort_callback(self, abort):
        """
        Callback for abort message
        """
        if abort:
            self.abort = True
            rospy.logwarn("Received Abort. Stop publishing anything.")


    def estimation_callback(self, estim):
        """
        Get the current state of the vehicle from the state estimation node.
        """
        self.current_state = self.get_euler_from_quaternion(estim.pose)


    def ref_callback(self, ref):
        """
        Get desired reference depth.
        """
        self.ref[0] = ref.depth
        self.ref[1] = ref.pitch


    def get_euler_from_quaternion(self, pose):
        """
        Extracts the position and Euler angles.
        """
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z

        quat = [pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w]

        rpy = euler_from_quaternion(quat)

        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]

        states = np.array([x, y, z, roll, pitch, yaw])

        return states
    #endregion


    #region Publisher
    def publish_control_action(self, u):
        """
        Publish the control action to the actuators.
        """
        vbs = PercentStamped()
        lcg = PercentStamped()


        vbs.value = int(u[0])
        lcg.value = int(u[1])

        # Publish to actuators
        self.vbs_pub.publish(vbs)
        self.lcg_pub.publish(lcg)


    def publish_convenience_topics(self, u):
        """
        Publish convenience topics for plotting and debugging.
        """
        # Publish reference
        ref_msg = ControlState()
        ref_msg.pose.x = 0.0
        ref_msg.pose.y = 0.0
        ref_msg.pose.z = self.ref[0]
        ref_msg.pose.roll = 0.0
        ref_msg.pose.pitch = self.ref[1]
        ref_msg.pose.yaw = 0.0

        ref_msg.vel.x = 0.0
        ref_msg.vel.y = 0.0
        ref_msg.vel.z = 0.0
        ref_msg.vel.roll = 0.0
        ref_msg.vel.pitch = 0.0
        ref_msg.vel.yaw = 0.0

        self.ref_pub.publish(ref_msg)

        # Publish current state
        state_msg = ControlState()
        state_msg.pose.x = self.current_state[0]
        state_msg.pose.y = self.current_state[1]
        state_msg.pose.z = self.current_state[2]
        state_msg.pose.roll = self.current_state[3]
        state_msg.pose.pitch = self.current_state[4]
        state_msg.pose.yaw = self.current_state[5]

        state_msg.vel.x = 0.0
        state_msg.vel.y = 0.0
        state_msg.vel.z = 0.0
        state_msg.vel.roll = 0.0
        state_msg.vel.pitch = 0.0
        state_msg.vel.yaw = 0.0
        self.state_pub.publish(state_msg)

        # Publish error
        error_msg = ControlError()
        error_msg.x = 0.0
        error_msg.y = 0.0
        error_msg.z = self.error[0]
        error_msg.roll = 0.0
        error_msg.pitch = self.error[1]
        error_msg.yaw = 0.0
        error_msg.heading = 0.0
        error_msg.distance = 0.0

        self.error_pub.publish(error_msg)

        # Publish control input
        control_input_msg = ControlInput()
        control_input_msg.thrusterRPM = 0.0
        control_input_msg.thrusterHorizontal = 0.0
        control_input_msg.thusterVertical = 0.0
        control_input_msg.vbs = u[0]
        control_input_msg.lcg = u[1]

        self.control_input_pub.publish(control_input_msg)

        # Publish neutral control input
        control_neutral_msg = ControlInput()
        control_neutral_msg.thrusterRPM = 0.0
        control_neutral_msg.thrusterHorizontal = 0.0
        control_neutral_msg.thusterVertical = 0.0
        control_neutral_msg.vbs = self.u_neutral[0]
        control_neutral_msg.lcg = self.u_neutral[1]

        self.control_neutral_pub.publish(control_neutral_msg)
    #endregion


    # Controller
    def compute_control_action(self):
        """
        Depth and pitch control
        u = [vbs, lcg]
        x = [z, pitch]
        """
        u = np.array([self.vbs_neutral,
                        self.lcg_neutral])

        control_state = np.array([self.current_state[2], self.current_state[4]])

        self.error_prev = self.error
        self.error = self.ref - control_state

        # Depth is always negative, which is why we change the signs on the
        # depth error. Then we can keep the remainder of the control structure
        self.error[0] = -self.error[0]

        # Anti windup integral is calculated separately, because
        # dim(e) != dim(u).
        self.calculate_anti_windup_integral()

        self.integral += self.error * (1/self.loop_freq)
        self.deriv = (self.error - self.error_prev) * self.loop_freq

        u[0] = self.vbs_Kp*self.error[0] \
                + self.vbs_neutral \
                + self.vbs_Ki*(self.integral[0] + self.anti_windup_diff_integral[0]) \
                + self.vbs_Kd*self.deriv[0]   # PID control vbs

        u[1] = self.lcg_Kp*self.error[1] \
                + self.lcg_neutral \
                + self.lcg_Ki*(self.integral[1] + self.anti_windup_diff_integral[1]) \
                + self.lcg_Kd*self.deriv[1]   # PID control lcg

        return u


    def calculate_anti_windup_integral(self):
        """
        Calculate the anti windup integral
        """
        self.anti_windup_diff_integral += (self.anti_windup_diff) * (1/self.loop_freq)


    def compute_anti_windup(self, u, u_limited):
        """
        Compute anti wind up difference.
        """
        self.anti_windup_diff = np.array([self.vbs_Kaw, self.lcg_Kaw]) * (u_limited - u)


    def limit_control_action(self,u):
        """
        Take hardware limitations into account and limit actuator outputs.
        """
        # Enforce hardware limits on actuator control
        u_limited = u.copy()     # without .copy(), python only makes a shallow copy of the array.

        # vbs limit
        if u_limited[0] > 100:
            u_limited[0] = 100
        if u_limited[0] < 0:
            u_limited[0] = 0

        # lcg limit
        if u_limited[1] > 100:
            u_limited[1] = 100
        if u_limited[1] < 0:
            u_limited[1] = 0

        return u_limited




    def print_states(self, u, u_limited):
        """
        Print function for system states
        use print("string {}".format(data)) instead. Python will format data accordingly
        and print it in place of {}. To print multiple variables, use multiple {}.
        Using the curses package, we can overwrite the console output. Is a bit weird when we have
        other cli outputs, too, but makes looking at the control stuff a lot easier.
        """
        np.set_printoptions(suppress=True)
        self.limit_output_cnt += 1
        if self.limit_output_cnt % 20 == 1:
            # self.console.addstr(0,0, "All in ENU: [x, y, z, roll, pitch, yaw]")
            # self.console.addstr(1,0, (""))
            # self.console.addstr(2,0, "Current States: {}".format(np.array2string(self.state_estimated, precision = 2, suppress_small = True, floatmode = 'fixed')))
            # self.console.addstr(3,0, "Reference States: {}".format(np.array2string(self.ref_odom, precision = 2, suppress_small = True, floatmode = 'fixed')))
            # self.console.addstr(4,0, "Distance Error: {:.4f}, Pitch Angle: {:.2f}, Depth Error: {:.2f}, velocity error: {:.2f}".format(self.distance_error, self.error[4], self.error[2], self.error_velocity))
            # self.console.addstr(5,0, (""))
            # self.console.addstr(6,0, "                   [RPM,  hor,  ver,  vbs,  lcg]")
            # self.console.addstr(7,0, "Control Input raw: {}".format(np.array2string(u, precision = 2, floatmode = 'fixed')))
            # self.console.addstr(8,0, "Control Input lim: {}".format(np.array2string(u_limited, precision = 2, floatmode = 'fixed')))
            # self.console.addstr(9,0, "Anti Windup Int: {}".format(np.array2string(self.anti_windup_diff_integral, precision = 2, floatmode = 'fixed')))
            # self.console.addstr(10,0, "")
            # self.console.refresh()
            if not self.abort:
                print("All in ENU: [z, pitch]")
                print("")
                print("Current States: {}".format(np.array2string(self.current_state, precision = 2, suppress_small = True, floatmode = 'fixed')))
                print("Reference States: {}".format(np.array2string(self.ref, precision = 2, suppress_small = True, floatmode = 'fixed')))
                print("Pitch Angle: {:.2f}, Depth Error: {:.2f}".format(self.error[1], self.error[0]))
                print("")
                print("[vbs, lcg]")
                print("Control Input raw: {}".format(np.array2string(u, precision = 2, floatmode = 'fixed')))
                print("Control Input lim: {}".format(np.array2string(u_limited, precision = 2, floatmode = 'fixed')))
                print("Anti Windup Int: {}".format(np.array2string(self.anti_windup_diff_integral, precision = 2, floatmode = 'fixed')))
                print("-----")
            else:
                print("")
                print("EMERGECY PROTOCOL")
                print("BT controls SAM")
                print("-----")


if __name__ == "__main__":
    rospy.init_node("DepthKeepingController")
    controller = DepthKeepingController(rospy.get_name())
