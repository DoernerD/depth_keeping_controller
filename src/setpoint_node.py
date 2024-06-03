#!/usr/bin/python3
# Copyright 2023 David Doerner (ddorner@kth.se)
"""
Node to publish the setpoint for the depth controller for a certain duration.
"""

import rospy
from std_msgs.msg import Float64, Bool, Empty
from smarc_bt.msg import MissionControl
from depth_keeping.msg import Setpoint, ControlReference


class SetpointPublisher:
    """
    This node publishes the setpoint for the depth controller
    """
    def __init__(self):

        # Read parameters
        self.loop_frequency = rospy.get_param('~loop_frequency', 10.0)

        # Init
        self.duration = 0.0
        self.abort = False
        self.mission_active = False
        self.triggered = False
        self.trigger_time = rospy.Time.now()
        self.ref = ControlReference()
        self.ref.depth = 0.0
        self.ref.pitch = 0.0

        # Topics
        abort_topic = rospy.get_param("~abort_topic")
        mission_topic = rospy.get_param("~mission_topic")
        setpoint_topic = rospy.get_param('~setpoint_topic', 'setpoint')
        ref_topic = rospy.get_param('~ref_topic', 'ref')

        # Subscribers
        rospy.Subscriber(abort_topic, Empty, self.abort_callback, queue_size=1)
        rospy.Subscriber(mission_topic, MissionControl, self.mission_callback, queue_size=1)
        rospy.Subscriber(setpoint_topic, Setpoint, self.setpoint_callback, queue_size=1)

        # Publishers
        self.ref_pub = rospy.Publisher(ref_topic, ControlReference, queue_size=10)

        # Init
        

        rate = rospy.Rate(self.loop_frequency)  # 10 Hz


        while not rospy.is_shutdown():

            if self.triggered and not self.abort:
                if self.ref.depth > 0.5:
                    rospy.loginfo("Setpoint is %f. SAM can't fly, change sign.", self.ref.depth)
                    self.triggered = False
                else:
                    if (rospy.Time.now() - self.trigger_time < rospy.Duration(self.duration)) or self.mission_active:
                        rospy.loginfo("Triggered. Triggering setpoint %.2f, %.2f", self.ref.depth, self.ref.pitch)

                        # Publish setpoint
                        self.ref_publisher(self.ref)
                    else: 
                        rospy.loginfo("Triggered for more than %f seconds. Triggering setpoint 0", self.duration)
                        # Publish setpoint 0
                        self.set_ref(0.0, 0.0)
                        self.ref_publisher(self.ref)

                        self.triggered = False
            else:
                self.set_ref(0.0, 0.0)
                self.ref_publisher(self.ref)

            rate.sleep()


    def abort_callback(self, abort):
        """
        Callback for abort message
        """
        if abort:
            rospy.logwarn("Received Abort. Setting setpoint to 0.")
            self.abort = True
            self.triggered = False
            self.set_ref(0.0, 0.0)
            self.ref_publisher(self.ref)  # Publish a setpoint of 0.0


    def mission_callback(self, mission_status):
        """
        Callback for mission status
        """

        if mission_status.plan_state == 0:
            rospy.logwarn("Received Mission Start. Timer disabled")
            self.mission_active = True

        elif mission_status.plan_state == 3:
            rospy.logwarn("Received Abort. Hands Off")

        elif mission_status.plan_state == 5:
            rospy.logwarn("Received Mission Complete.")
            self.set_ref(0.0, 0.0)

        else:
            rospy.logwarn("Received Mission not Start. Timer enabled")
            self.mission_active = False



    def setpoint_callback(self, setpoint):
        """
        Callback for the setpoint subscriber
        """
        self.triggered = setpoint.trigger
        self.trigger_time = rospy.Time.now()
        self.set_ref(setpoint.depth, setpoint.pitch)
        self.duration = setpoint.duration

        rospy.loginfo("setpoint cb")

        if not setpoint.trigger:
            rospy.loginfo("Received trigger false, setting setpoint to 0")
            self.set_ref(0.0, 0.0)
            self.ref_publisher(self.ref)


    def ref_publisher(self, ref):
        """
        Publishes the reference point
        """
        self.ref_pub.publish(ref)


    def set_ref(self, depth, pitch):
        """
        Set the reference for publishing
        """
        self.ref.depth = depth
        self.ref.pitch = pitch


if __name__ == '__main__':
    try:
        rospy.init_node('SetpointPublisher')
        node = SetpointPublisher()
    except rospy.ROSInterruptException:
        pass
