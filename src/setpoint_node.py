#!/usr/bin/python3
# Copyright 2023 David Doerner (ddorner@kth.se)
"""
Node to publish the setpoint for the depth controller for a certain duration.
"""

import rospy
from std_msgs.msg import Float64
from depth_keeping.msg import Setpoint, ControlReference


class SetpointPublisher:
    """
    This node publishes the setpoint for the depth controller
    """
    def __init__(self):

        # Read parameters
        self.loop_frequency = rospy.get_param('~loop_frequency', 10.0)

        # Init
        self.depth_setpoint = 0.0
        self.duration = 0.0

        # Topics
        setpoint_topic = rospy.get_param('~setpoint_topic', 'setpoint')
        ref_topic = rospy.get_param('~ref_topic', 'trigger')

        # Subscribers
        rospy.Subscriber(setpoint_topic, Setpoint, self.setpoint_callback)

        # Publishers
        self.ref_pub = rospy.Publisher(ref_topic, ControlReference, queue_size=10)

        # Init
        rate = rospy.Rate(self.loop_frequency)  # 10 Hz
        self.triggered = False
        self.trigger_time = rospy.Time.now()
        self.ref = ControlReference()
        self.ref.depth = 0.0
        self.ref.pitch = 0.0

        while not rospy.is_shutdown():

            if self.triggered:
                if self.depth_setpoint > 0.0:
                    rospy.loginfo("Setpoint is %f. SAM can't fly, change sign.", self.depth_setpoint)
                    self.triggered = False
                else:
                    if rospy.Time.now() - self.trigger_time < rospy.Duration(self.duration):
                        rospy.loginfo("Triggered. Triggering setpoint %f", self.depth_setpoint)

                        # Publish setpoint
                        self.ref_publisher(self.ref)
                    else:
                        rospy.loginfo("Triggered for more than %f seconds. Triggering setpoint 0", self.duration)
                        # Publish setpoint 0
                        self.set_ref(0.0, 0.0)
                        self.ref_publisher(self.ref)  # Publish a setpoint of 0.0

                        self.triggered = False
            else:
                self.set_ref(0.0, 0.0)
                self.ref_publisher(self.ref)  # Publish a setpoint of 0.0

            rate.sleep()


    def setpoint_callback(self, setpoint):
        """
        Callback for the setpoint subscriber
        """
        self.triggered = setpoint.trigger
        self.trigger_time = rospy.Time.now()
        self.set_ref(setpoint.depth, setpoint.pitch)
        self.duration = setpoint.duration

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
