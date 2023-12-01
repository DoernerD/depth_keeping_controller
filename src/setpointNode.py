#!/usr/bin/python3
# Copyright 2023 David Doerner (ddorner@kth.se)

import rospy
from std_msgs.msg import Float64, Bool



class SetpointPublisher:
    def __init__(self):

        # Read parameters
        self.loop_frequency = rospy.get_param('~loop_frequency', 10.0)
        self.depth_setpoint = rospy.get_param('~depth_setpoint', 1.0)
        self.duration = rospy.get_param('~duration', 5.0)

        # Topics
        trigger_topic = rospy.get_param('~trigger_topic', 'trigger')
        setpoint_topic = rospy.get_param('~setpoint_topic', 'setpoint')

        # Subscribers
        rospy.Subscriber(trigger_topic, Bool, self.trigger_callback)

        # Publishers
        self.setpoint_pub = rospy.Publisher(setpoint_topic, Float64, queue_size=10)

        # Init
        rate = rospy.Rate(self.loop_frequency)  # 10 Hz
        self.triggered = False
        self.trigger_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.triggered:
                if rospy.Time.now() - self.trigger_time < rospy.Duration(self.duration):
                    rospy.loginfo("Triggered. Triggering setpoint %f", self.depth_setpoint)

                    # Publish setpoint
                    self.setpoint_publisher(self.depth_setpoint)
                else:
                    rospy.loginfo("Triggered for more than %f seconds. Triggering setpoint 0", self.duration)
                    # Publish setpoint 0
                    self.setpoint_publisher(0.0)  # Publish a setpoint of 0.0

                    self.triggered = False
            else:
                self.setpoint_publisher(0.0)  # Publish a setpoint of 0.0

            rate.sleep()


    def trigger_callback(self, trigger):
        self.triggered = trigger.data
        self.trigger_time = rospy.Time.now()

        if not trigger.data:
            rospy.loginfo("Received trigger false, setting setpoint to 0")
            self.setpoint_publisher(0.0)


    def setpoint_publisher(self, setpoint):
        self.setpoint_pub.publish(setpoint)


if __name__ == '__main__':
    try:
        rospy.init_node('SetpointPublisher')
        node = SetpointPublisher()
    except rospy.ROSInterruptException:
        pass
