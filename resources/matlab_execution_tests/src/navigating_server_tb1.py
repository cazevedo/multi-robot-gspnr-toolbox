#! /usr/bin/env python
from random import randint

import rospy

import actionlib

from matlab_execution_tests.msg import NavigationAction
from matlab_execution_tests.msg import NavigationGoal
from matlab_execution_tests.msg import NavigationFeedback
from matlab_execution_tests.msg import NavigationResult

class NavigatingAction(object):
    # create messages that are used to publish feedback/result
    _feedback = NavigationFeedback()
    _result = NavigationResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, NavigationAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        rospy.loginfo("Reaching here")
        r = rospy.Rate(1)
        success = True

        if goal.destination == 1:
            max_time = 5
        elif goal.destination == 2:
            max_time = 5
        else:
            max_time = 10

        travel_time = max_time
        current_time = 0

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, travelling to destination %i' % (self._action_name, goal.destination) )

        # start executing the action
        for i in range(1, travel_time):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            current_time = current_time + 1
            self._feedback.seconds_left = travel_time - current_time
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()

        if success:
            self._result.success = 1
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('tb1_Navigate')
    server = NavigatingAction(rospy.get_name())
    rospy.spin()
