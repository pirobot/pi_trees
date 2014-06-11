#!/usr/bin/env python

"""
    pi_trees_ros.py - Version 0.1 2013-08-28
    
    ROS wrappers for the pi_trees_lib.py library
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from pi_trees_lib.pi_trees_lib import *

class MonitorTask(Task):
    """
        Turn a ROS subscriber into a Task.
    """
    def __init__(self, name, topic, msg_type, msg_cb, wait_for_message=True, timeout=5):
        super(MonitorTask, self).__init__(name)
        
        self.topic = topic
        self.msg_type = msg_type
        self.timeout = timeout
        self.msg_cb = msg_cb
                
        rospy.loginfo("Subscribing to topic " + topic)
        
        if wait_for_message:
            try:
                rospy.wait_for_message(topic, msg_type, timeout=self.timeout)
                rospy.loginfo("Connected.")
            except:
                rospy.loginfo("Timed out waiting for " + topic)
                
        # Subscribe to the given topic with the given callback function executed via run() 
        rospy.Subscriber(self.topic, self.msg_type, self._msg_cb)
        
    def _msg_cb(self, msg):
        self.set_status(self.msg_cb(msg))
        
    def run(self):
        return self.status
    
    def reset(self):
        pass
    
class ServiceTask(Task):
    """
        Turn a ROS service into a Task.
    """
    def __init__(self, name, service, service_type, request, result_cb=None, wait_for_service=True, timeout=5):
        super(ServiceTask, self).__init__(name)
        
        self.result = None
        self.request = request
        self.timeout = timeout
        self.result_cb = result_cb
                
        rospy.loginfo("Connecting to service " + service)
        
        if wait_for_service:
            rospy.loginfo("Waiting for service")
            rospy.wait_for_service(service, timeout=self.timeout)
            rospy.loginfo("Connected.")
        
        # Create a service proxy
        self.service_proxy = rospy.ServiceProxy(service, service_type)
        
    def run(self):
        try:
            result = self.service_proxy(self.request)
            if self.result_cb is not None:
                self.result_cb(result)
            return TaskStatus.SUCCESS
        except:
            rospy.logerr(sys.exc_info())
            return TaskStatus.FAILURE
        
    def reset(self):
        pass
        
class SimpleActionTask(Task):
    """
        Turn a ROS action into a Task.
    """
    def __init__(self, name, action, action_type, goal, rate=5, connect_timeout=10, result_timeout=30, reset_after=False, active_cb=None, done_cb=None, feedback_cb=None):
        super(SimpleActionTask, self).__init__(name)
        
        self.action = action
        self.goal = goal
        self.tick = 1.0 / rate
        self.rate = rospy.Rate(rate)

        self.result = None
        self.connect_timeout = connect_timeout
        self.result_timeout = result_timeout
        self.reset_after = reset_after
        
        if done_cb == None:
            done_cb = self.default_done_cb
        self.done_cb = done_cb
        
        if active_cb == None:
            active_cb = self.default_active_cb
        self.active_cb = active_cb
        
        if feedback_cb == None:
            feedback_cb = self.default_feedback_cb
        self.feedback_cb = feedback_cb
                
        self.action_started = False
        self.action_finished = False
        self.goal_status_reported = False
        self.time_so_far = 0.0
        
        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                            'SUCCEEDED', 'ABORTED', 'REJECTED',
                            'PREEMPTING', 'RECALLING', 'RECALLED',
                            'LOST']
    
        rospy.loginfo("Connecting to action " + action)

        # Subscribe to the base action server
        self.action_client = actionlib.SimpleActionClient(action, action_type)

        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait up to timeout seconds for the action server to become available
        try:
            self.action_client.wait_for_server(rospy.Duration(self.connect_timeout))
        except:
            rospy.loginfo("Timed out connecting to the action server " + action)
    
        rospy.loginfo("Connected to action server")

    def run(self):
        # Send the goal
        if not self.action_started:
            rospy.loginfo("Sending " + str(self.name) + " goal to action server...")
            self.action_client.send_goal(self.goal, done_cb=self.done_cb, active_cb=self.active_cb, feedback_cb=self.feedback_cb)
            self.action_started = True 
        
        ''' We cannot use the wait_for_result() method here as it will block the entire
            tree so we break it down in time slices of duration 1 / rate.
        '''
        if not self.action_finished:
            self.time_so_far += self.tick
            self.rate.sleep()
            if self.time_so_far > self.result_timeout:
                self.action_client.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
                return TaskStatus.FAILURE
            else:
                return TaskStatus.RUNNING
        else:
            # Check the final goal status returned by default_done_cb
            if self.goal_status == GoalStatus.SUCCEEDED:
                  self.action_finished = True
                  if self.reset_after:
                      self.reset()
                  return TaskStatus.SUCCESS
            elif self.goal_status == GoalStatus.ABORTED:
                self.action_started = False
                self.action_finished = False
                return TaskStatus.FAILURE
            else:
                self.action_started = False
                self.action_finished = False
                self.goal_status_reported = False
                return TaskStatus.RUNNING
                            
    def default_done_cb(self, status, result):
        # Check the final status
        self.goal_status = status
        self.action_finished = True
        
        if not self.goal_status_reported:
            rospy.loginfo(str(self.name) + " ended with status " + str(self.goal_states[status]))
            self.goal_status_reported = True
    
    def default_active_cb(self):
        pass
        
    def default_feedback_cb(self, msg):
        pass
    
    def reset(self):
        self.action_started = False
        self.action_finished = False
        self.goal_status_reported = False
        self.time_so_far = 0.0
