#!/usr/bin/env python

"""
    monitor_topic.py - Version 1.0 2013-09-22
    
    Monitor a ROS topic for a target value
    
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
from std_msgs.msg import Float32
from pi_trees_ros import *
import time

class MonitorTopicExample():
    def __init__(self):
        # The root node
        BEHAVE = Sequence("behave")
        
        # The "stay charged" node
        STAY_CHARGED = Selector("stay_charged")
        
        # Check the battery level (uses MonitorTask)
        CHECK_BATTERY = MonitorTask("CHECK_BATTERY", "battery_level", Float32, self.check_battery)
        
        # The charge robot task (uses ServiceTask)
        CHARGE_ROBOT = ServiceTask("CHARGE_ROBOT", "battery_simulator/set_battery_level", SetBatteryLevel, 100, result_cb=self.recharge_cb)
  
        # Build the recharge sequence using inline construction
        RECHARGE = Sequence("RECHARGE", [NAV_DOCK_TASK, CHARGE_ROBOT])
        
        # The check battery condition
        CHECK_BATTERY = CheckBattery("check_battery", "battery_level", Float32, True)

        # Add the check battery and recharge tasks to the stay healthy task
        STAY_CHARGED.add_child(CHECK_BATTERY)
        
        COUNT = loop(Count("count 3", n=3), iterations=2)
        
        #LOOP = Loop("loop 2", iterations=2)
        #LOOP.add_child(COUNT)

        # Build the full tree from the two subtrees
        BEHAVE.add_child(STAY_CHARGED)
        BEHAVE.add_child(COUNT)
                
        print "Behavior Tree Structure"
        print_tree(BEHAVE)
            
        # Run the tree
        while True:
            status = BEHAVE.run()
            if status == TaskStatus.SUCCESS:
                print "Finished running tree."
                break
    
class Count(Task):
    def __init__(self, name, *args, **kwargs):
        super(Count, self).__init__(name, *args, **kwargs)
        
        self.n = kwargs['n']
        self.count = 0
        print "Creating task Count", self.n
 
    def run(self):
        if self.count < self.n:
            print "Counting...", self.count
            time.sleep(0.5)
            self.count += 1
            return TaskStatus.RUNNING

        return TaskStatus.SUCCESS
    
    def reset(self):
        self.count = 0
        
class CheckBattery(MonitorTopic):
    def __init__(self, name, *args):
        super(CheckBattery, self).__init__(name, *args, msg_callback=self.msg_callback, timeout=10)
        self.battery_level = None
        self.low_battery_threshold = 50
        
    def msg_callback(self, msg):
        print msg
        self.battery_level = msg.data
 
    def run(self):
        while self.battery_level is None:
            rospy.sleep(0.5)

        if self.battery_level < self.low_battery_threshold and self.battery_level is not None:
            print("LOW BATTERY!")
            return TaskStatus.FAILURE
        else:
            return TaskStatus.SUCCESS


if __name__ == '__main__':
    tree = MonitorTopicExample()

