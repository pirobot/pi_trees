#!/usr/bin/env python

"""
    battery_simulator.py - Version 1.0 2013-03-18
    
    Publish a simulated battery level (0-100) with a configurable runtime.
    
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

import roslib; roslib.load_manifest('rbx2_utils')
import rospy
from diagnostic_msgs.msg import *
from std_msgs.msg import Float32
from rbx2_msgs.srv import *
from dynamic_reconfigure.server import Server
import dynamic_reconfigure.client
from rbx2_utils.cfg import BatterySimulatorConfig
import thread

class Battery():
    def __init__(self):
        rospy.init_node("battery_simulator")
        
        self.rate = rospy.get_param("~rate", 1)
        r = rospy.Rate(self.rate)
        
        self.battery_runtime = rospy.get_param("~battery_runtime", 30) # seconds
        
        self.initial_battery_level = rospy.get_param("~initial_battery_level", 100)
        
        self.current_battery_level = self.initial_battery_level
        
        self.reset_battery_level = self.initial_battery_level
        
        self.battery_step = float(self.initial_battery_level) / self.rate / self.battery_runtime

        # Reserve a thread lock
        self.mutex = thread.allocate_lock()
        
        # Fire up the dynamic_reconfigure server
        dyn_server = Server(BatterySimulatorConfig, self.dynamic_reconfigure_callback)
        
        # Connect to the dynamic_reconfigure server
        dyn_client = dynamic_reconfigure.client.Client("battery_simulator", timeout=60)
        
        # A service to maually set the battery level
        rospy.Service('~charge_battery', ChargeBattery, self.ChargeBatteryHandler)

        diag_pub = rospy.Publisher("diagnostics", DiagnosticArray)
        
        battery_level_pub = rospy.Publisher("battery_level", Float32)

        rospy.loginfo("Publishing simulated battery level with a runtime of " + str(self.battery_runtime) + " seconds...")

        while not rospy.is_shutdown():
            status = DiagnosticStatus()
            status.name = "Battery Level"

            if self.current_battery_level < 20:
                status.message = "Low Battery"
                status.level = DiagnosticStatus.ERROR
            elif self.current_battery_level < 50:
                status.message = "Medium Battery"
                status.level = DiagnosticStatus.WARN     
            else:
                status.message = "Battery OK"
                status.level = DiagnosticStatus.OK
            
            status.values.append(KeyValue("Battery Level", str(self.current_battery_level)))
            
            msg = DiagnosticArray()
            msg.header.stamp = rospy.Time.now()
            msg.status.append(status)
            
            diag_pub.publish(msg)
            
            battery_level_pub.publish(self.current_battery_level)
            
            self.current_battery_level = max(0, self.current_battery_level - self.battery_step)
                        
            r.sleep()
            
    def dynamic_reconfigure_callback(self, config, level):
        if self.battery_runtime != config['battery_runtime']:
            self.battery_runtime = config['battery_runtime']
            self.battery_step = 100.0 / self.rate / self.battery_runtime
            
        if self.reset_battery_level != config['reset_battery_level']:
            self.reset_battery_level = config['reset_battery_level']
            self.mutex.acquire()
            self.current_battery_level = self.reset_battery_level
            self.mutex.release()
                    
        return config
            
    def ChargeBatteryHandler(self, req):
        self.mutex.acquire()
        self.current_battery_level = req.value
        self.mutex.release()
        return ChargeBatteryResponse()

if __name__ == '__main__':
    Battery()

    
    