#!/usr/bin/env python

"""
    counting.py - Version 1.0 2013-09-22
    
    Perform a counting task while monitoring a fake battery level
    
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

from pi_trees.pi_trees import *
import time

class CountingExample():
    def __init__(self):
        # The root node
        BEHAVE = Sequence("behave")
        
        # The "stay charged" node
        STAY_CHARGED = Selector("stay_charged")
        
        # The check battery condition
        CHECK_BATTERY = FakeCheckBattery("check_battery")

        # Add the check battery and recharge tasks to the stay healthy task
        STAY_CHARGED.add_child(CHECK_BATTERY)
        
        COUNT3 = Count("count 3", n=3)
        COUNT5 = Count("count 5", n=5)
        COUNT7 = Count("count 7", n=7)
        
        LOOP2 = Loop("loop 2", iterations=2)
        LOOP2.add_child(COUNT3)

        # Build the full tree from the two subtrees
        #BEHAVE.add_child(STAY_CHARGED)
        BEHAVE.add_child(LOOP2)
        BEHAVE.add_child(COUNT5)
        BEHAVE.add_child(COUNT7)
                
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
        
class FakeCheckBattery(Task):
    def __init__(self, name, counter=3):
        super(FakeCheckBattery, self).__init__(name)
        print "Connecting to battery monitor"
        self.counter = counter
        self.level = counter
 
    def run(self):
        time.sleep(0.5)
        if self.level == 0:
            print("LOW BATTERY!")
            print("Recharging...")
            time.sleep(2)
            print("Battery Recharged")
            self.level = self.counter
        else:
            self.level -= 1
            print("Battery is OK.")
            return TaskStatus.SUCCESS


if __name__ == '__main__':
    tree = CountingExample()

