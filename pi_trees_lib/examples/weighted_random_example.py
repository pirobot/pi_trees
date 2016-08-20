#!/usr/bin/env python

"""
    random_example.py - Version 1.0 2016-08-20
    
    Select from a number of behaviors randomly.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2016 Patrick Goebel.  All rights reserved.

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

from pi_trees_lib.pi_trees_lib import *
import time

class WeightedRandomExample():
    def __init__(self):
        # The root node
        BEHAVE = Sequence("behave")
        
        # Create a ParallelOne composite task (returns SUCCESS as soon as any subtask returns SUCCESS)
        WEIGHTED_RANDOM_TASKS = WeightedRandomSelector("Weighted Random Selector", weights=[3, 2, 1], reset_after=True)
        
        # Create three counting tasks
        TASK1 = Message("High_Probability", "Highest Probability Task")
        TASK2 = Message("Medium_Probability", "Medium Probability Task")
        TASK3 = Message("Low_Probability", "Lowest Probability Task")

        # Add the tasks to the parallel composite task
        WEIGHTED_RANDOM_TASKS.add_child(TASK1)
        WEIGHTED_RANDOM_TASKS.add_child(TASK2)
        WEIGHTED_RANDOM_TASKS.add_child(TASK3)
        
        LOOP = Loop("Loop_Forever", iterations=-1)
        
        LOOP.add_child(WEIGHTED_RANDOM_TASKS)
        
        # Add the composite task to the root task
        BEHAVE.add_child(LOOP)
        
        # Print a simple representation of the tree
        print "Behavior Tree Structure"
        print_tree(BEHAVE)
            
        # Run the tree
        while True:
            status = BEHAVE.run()
            if status == TaskStatus.SUCCESS:
                print "Finished running tree."
                break

# A task to print a message
class Message(Task):
    def __init__(self, name, message, *args, **kwargs):
        super(Message, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.message = message
        print "Creating message task: ", self.message
 
    def run(self):
        print self.message
        time.sleep(1)

        return TaskStatus.SUCCESS


if __name__ == '__main__':
    tree = WeightedRandomExample()

