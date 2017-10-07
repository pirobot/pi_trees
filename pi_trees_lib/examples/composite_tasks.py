#!/usr/bin/env python

from pi_trees_lib.pi_trees_lib import *
import time

class CompositeTasks():
    def __init__(self):
        # The root node
        BEHAVE = Sequence("behave")
        
        # Create a Selector composite task (returns SUCCESS as soon as any subtask returns SUCCESS)
        SELECTOR_TASK = Selector("Selector Count")

        # Create a Sequence composite task (returns FAULURE as soon as any subtask returns FAILURE)
        SEQUENCE_TASK = Sequence("Sequence Count")
        
        WAIT_TASK = WaitTask("Wait Task", 5)
        
        # Create three counting tasks
        COUNT2 = Count("Count+2", 1, 2, 1)
        COUNT5 = Count("Count-5", 5, 1, -1)
        COUNT16 = Count("Count+16", 1, 16, 1)

        # Add the tasks to the sequence composite task
        SEQUENCE_TASK.add_child(COUNT2)
        SEQUENCE_TASK.add_child(WAIT_TASK)
        SEQUENCE_TASK.add_child(COUNT5)
        SEQUENCE_TASK.add_child(COUNT16)
        
        # Add the tasks to the selector composite task
        SELECTOR_TASK.add_child(COUNT5)
        SELECTOR_TASK.add_child(COUNT2)
        SELECTOR_TASK.add_child(COUNT16)
        
        # Add the composite task to the root task
        BEHAVE.add_child(SEQUENCE_TASK)
        BEHAVE.add_child(SELECTOR_TASK)

        # Print a simple representation of the tree
        print "Behavior Tree Structure"
        print_tree(BEHAVE, use_symbols=True)
            
        # Run the tree
        while True:
            status = BEHAVE.run()
            if status == TaskStatus.SUCCESS:
                print "Finished running tree."
                break

# A counting task that extends the base Task task
class Count(Task):
    def __init__(self, name, start, stop, step, *args, **kwargs):
        super(Count, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.start = start
        self.stop = stop
        self.step = step
        self.count = self.start
        print "Creating task Count", self.start, self.stop, self.step
 
    def run(self):
        if abs(self.count - self.stop - self.step) <= 0:
            return TaskStatus.SUCCESS
        else:
            print self.name, self.count
            time.sleep(0.5)
            self.count += self.step
            if abs(self.count - self.stop - self.step) <= 0:
                return TaskStatus.SUCCESS
            else:
                return TaskStatus.RUNNING

    
    def reset(self):
        self.count = self.start

if __name__ == '__main__':
    tree = CompositeTasks()

