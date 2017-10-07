#!/usr/bin/env python

from pi_trees_lib.pi_trees_lib import *
import time

class ParallelExample():
    def __init__(self):
        # The root node
        BEHAVE = Sequence("behave")
        
        # Create a ParallelOne composite task (returns SUCCESS as soon as any subtask returns SUCCESS)
        PARALLEL_TASKS = ParallelOne("Counting in Parallel")
        
        # Create three counting tasks
        COUNT2 = Count("Count+2", 1, 2, 1)
        COUNT5 = Count("Count-5", 5, 1, -1)
        COUNT16 = Count("Count+16", 1, 16, 1)

        # Add the tasks to the parallel composite task
        PARALLEL_TASKS.add_child(COUNT5)
        PARALLEL_TASKS.add_child(COUNT2)
        PARALLEL_TASKS.add_child(COUNT16)
        
        # Add the composite task to the root task
        BEHAVE.add_child(PARALLEL_TASKS)
        
        # Print a simple representation of the tree
        print "Behavior Tree Structure"
        print_tree(BEHAVE)
            
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
    tree = ParallelExample()

