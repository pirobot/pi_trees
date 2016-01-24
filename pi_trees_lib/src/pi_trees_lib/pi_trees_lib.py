#!/usr/bin/env python

"""
    pi_trees_lib.py - Version 0.1 2013-08-28
    
    Core classes for implementing Behavior Trees in Python
    
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

import string
import random

# import pygraphviz as pgv
# from pygraph.classes.graph import graph
# from pygraph.classes.digraph import digraph
# from pygraph.algorithms.searching import breadth_first_search
# from pygraph.readwrite.dot import write
# import gv

class TaskStatus(object):
    """ A class for enumerating task statuses """
    FAILURE = 0
    SUCCESS = 1
    RUNNING = 2
    
class Task(object):
    """ "The base Task class """
    def __init__(self, name, children=None, *args, **kwargs):
        self.name = name
        self.status = None
                
        if children is None:
            children = []
            
        self.children = children
                         
    def run(self):
        pass

    def reset(self):
        for c in self.children:
            c.reset()

    def add_child(self, c):
        self.children.append(c)

    def remove_child(self, c):
        self.children.remove(c)
        
    def prepend_child(self, c):
        self.children.insert(0, c)
        
    def insert_child(self, c, i):
        self.children.insert(i, c)
        
    def get_status(self):
        return self.status
    
    def set_status(self, s):
        self.status = s
    
    def announce(self):
        print("Executing task " + str(self.name))
    
    # These next two functions allow us to use the 'with' syntax
    def __enter__(self):
        return self.name
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if  exc_type is not None:
            return False
        return True

class Selector(Task):
    """ A selector runs each task in order until one succeeds,
        at which point it returns SUCCESS. If all tasks fail, a FAILURE
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """
    def __init__(self, name, *args, **kwargs):
        super(Selector, self).__init__(name, *args, **kwargs)
 
    def run(self):
        for c in self.children:
            
            c.status = c.run()
            
            if c.status != TaskStatus.FAILURE:
                return c.status

        return TaskStatus.FAILURE
 
class Sequence(Task):
    """
        A sequence runs each task in order until one fails,
        at which point it returns FAILURE. If all tasks succeed, a SUCCESS
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """
    def __init__(self, name, *args, **kwargs):
        super(Sequence, self).__init__(name, *args, **kwargs)
 
    def run(self):
        for c in self.children:
            
            c.status = c.run()
                         
            if c.status != TaskStatus.SUCCESS:
                return c.status   
            
        return TaskStatus.SUCCESS
    
class RandomSelector(Task):
    """ A selector runs each task in random order until one succeeds,
        at which point it returns SUCCESS. If all tasks fail, a FAILURE
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """
    def __init__(self, name, *args, **kwargs):
        super(RandomSelector, self).__init__(name, *args, **kwargs)
        
        self.shuffled = False
 
    def run(self):
        if not self.shuffled:
            random.shuffle(self.children)
            self.shuffled = True
                    
        for c in self.children:
            
            c.status = c.run()
            
            if c.status != TaskStatus.FAILURE:
                if c.status == TaskStatus.SUCCESS:
                    self.shuffled = False

                return c.status

        self.shuffled = False

        return TaskStatus.FAILURE
    
class RandomSequence(Task):
    """
        A sequence runs each task in random order until one fails,
        at which point it returns FAILURE. If all tasks succeed, a SUCCESS
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """
    def __init__(self, name, *args, **kwargs):
        super(RandomSequence, self).__init__(name, *args, **kwargs)
        
        self.shuffled = False
 
    def run(self):
        if not self.shuffled:
            random.shuffle(self.children)
            self.shuffled = True
        
        for c in self.children:
            
            c.status = c.run()
                         
            if c.status != TaskStatus.SUCCESS:
                if c.status == TaskStatus.FAILURE:
                    self.shuffled = False
                    
                return c.status   

        self.shuffled = False

        return TaskStatus.SUCCESS
    
class Iterator(Task):
    """
        Iterate through all child tasks ignoring failure.
    """
    def __init__(self, name, *args, **kwargs):
        super(Iterator, self).__init__(name, *args, **kwargs)
 
    def run(self):
        for c in self.children:
            
            c.status = c.run()
                         
            if c.status != TaskStatus.SUCCESS and c.status != TaskStatus.FAILURE:
                return c.status
            
        return TaskStatus.SUCCESS
    
class RandomIterator(Task):
    """
        Iterate through all child tasks randomly (without replacement) ignoring failure.
    """
    def __init__(self, name, *args, **kwargs):
        super(RandomIterator, self).__init__(name, *args, **kwargs)
        
        self.shuffled = False
 
    def run(self):
        if not self.shuffled:
            random.shuffle(self.children)
            self.shuffled = True

        for c in self.children:
            
            c.status = c.run()
                         
            if c.status != TaskStatus.SUCCESS and c.status != TaskStatus.FAILURE:
                return c.status
            
        self.shuffled = False

        return TaskStatus.SUCCESS
    
class ParallelOne(Task):
    """
        A parallel task runs each child task at (roughly) the same time.
        The ParallelOne task returns success as soon as any child succeeds.
    """
    def __init__(self, name, *args, **kwargs):
        super(ParallelOne, self).__init__(name, *args, **kwargs)
                 
    def run(self):
        for c in self.children:
            c.status = c.run()

            if c.status == TaskStatus.SUCCESS:
                return TaskStatus.SUCCESS
        
        return TaskStatus.FAILURE
        
class ParallelAll(Task):
    """
        A parallel task runs each child task at (roughly) the same time.
        The ParallelAll task requires all subtasks to succeed for it to succeed.
    """
    def __init__(self, name, *args, **kwargs):
        super(ParallelAll, self).__init__(name, *args, **kwargs)
                 
    def run(self):
        n_success = 0
        n_children = len(self.children)

        for c in self.children:
            c.status = c.run()
            if c.status == TaskStatus.SUCCESS:
                n_success += 1

            if c.status == TaskStatus.FAILURE:
                return TaskStatus.FAILURE

        if n_success == n_children:
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.RUNNING
        
class Loop(Task):
    """
        Loop over one or more subtasks for the given number of iterations
        Use the value -1 to indicate a continual loop.
    """
    def __init__(self, name, announce=True, *args, **kwargs):
        super(Loop, self).__init__(name, *args, **kwargs)
        
        self.iterations = kwargs['iterations']
        self.announce = announce
        self.loop_count = 0
        self.name = name
        print("Loop iterations: " + str(self.iterations))
        
    def run(self):
        
        while True:
            if self.iterations != -1 and self.loop_count >= self.iterations:
                return TaskStatus.SUCCESS
                        
            for c in self.children:
                while True:
                    c.status = c.run()
                    
                    if c.status == TaskStatus.SUCCESS:
                        break

                    return c.status
                
                c.reset()
                
            self.loop_count += 1
            
            if self.announce:
                print(self.name + " COMPLETED " + str(self.loop_count) + " LOOP(S)")
                
class Limit(Task):
    """
        Limit the number of times a task can execute
    """
    def __init__(self, name, announce=True, *args, **kwargs):
        super(Limit, self).__init__(name, *args, **kwargs)
        
        self.max_executions = kwargs['max_executions']
        self.announce = announce
        self.execution_count = 0
        self.name = name
        print("Limit number of executions to: " + str(self.max_executions))
        
    def run(self):
        if self.execution_count >= self.max_executions:
            self.execution_count = 0
            
            if self.announce:
                print(self.name + " reached maximum number (" + str(self.max_executions) + ") of executions.")
                
            return TaskStatus.FAILURE
                    
        for c in self.children:
            c.status = c.run()
            self.execution_count += 1
            return c.status


class IgnoreFailure(Task):
    """
        Always return either RUNNING or SUCCESS.
    """
    def __init__(self, name, *args, **kwargs):
        super(IgnoreFailure, self).__init__(name, *args, **kwargs)
 
    def run(self):
        
        for c in self.children:
            
            c.status = c.run()
            
            if c.status == TaskStatus.FAILURE:
                return TaskStatus.SUCCESS
            else:
                return c.status

        return TaskStatus.SUCCESS
    
class TaskNot(Task):
    """
        Turn SUCCESS into FAILURE and vice-versa
    """
    def __init__(self, name, *args, **kwargs):
        super(TaskNot, self).__init__(name, *args, **kwargs)
 
    def run(self):
        
        for c in self.children:
            
            c.status = c.run()
            
            if c.status == TaskStatus.FAILURE:
                return TaskStatus.SUCCESS
            
            elif c.status == TaskStatus.FAILURE:
                return TaskStatus.SUCCESS
            
            else:
                return c.status
    
class AutoRemoveSequence(Task):
    """ 
        Remove each successful subtask from a sequence 
    """
    def __init__(self, name, *args, **kwargs):
        super(AutoRemoveSequence, self).__init__(name, *args, **kwargs)
 
    def run(self):
        for c in self.children:
            c.status = c.run()
            
            if c.status == TaskStatus.FAILURE:
                return TaskStatus.FAILURE
            
            if c.statuss == TaskStatus.RUNNING:
                return TaskStatus.RUNNING
        
            try:
                self.children.remove(self.children[0])
            except:
                return TaskStatus.FAILURE
                
        return TaskStatus.SUCCESS
    
class CallbackTask(Task):
    """ 
        Turn any callback function (cb) into a task
    """
    def __init__(self, name, cb=None, cb_args=[], cb_kwargs={}, **kwargs):
        super(CallbackTask, self).__init__(name, cb=None, cb_args=[], cb_kwargs={}, **kwargs)
        
        self.name = name
        self.cb = cb
        self.cb_args = cb_args
        self.cb_kwargs = cb_kwargs
  
    def run(self):
        status = self.cb(*self.cb_args, **self.cb_kwargs)
         
        if status == 0 or status == False:
            return TaskStatus.FAILURE
        
        elif status == 1 or status == True:
            return TaskStatus.SUCCESS
        
        else:
            return TaskStatus.RUNNING

class loop(Task):
    """
        Loop over one or more subtasks a given number of iterations
    """
    def __init__(self, task, iterations=-1):
        new_name = task.name + "_loop_" + str(iterations)
        super(loop, self).__init__(new_name)

        self.task = task
        self.iterations = iterations
        self.old_run = task.run
        self.old_reset = task.reset
        self.old_children = task.children
        self.loop_count = 0
        
        print("Loop iterations: " + str(self.iterations))
        
    def run(self):
        if self.iterations != -1 and self.loop_count >= self.iterations:
            return TaskStatus.SUCCESS

        print("Loop " + str(self.loop_count + 1))
            
        while True:
            self.status = self.old_run()
            
            if self.status == TaskStatus.SUCCESS:
                break
            else:
                return self.status
                
        self.old_reset()
        self.loop_count += 1

        self.task.run = self.run

        return self.task
    
class limit(Task):
    """
        Limit a task to the given number of executions
    """
    def __init__(self, task, max_executions=-1):
        new_name = task.name + "_limit_" + str(max_executions)
        super(limit, self).__init__(new_name)

        self.task = task
        self.max_executions = max_executions
        self.old_run = task.run
        self.old_reset = task.reset
        self.old_children = task.children
        self.execution_count = 0
        
        print("Limit number of executions to: " + str(self.max_executions))
        
    def run(self):
        if self.max_executions != -1 and self.execution_count >= self.max_executions:
            self.execution_count = 0
            
            if self.announce:
                print(self.name + " reached maximum number (" + str(self.max_executions) + ") of executions.")
                
            return TaskStatus.FAILURE
            
        while True:
            self.status = self.old_run()
            
            if self.status == TaskStatus.SUCCESS:
                break
            else:
                return self.status
                
        self.old_reset()
        self.execution_count += 1

        self.task.run = self.run

        return self.task
    
class ignore_failure(Task):
    """
        Always return either RUNNING or SUCCESS.
    """
    def __init__(self, task):
        new_name = task.name + "_ignore_failure"
        super(ignore_failure, self).__init__(new_name)

        self.task = task
        self.old_run = task.run
        
    def run(self):
        while True:    
            self.status = self.old_run()
            
            if self.status == TaskStatus.FAILURE:
                return TaskStatus.SUCCESS
            else:
                return self.status
        
        self.task.run = self.run
        
        return self.task
    
class task_not(Task):
    """
        Turn SUCCESS into FAILURE and vice-versa
    """
    def __init__(self, task):
        new_name = task.name + "_not"
        super(task_not, self).__init__(new_name)

        self.task = task
        self.old_run = task.run
        
    def run(self):
        while True:    
            self.status = self.old_run()
            
            if self.status == TaskStatus.FAILURE:
                return TaskStatus.SUCCESS
            
            elif self.status == TaskStatus.FAILURE:
                return TaskStatus.SUCCESS
            
            else:
                return self.status
        
        self.task.run = self.run
        
        return self.task

def print_tree(tree, indent=0):
    """
        Print an ASCII representation of the tree
    """
    for c in tree.children:
        print "    " * indent, "-->", c.name
        
        if c.children != []:
            print_tree(c, indent+1)
            
def print_phpsyntax_tree(tree):    
    """
        Print an output compatible with ironcreek.net/phpSyntaxTree
    """
    for c in tree.children:
        print "[" + string.replace(c.name, "_", "."),
        if c.children != []:
            print_phpsyntax_tree(c),
        print "]",
    
# def print_dot_tree(root):
#     """
#         Print an output compatible with the DOT synatax and Graphiz
#     """
#     gr = pgv.AGraph(rotate='0', bgcolor='lightyellow')
#     gr.node_attr['fontsize']='9'
#                 
#     def add_edges(root):
#         for c in root.children:
#             gr.add_edge((root.name, c.name))
#             if c.children != []:
#                 add_edges(c)
#                 
#     add_edges(root)
#     
#     st, order = breadth_first_search(gr, root=root.name)
#     
#     gst = digraph()
#     gst.add_spanning_tree(st)
#     
#     dot = write(gst)
#     gvv = gv.readstring(dot)
#     
#     gv.layout(gvv,'dot')
#     gv.render(gvv,'png','tree.png')
    
    #gr.write("tree.dot")
    
    # Draw as PNG
    #gr.layout(prog='dot')
    #gr.draw('tree.png')
    
                

 
