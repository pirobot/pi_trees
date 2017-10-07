#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rbx2_msgs.srv import *
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

class BlackBoard():
    def __init__(self):
        self.battery_level = None
        self.charging = None

class Patrol():
    def __init__(self):
        rospy.init_node("patrol_tree")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # Initialize the black board
        self.blackboard = BlackBoard()
        
        # How frequently do we "tic" the tree?
        rate = rospy.get_param('~rate', 10)
        
        # Convert tic rate to a ROS rate
        tic = rospy.Rate(rate)
        
        # Where should the DOT file be stored.  Default location is $HOME/.ros/tree.dot
        dotfilepath = rospy.get_param('~dotfilepath', None)

        # Create a list to hold the move_base tasks
        MOVE_BASE_TASKS = list()
        
        n_waypoints = len(self.waypoints)
        
        # Create simple action navigation task for each waypoint
        for i in range(n_waypoints):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.waypoints[i % n_waypoints]
            
            move_base_task = SimpleActionTask("MOVE_BASE_TASK_" + str(i), "move_base", MoveBaseAction, goal, reset_after=False)
            
            MOVE_BASE_TASKS.append(move_base_task)
        
        # Set the docking station pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.docking_station_pose
        
        # Assign the docking station pose to a move_base action task
        NAV_DOCK_TASK = SimpleActionTask("NAV_DOC_TASK", "move_base", MoveBaseAction, goal, reset_after=False)
        
        # Create the root node
        BEHAVE = Sequence("BEHAVE")
        
        # Create the "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY")
        
        # Create the patrol loop decorator
        LOOP_PATROL = Loop("LOOP_PATROL", iterations=self.n_patrols)
        
        # Add the two subtrees to the root node in order of priority
        BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(LOOP_PATROL)
        
        # Create the patrol iterator
        PATROL = Iterator("PATROL")
        
        # Add the move_base tasks to the patrol task
        for task in MOVE_BASE_TASKS:
            PATROL.add_child(task)
  
        # Add the patrol to the loop decorator
        LOOP_PATROL.add_child(PATROL)
        
        # Add the battery check and recharge tasks to the "stay healthy" task
        with STAY_HEALTHY:
            # Monitor the fake battery level by subscribing to the /battery_level topic
            MONITOR_BATTERY = MonitorTask("MONITOR_BATTERY", "battery_level", Float32, self.monitor_battery)
            
            # Is the fake battery level below threshold?
            CHECK_BATTERY = CallbackTask("BATTERY_OK?", self.check_battery)  
            
            # Set the fake battery level back to 100 using a ServiceTask
            CHARGE_COMPLETE = ServiceTask("CHARGE_COMPLETE", "/battery_simulator/set_battery_level", SetBatteryLevel, 100, result_cb=self.recharge_cb)
            
            # Sleep for the given interval to simulate charging
            CHARGING = RechargeRobot("CHARGING", interval=3, blackboard=self.blackboard)
      
            # Build the recharge sequence using inline construction
            RECHARGE = Sequence("RECHARGE", [NAV_DOCK_TASK, CHARGING, CHARGE_COMPLETE], reset_after=True)
                
            # Add the check battery and recharge tasks to the stay healthy selector
            STAY_HEALTHY.add_child(CHECK_BATTERY)
            STAY_HEALTHY.add_child(RECHARGE)
                
        # Display the tree before beginning execution
        print "Patrol Behavior Tree"
        print_tree(BEHAVE)
        
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.status = BEHAVE.run()
            tic.sleep()
            print_dot_tree(BEHAVE, dotfilepath)
            
    def monitor_battery(self, msg):
        # Store the battery level as published on the fake battery level topic
        self.blackboard.battery_level = msg.data
        return True
    
    def check_battery(self):
        # Don't run the check if we are charging
        if self.blackboard.charging:
            return False
        
        if self.blackboard.battery_level is None:
            return None
        elif self.blackboard.battery_level < self.low_battery_threshold:
            rospy.loginfo("LOW BATTERY - level: " + str(int(self.blackboard.battery_level)))
            return False
        else:
            return True
    
    def recharge_cb(self, result):
        rospy.loginfo("BATTERY CHARGED!")
        self.blackboard.battery_level = 100
        self.blackboard.charging = False
        rospy.sleep(2)
        return True
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        
class RechargeRobot(Task):
    def __init__(self, name, interval=3, blackboard=None):
        super(RechargeRobot, self).__init__(name)
       
        self.name = name
        self.interval = interval
        self.blackboard = blackboard
        
        self.timer = 0
         
    def run(self):
        if self.timer == 0:
            rospy.loginfo("CHARGING THE ROBOT!")
            
        if self.timer < self.interval:
            self.timer += 0.1
            rospy.sleep(0.1)
            self.blackboard.charging = True
            return TaskStatus.RUNNING
        else:
            return TaskStatus.SUCCESS
    
    def reset(self):
        self.status = None
        self.timer = 0

if __name__ == '__main__':
    tree = Patrol()

