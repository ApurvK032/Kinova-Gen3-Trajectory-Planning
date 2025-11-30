#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import pickle

class TrajectoryExecutor(object):
    def __init__(self):
        super(TrajectoryExecutor, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('trajectory_executor')
        
        try:
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.is_init_success = True
            rospy.loginfo("Trajectory executor initialized successfully")
        except Exception as e:
            rospy.logerr(f"Initialization failed: {e}")
            self.is_init_success = False
    
    def execute_trajectories(self):
        if not self.is_init_success:
            return False
        
        arm_group = self.arm_group
        
        # Move to home first to match collection start position
        rospy.loginfo("Moving to home position first...")
        arm_group.set_named_target("home")
        arm_group.go(wait=True)
        arm_group.stop()
        rospy.sleep(2)
        
        # Load trajectories
        try:
            with open('/home/catkin_ws/trajectories.pkl', 'rb') as f:
                trajectories = pickle.load(f)
            rospy.loginfo(f"Loaded {len(trajectories)} trajectories")
        except Exception as e:
            rospy.logerr(f"Failed to load trajectories: {e}")
            return False
        
        # Execute each trajectory
        for i, traj in enumerate(trajectories, start=1):
            rospy.loginfo(f"Executing trajectory {i}/{len(trajectories)}...")
            success = arm_group.execute(traj, wait=True)
            arm_group.stop()
            if success:
                rospy.loginfo(f"Trajectory {i} executed successfully")
            else:
                rospy.logerr(f"Trajectory {i} execution failed")
            rospy.sleep(2)
        
        rospy.loginfo("All trajectories executed!")
        return True

def main():
    executor = TrajectoryExecutor()
    if executor.is_init_success:
        executor.execute_trajectories()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
