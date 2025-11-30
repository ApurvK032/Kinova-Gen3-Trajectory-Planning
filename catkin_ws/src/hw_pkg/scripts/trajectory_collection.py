#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import pickle

class TrajectoryCollector(object):
    def __init__(self):
        super(TrajectoryCollector, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('trajectory_collector')
        
        try:
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.is_init_success = True
            rospy.loginfo("Trajectory collector initialized successfully")
        except Exception as e:
            rospy.logerr(f"Initialization failed: {e}")
            self.is_init_success = False
    
    def plan_and_execute_then_save(self, trajectory_name, target_type, target_value):
        """Plan a trajectory, execute it, then save the trajectory"""
        arm_group = self.arm_group
        
        rospy.loginfo(f"Planning {trajectory_name}...")
        
        if target_type == "named":
            arm_group.set_named_target(target_value)
        elif target_type == "joint":
            arm_group.set_joint_value_target(target_value)
        
        # Plan the trajectory
        success, traj, time, error = arm_group.plan()
        
        if not success:
            rospy.logerr(f"Failed to plan {trajectory_name}")
            return None
        
        rospy.loginfo(f"{trajectory_name} planned successfully")
        
        # Execute the trajectory
        rospy.loginfo(f"Executing {trajectory_name}...")
        exec_success = arm_group.execute(traj, wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        
        if exec_success:
            rospy.loginfo(f"{trajectory_name} executed successfully")
            rospy.sleep(2)
            return traj
        else:
            rospy.logerr(f"Failed to execute {trajectory_name}")
            return None
    
    def collect_trajectories(self):
        if not self.is_init_success:
            return False
        
        arm_group = self.arm_group
        trajectories = []
        
        # I1: Start from home
        rospy.loginfo("I1: Moving to home position...")
        arm_group.set_named_target("home")
        arm_group.go(wait=True)
        arm_group.stop()
        rospy.sleep(2)
        
        # T1: Home -> Vertical
        traj1 = self.plan_and_execute_then_save("T1: Home to Vertical", "named", "vertical")
        if traj1:
            trajectories.append(traj1)
        
        # T2: Vertical -> Home
        traj2 = self.plan_and_execute_then_save("T2: Vertical to Home", "named", "home")
        if traj2:
            trajectories.append(traj2)
        
        # T3: Home -> Small joint change
        joint_goal = arm_group.get_current_joint_values()
        joint_goal[0] += 0.3
        joint_goal[2] += 0.2
        traj3 = self.plan_and_execute_then_save("T3: Home to Custom Pose 1", "joint", joint_goal)
        if traj3:
            trajectories.append(traj3)
        
        # T4: Custom -> Different custom
        joint_goal = arm_group.get_current_joint_values()
        joint_goal[1] += 0.25
        joint_goal[4] += 0.2
        traj4 = self.plan_and_execute_then_save("T4: Custom Pose 1 to Custom Pose 2", "joint", joint_goal)
        if traj4:
            trajectories.append(traj4)
        
        # T5: Custom 2 -> Home
        traj5 = self.plan_and_execute_then_save("T5: Custom Pose 2 to Home", "named", "home")
        if traj5:
            trajectories.append(traj5)
        
        # Save all trajectories
        if len(trajectories) == 5:
            with open('/home/catkin_ws/trajectories.pkl', 'wb') as f:
                pickle.dump(trajectories, f)
            rospy.loginfo(f"SUCCESS! Saved {len(trajectories)} trajectories!")
            return True
        else:
            rospy.logerr(f"Only collected {len(trajectories)}/5 trajectories")
            if len(trajectories) > 0:
                with open('/home/catkin_ws/trajectories.pkl', 'wb') as f:
                    pickle.dump(trajectories, f)
                rospy.loginfo(f"Saved {len(trajectories)} trajectories anyway")
            return False

def main():
    collector = TrajectoryCollector()
    if collector.is_init_success:
        collector.collect_trajectories()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
