#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def eul_to_qua(Eular):
    Eular_Div = [0, 0, 0]
    Eular_Div[0], Eular_Div[1], Eular_Div[2] = Eular[0]/2.0, Eular[1]/2.0, Eular[2]/2.0
        
    ca, cb, cc = math.cos(Eular_Div[0]), math.cos(Eular_Div[1]), math.cos(Eular_Div[2])
    sa, sb, sc = math.sin(Eular_Div[0]), math.sin(Eular_Div[1]), math.sin(Eular_Div[2])
        
    x = sa*cb*cc - ca*sb*sc
    y = ca*sb*cc + sa*cb*sc
    z = ca*cb*sc - sa*sb*cc
    w = ca*cb*cc + sa*sb*sc

    orientation = Quaternion()
    orientation.x, orientation.y, orientation.z, orientation.w = x, y, z, w
    return orientation

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('xarm6')
                
        # 获取终端link的名称
        # end_effector_link = arm.get_end_effector_link()
        end_effector_link = 'link6'
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'link_b1'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)
        
        # 控制机械臂先回到初始化位置
        # arm.set_named_target('home')
        # arm.go()
        # rospy.sleep(1)

        file = open("/home/zeal/xarm_ros/src/xarm_planner/datas/Cartesian_data.txt") 
        dataMat=[] 
        for line in file.readlines():  
             curLine=line.strip().split('\t') 
             floatLine=map(float,curLine)   
             dataMat.append(floatLine)  
        file.close() 
        print(dataMat[2][0])

        pitch = dataMat[3][0]*math.pi/180.0
        yaw = dataMat[4][0]*math.pi/180.0
        roll = dataMat[5][0]*math.pi/180.0
        Eular = [pitch, yaw, roll]

        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = dataMat[0][0]
        target_pose.pose.position.y = dataMat[1][0]
        target_pose.pose.position.z = dataMat[2][0]
        # target_pose.pose.orientation.x = dataMat[3][0]
        # target_pose.pose.orientation.y = dataMat[4][0]
        # target_pose.pose.orientation.z = dataMat[5][0]
        # target_pose.pose.orientation.w = -0.293653
        target_pose.pose.orientation = eul_to_qua(Eular)
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(1)
         
        # 控制机械臂终端向右移动5cm
        # arm.shift_pose_target(1, -0.05, end_effector_link)
        # arm.go()
        # rospy.sleep(1)
  
        # 控制机械臂终端反向旋转90度
        # arm.shift_pose_target(3, -1.57, end_effector_link)
        # arm.go()
        # rospy.sleep(1)
           
        # 控制机械臂回到初始化位置
        # arm.set_named_target('home')
        # arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    