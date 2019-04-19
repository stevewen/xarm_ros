#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
import numpy as np

class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('xarm6')
        
        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = moveit_commander.MoveGroupCommander('hand')
        
        # 设置机械臂和夹爪的允许误差值
        arm.set_goal_joint_tolerance(0.001)
        gripper.set_goal_joint_tolerance(0.001)
        
        # 控制机械臂先回到初始化位置
        # arm.set_named_target('home')
        # arm.go()
        # rospy.sleep(2)
         
        # 设置夹爪的目标位置，并控制夹爪运动
        # gripper.set_joint_value_target([0.01])
        # gripper.go()
        # rospy.sleep(1)

        # f=open("/home/zeal/xarm_ros/src/xarm_planner/datas/joint_data.txt")
        # data = f.read() 
        # f.close()
        # print(data)

        file = open("/home/zeal/xarm_ros/src/xarm_planner/datas/joint_data.txt") 
        dataMat=[] 
        for line in file.readlines():  
             curLine=line.strip().split('\t') 
             floatLine=map(float,curLine)   
             dataMat.append(floatLine)  
        file.close() 
        print(dataMat[2][0])
        
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        # joint_positions = [0.0, -0.0, -0.0, 0.0, 0.0, 0.0]
        #joint_positions = [0.5, -0.0, -2.8, 0.0, 1.0, 0.0]
        joint_positions = [dataMat[0][0], dataMat[1][0], dataMat[2][0], dataMat[3][0], dataMat[4][0], dataMat[5][0]]
        arm.set_joint_value_target(joint_positions)
                 
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass