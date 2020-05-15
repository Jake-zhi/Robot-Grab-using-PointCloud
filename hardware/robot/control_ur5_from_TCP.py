#!/usr/bin/env python
# -*- coding: utf-8 -*-

socket_test = False

import socket
if not socket_test:
	import rospy, sys
	import moveit_commander
	from geometry_msgs.msg import PoseStamped, Pose


# 获取本机ip地址
def get_my_ip():
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		s.connect(('www.baidu.com', 0))
		ip = s.getsockname()[0]
	except:
		ip = "x.x.x.x"
	finally:
		s.close()
	return ip


class UR5_robot:
	if not socket_test:
		arm = moveit_commander.MoveGroupCommander('manipulator')  # 初始化需要使用move group控制的机械臂中的arm group
		end_effector_link = arm.get_end_effector_link()  # 获取终端link的名称
		reference_frame = 'base_link'  # 设置目标位置所使用的参考坐标系
		target_pose = PoseStamped()

	def __init__(self, node_name='UR5_robot', host_ip='localhost', port=12345):
		if not socket_test:
			# 初始化UR5和moveit
			moveit_commander.roscpp_initialize(sys.argv)  # 初始化move_group的API
			rospy.init_node(node_name)  # 初始化ROS节点
			self.arm.allow_replanning(True)  # 当运动规划失败后，允许重新规划
			self.arm.set_pose_reference_frame(self.reference_frame)
			self.arm.set_goal_position_tolerance(0.001)  # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
			self.arm.set_goal_orientation_tolerance(0.01)
			self.arm.set_max_acceleration_scaling_factor(0.5)  # 设置允许的最大速度和加速度
			self.arm.set_max_velocity_scaling_factor(0.5)
			# 控制机械臂先回到初始化位置
			self.arm.set_named_target('home')
			self.arm.go()
			rospy.sleep(1)
		
		# 初始化socket
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		print("host address:%s:%s" % (host_ip, port))
		Arr = (host_ip, port)
		s.bind(Arr)
		s.listen(1)
		print("Waiting For Client...")
		self.sock, addr = s.accept()  # 接收client发送的信息
		info = self.sock.recv(1024).decode()
		if info == 'hello':
			print('Connection Success')
		else:
			print('''Warning : reveived something, but not 'hello', maybe there is some encoding problem.''')
		print("robot and tcp server init done.")
	
	if not socket_test:
		def __del__(self):
		# 关闭并退出moveit
			moveit_commander.roscpp_shutdown()
	
	if not socket_test:
		# 输入为pose_stamped
		def move_to_target_pose(self):
			self.arm.set_start_state_to_current_state()  # 设置机器臂当前的状态作为运动初始状态
			self.arm.set_pose_target(self.target_pose, self.end_effector_link)  # 设置机械臂终端运动的目标位姿
			traj = self.arm.plan()  # 规划运动路径
			self.arm.execute(traj)  # 按照规划的运动路径控制机械臂运动
			rospy.sleep(1)
			
	# 收到字符串，字符串为七个浮点数，空格隔开
	def receive_command(self):
		info = self.sock.recv(1024).decode()  # 接收client发送的信息
		data = info.split()
		if len(data) != 7:
			print('data format error.')
			return

		# 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
		# 姿态使用四元数描述，基于base_link坐标系
		print("target_pose is:", data)
		if not socket_test:
			self.target_pose.header.frame_id = self.reference_frame
			self.target_pose.header.stamp = rospy.Time.now()
			self.target_pose.pose.position.x = float(data[0])
			self.target_pose.pose.position.y = float(data[1])
			self.target_pose.pose.position.z = float(data[2])
			self.target_pose.pose.orientation.x = float(data[3])
			self.target_pose.pose.orientation.y = float(data[4])
			self.target_pose.pose.orientation.z = float(data[5])
			self.target_pose.pose.orientation.w = float(data[6])
			

	def receive_command_and_go(self):
		self.receive_command()
		if not socket_test:
			self.move_to_target_pose()


if __name__ == "__main__":
	robot = UR5_robot()
	while 1:
		robot.receive_command_and_go()
