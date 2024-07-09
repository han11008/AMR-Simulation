import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from fleet_msgs.srv import TaskRequest
from fleet_msgs.msg import TaskResponse, TaskStatus, BotPose
import os, math
import numpy as np
from time import time

from PySide2.QtCore import QThread, Signal, Slot

ORIENTATION = {
	'ORIGINAL': 0,
	'CLOCKWISE90': 1,
	'CLOCKWISE180': 2,
	'CLOCKWISE270': 3
}

GET_POSE_TOLERANCE = 3.0

class TaskManageThread(QThread):
	update_signal = Signal(str, str, str, bool)

	def __init__(self):
		super().__init__()
		self.node = Node(f'task_manage_node_{os.getpid()}')

		req_cbg = MutuallyExclusiveCallbackGroup()
		self.task_requester = self.node.create_client(TaskRequest, '/task_request', callback_group=req_cbg)

		res_cbg = MutuallyExclusiveCallbackGroup()
		transient_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
		_ = self.node.create_subscription(TaskResponse, '/task_response', self.getResponse, transient_qos, callback_group=res_cbg)

		self.reqs = []
		self.send_goal_timer = self.node.create_timer(1.0, self._sendGoal, callback_group=req_cbg)

	@Slot(str, str, bool, float, float)
	def assignTask(self, task_uuid, bot='', cancel=False, goal_x=0.0, goal_y=0.0):
		req = TaskRequest.Request()
		req.task_uuid = task_uuid
		req.bot_name = bot
		req.cancel = cancel
		req.goal.position.x = goal_x
		req.goal.position.y = goal_y
		self.reqs.append(req)

	def _sendGoal(self):
		if self.reqs != []:
			count = 0
			while not self.task_requester.wait_for_service(timeout_sec=1.5):
				count += 1
				if count > 5:
					for req in self.reqs:
						self.update_signal.emit(req.task_uuid, req.bot_name, 'Rejected', True)
					self.reqs = []
					return

			for req in self.reqs:
				send_future = self.task_requester.call_async(req)
				send_future.add_done_callback(self._sendGoalResult)
			self.reqs = []

	def _sendGoalResult(self, future):
		pass

	def getResponse(self, msg):
		if msg.task_status.status == TaskStatus.TASK_EXECUTING:
			self.update_signal.emit(msg.task_uuid, msg.bot_name, 'Executing', False)
		elif msg.task_status.status == TaskStatus.TASK_SUCCEEDED:
			self.update_signal.emit(msg.task_uuid, msg.bot_name, 'Succeeded', True)
		elif msg.task_status.status == TaskStatus.TASK_CANCELLED:
			self.update_signal.emit(msg.task_uuid, msg.bot_name, 'Cancelled', True)
		elif msg.task_status.status == TaskStatus.TASK_ABORTED:
			self.update_signal.emit(msg.task_uuid, msg.bot_name, 'Aborted', True)

	def run(self):
		self.executor = MultiThreadedExecutor()
		self.executor.add_node(self.node)
		try:
			self.executor.spin()
		except:
			pass

	def stop(self):
		self.node.destroy_node()
		self.executor.shutdown()

class PosUpdaterThread(QThread):
	update_signal = Signal(str, int, int, float)

	def __init__(self, map_ori_x, map_ori_y, phy_ori_x, phy_ori_y, map_resolution, map_orientation):
		super().__init__()
		self.map_ori_x = map_ori_x
		self.map_ori_y = map_ori_y
		self.phy_ori_x = phy_ori_x
		self.phy_ori_y = phy_ori_y
		self.map_resolution = map_resolution
		self.map_orientation = map_orientation
		self.fleet = {}

		self.node = Node(f'fleet_pose_listener_{os.getpid()}')
		sub_cbg = MutuallyExclusiveCallbackGroup()
		_ = self.node.create_subscription(BotPose, '/fleet_pose', self.getPose, 10, callback_group=sub_cbg)

		check_cbg = MutuallyExclusiveCallbackGroup()
		self.check_fleet_timer = self.node.create_timer(1.0, self._checkFleet, callback_group=check_cbg)

	@Slot(int, int, float)
	def editParam(self, map_ori_x, map_ori_y, map_resolution):
		self.map_ori_x = map_ori_x
		self.map_ori_y = map_ori_y
		self.map_resolution = map_resolution

	def phyPosToMap(self, phy_x, phy_y):
		map_x = (phy_x - self.phy_ori_x) / self.map_resolution
		map_y = (phy_y - self.phy_ori_y) / self.map_resolution

		if self.map_orientation == ORIENTATION['CLOCKWISE90']:
			tmp_x = map_x
			map_x = map_y
			map_y = -tmp_x
		elif self.map_orientation == ORIENTATION['CLOCKWISE180']:
			map_x = -map_x
			map_y = -map_y
		elif self.map_orientation == ORIENTATION['CLOCKWISE270']:
			tmp_x = map_x
			map_x = -map_y
			map_y = tmp_x

		map_x = round(self.map_ori_x + map_x)
		map_y = round(self.map_ori_y - map_y)

		return (map_x, map_y)

	def getPose(self, msg):
		self.fleet[msg.bot_name] = time()
		x, y = self.phyPosToMap(msg.current_pose.position.x, msg.current_pose.position.y)
		self.update_signal.emit(msg.bot_name, x, y, self.getOrientation(msg.current_pose.orientation.z, msg.current_pose.orientation.w))

	def getOrientation(self, qz, qw):
		return math.degrees(math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz)))

	def _checkFleet(self):
		bots = list(self.fleet.keys())
		current_time = time()
		for bot in bots:
			if current_time - self.fleet[bot] > GET_POSE_TOLERANCE:
				self.update_signal.emit(bot, -1, -1, 0.0)
				_ = self.fleet.pop(bot)

	def run(self):
		self.executor = MultiThreadedExecutor()
		self.executor.add_node(self.node)
		try:
			self.executor.spin()
		except:
			pass

	def stop(self):
		self.node.destroy_node()
		self.executor.shutdown()

