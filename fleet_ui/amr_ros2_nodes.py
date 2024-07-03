import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from fleet_msgs.srv import TaskRequest
from fleet_msgs.msg import TaskResponse, TaskStatus
import os
import numpy as np
from time import time

from PySide2.QtCore import QThread, Signal, Slot

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

