import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from task_msgs.srv import TaskRequest, CancelRequest
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import os
import numpy as np

from PySide2.QtCore import QThread, Signal, Slot
from PySide2.QtGui import QPixmap, QImage

ORIENTATION = {
	'ORIGINAL': 0,
	'CLOCKWISE90': 1,
	'CLOCKWISE180': 2,
	'CLOCKWISE270': 3
}

class NavDaemonThread(QThread):
	feedback = Signal(str, bool)
	update_goal_id = Signal(int)

	def __init__(self, goals, max_resend=10, task_timeout=2.0, wait_response_timeout=10.0):
		super().__init__()
		self.goal_phy_coors = goals
		# Key: str((goal_x, goal_y)), value: patrol_stop_number
		self.goal_ids = {}
		for idx in range(len(goals)):
			x, y = goals[idx]
			self.goal_ids[f'{(x, y)}'] = idx
		self.max_resend = max_resend
		self.task_timeout = task_timeout
		self.wait_response_timeout = wait_response_timeout

		self.node = Node(f'navigation_commander_{os.getpid()}')
		# Send navigation goals to navigator_client with topic
		cmd_cbg = MutuallyExclusiveCallbackGroup()
		
		self.commander = self.node.create_client(TaskRequest, 'nav_goals', callback_group=cmd_cbg)
		self.goals_to_assign = None
		self.next_goal = None

		# Send cancel request to navigator_client with service
		self.cancel_requester = self.node.create_client(CancelRequest, 'nav_cancel', callback_group=cmd_cbg)

		# Monitor the status of navgation task through navigator_client with topic
		fb_cbg = MutuallyExclusiveCallbackGroup()
		self.feedback_listener = self.node.create_subscription(String, 'nav_feedback', self._getFeedback, 10, callback_group=fb_cbg)
		self.nav2_alive = False
		self.end_keywords = ['no response', 'abort', 'rejected']
		self.check_nav2_timer = None

	def assignTask(self, goals): # "goals" is an ordered list of patrol stop id
		self._taskEnded()

		if self.goals_to_assign != goals:
			self.goals_to_assign = goals.copy()
			self.num_goals = len(goals)

		if self.num_goals > 0:
			if self.num_goals > 1:
				goals *= 3
			self.num_waiting_goals = len(goals)
			self.next_goal = goals[0]
			goals_flatten = []
			for goal in goals:
				x, y = self.goal_phy_coors[goal]
				goals_flatten.append(x)
				goals_flatten.append(y)

			new_task = TaskRequest.Request()
			new_task.goals = goals_flatten
			count = 0
			while not self.commander.wait_for_service(timeout_sec=1.5):
				count += 1
				if count > 5:
					self.feedback.emit('Cannot start the navigation task since the server has no response, please try again later.', False)
					self._taskEnded()
					return
			self.commander.call_async(new_task)

	def _sendGoalResult(self, future):
		pass

	def cancelTask(self):
		while not self.cancel_requester.wait_for_service(timeout_sec=1.5):
			count += 1
			if count > 5:
				self.feedback.emit('Cannot cancel the navigation task since the server has no response, please try again later.', False)
				self._taskEnded()
				return
		cancel_req = CancelRequest.Request()
		cancel_req.cancel = True
		self.cancel_requester.call_async(cancel_req)

	def _taskEnded(self):
		if self.check_nav2_timer is not None:
			self.node.destroy_timer(self.check_nav2_timer)
			self.check_nav2_timer = None
		self.nav2_alive = False

	def _checkNav2Alive(self):
		# Periodically called by self.check_nav2_timer
		# The navigation task is considerred to be failed if self.getFeedback has not been called for self.wait_response_timeout
		if not self.nav2_alive:
			self.feedback.emit('Navigation server has no response, please try again later.', False)
			self._taskEnded()
		self.nav2_alive = False

	def _getGoalId(self, log):
		sep = log.split(': ')
		if len(sep) == 2:
			goal_id = self.goal_ids.get(sep[-1])
			return goal_id if goal_id is not None else self.next_goal
		return self.next_goal

	def _getFeedback(self, msg):
		self.nav2_alive = True

		# If navigator_client sends "Navigation is accepted: <GOAL_COORDINATES_TUPLE>" indicates
		# it has received the goal and made bt_navigator execute navigation
		if msg.data.find('accepted') != -1:
			self.feedback.emit(f'Navigating to position {self._getGoalId(msg.data)}...', True)

			# Create a timer to check the status of navigation tasks
			if self.check_nav2_timer is None:
				timer_cbg = MutuallyExclusiveCallbackGroup()
				self.check_nav2_timer = self.node.create_timer(self.wait_response_timeout, self._checkNav2Alive, callback_group=timer_cbg)

		elif msg.data.find('cancelled') != -1:
			self.feedback.emit(f'Navigation task to position {self._getGoalId(msg.data)} is cancelled.', False)
			self._taskEnded()

		elif msg.data.find('finished') != -1:
			self.feedback.emit(f'Successfully reached position {self._getGoalId(msg.data)}.', False)
			self.update_goal_id.emit(self._getGoalId(msg.data))

			if self.num_goals == 1:
				self._taskEnded()
			else: # If self.num_goals > 1 indicates it is a patrol task
				self.next_goal += 1
				self.next_goal %= self.num_goals
				self.num_waiting_goals -= 1
				if self.num_waiting_goals % self.num_goals == 0:
					self.feedback.emit('Successfully finished a patrol!', False)
				if self.num_waiting_goals == 0:
					# Start another patrol task if it finished one
					self.assignTask(self.goals_to_assign)

		else:
			for kw in self.end_keywords:
				if msg.data.find(kw) != -1:
					self.feedback.emit(f'Navigation task to position {self.next_goal} is aborted.', False)
					self._taskEnded()
					break

	def run(self):
		self.executor = MultiThreadedExecutor()
		self.executor.add_node(self.node)
		try:
			self.executor.spin()
		except:
			pass

	def stop(self):
		self._taskEnded()
		self.node.destroy_node()
		self.executor.shutdown()

class PosUpdaterThread(QThread):
	req_signal = Signal(float, float)

	def __init__(self, map_ori_x, map_ori_y, phy_ori_x, phy_ori_y, map_resolution, map_orientation):
		super().__init__()
		self.map_ori_x = map_ori_x
		self.map_ori_y = map_ori_y
		self.phy_ori_x = phy_ori_x
		self.phy_ori_y = phy_ori_y
		self.map_resolution = map_resolution
		self.map_orientation = map_orientation

		self.node = Node(f'tf_map_amr_{os.getpid()}')
		self.node.tf_buffer = Buffer()
		self.node.tf_trans = TransformListener(self.node.tf_buffer, self.node)
		self.node.create_timer(0.4, self.reqTF)

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

	def reqTF(self):
		try:
			tf = self.node.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
			x, y = self.phyPosToMap(tf.transform.translation.x, tf.transform.translation.y)
			self.req_signal.emit(x, y)
		except:
			pass

	def run(self):
		self.executor = SingleThreadedExecutor()
		self.executor.add_node(self.node)
		try:
			self.executor.spin()
		except:
			pass

	def stop(self):
		self.node.destroy_node()
		self.executor.shutdown()

class CamUpdaterThread(QThread):
	req_signal = Signal(bool)

	def __init__(self, width, height, cam_frame_path):
		super().__init__()
		self.img_space = np.memmap(cam_frame_path, dtype=np.uint8, mode='w+', shape=(height * width * 3))

		self.node = Node(f'cam_viewer_{os.getpid()}')
		qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
		self.node.create_subscription(Image, '/image_raw', self.transToNumpyArr, qos_profile)

	def run(self):
		self.executor = SingleThreadedExecutor()
		self.executor.add_node(self.node)
		try:
			self.executor.spin()
		except:
			pass

	def transToNumpyArr(self, msg):
		self.img_space[:] = np.array(msg.data)
		self.req_signal.emit(True)

	def stop(self):
		self.node.destroy_node()
		self.executor.shutdown()

class PathUpdaterThread(QThread):
	req_signal = Signal(QPixmap)

	def __init__(self, map_ori_x, map_ori_y, phy_ori_x, phy_ori_y, map_img_w, map_img_h, map_resolution, map_orientation):
		super().__init__()
		self.map_ori_x = map_ori_x
		self.map_ori_y = map_ori_y
		self.phy_ori_x = phy_ori_x
		self.phy_ori_y = phy_ori_y
		self.map_img_w = map_img_w
		self.map_img_h = map_img_h
		self.map_img_row = self.map_img_w * 4
		self.map_resolution = map_resolution
		self.map_orientation = map_orientation
		self.has_task = False

		self.node = Node(f'path_viewer_{os.getpid()}')
		qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
		self.node.create_subscription(Path, '/plan', self.recvPath, qos_profile)

	@Slot(int, int, int, int, float)
	def editParam(self, map_ori_x, map_ori_y, map_img_w, map_img_h, map_resolution):
		self.map_ori_x = map_ori_x
		self.map_ori_y = map_ori_y
		self.map_img_w = map_img_w
		self.map_img_h = map_img_h
		self.map_img_row = self.map_img_w * 4
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

	def arrayToPixmap(self, arr):
		return QPixmap.fromImage(QImage(arr, self.map_img_w, self.map_img_h, self.map_img_row, QImage.Format_RGBA8888))

	def recvPath(self, msg):
		path_img = np.zeros((self.map_img_h, self.map_img_w, 4), dtype=np.uint8)
		if self.has_task:
			for pose_stamped in msg.poses:
				pose = pose_stamped.pose
				x, y = self.phyPosToMap(pose.position.x, pose.position.y)
				l = max(0, x-3)
				r = min(self.map_img_w, x+3)
				t = max(0, y-3)
				b = min(self.map_img_h, y+3)
				path_img[t:b+1, l:r+1, :] = [255, 128, 0, 200]
		self.req_signal.emit(self.arrayToPixmap(path_img))

	def run(self):
		self.executor = SingleThreadedExecutor()
		self.executor.add_node(self.node)
		try:
			self.executor.spin()
		except:
			pass

	def stop(self):
		self.node.destroy_node()
		self.executor.shutdown()

