import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionDetectorState
from rclpy.action.client import ActionClient
from action_msgs.msg import GoalStatusArray, GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Vector3
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from fleet_msgs.msg import TaskArrange, BotStatus
from time import time, sleep

CD_POLY = {
	'left_front': 'PolygonLeftFront',
	'right_front': 'PolygonRightFront',
	'left_back': 'PolygonLeftBack',
	'right_back': 'PolygonRightBack'
}

LINEAR_THRES = 0.1
ANGULAR_THRES = 0.7

SPEED = {
	'forward': LINEAR_THRES,
	'backward': -LINEAR_THRES,
	'turn_left': ANGULAR_THRES,
	'turn_right': -ANGULAR_THRES
}

REQUEST_TF_PERIOD = 0.2

class TaskClient(Node):
	def __init__(self):
		super().__init__('task_client')

		self.declare_parameter('keepout_radius', 1.5)
		keepout_radius = self.get_parameter('keepout_radius').get_parameter_value().double_value

		self.bot_name = self.get_namespace().split('/')[-1]

		self.recv_cmd_time = time()
		self.twist_msg_val = (0.0, 0.0) # (linear.x, angular.z)
		self.is_stop = True
		self.collision_detected = False
		self.task_paused = False
		nav_cbg = MutuallyExclusiveCallbackGroup()
		self.nav_cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.passCtrlCmd, 10, callback_group=nav_cbg)

		collision_cbg = MutuallyExclusiveCallbackGroup()
		self.motor_cmd_pub = self.create_publisher(Twist, 'motor_vel', 10, callback_group=collision_cbg)
		self.cd_sub = self.create_subscription(CollisionDetectorState, 'collision_detector_state', self.confCDState, 10, callback_group=collision_cbg)

		nav_client_cbg = MutuallyExclusiveCallbackGroup()
		qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
		self.goal_handle = None
		self.nav_end_type = BotStatus.STATUS_SUCCEEDED
		self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=nav_client_cbg, feedback_sub_qos_profile=qos)
		self.nav_cancel_requester = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal', callback_group=nav_client_cbg)
		self.bt_navigator_status = self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status', self.assistGetStatus, 1, callback_group=nav_client_cbg)

		task_cbg = MutuallyExclusiveCallbackGroup()
		transient_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
		self.task = None
		self.task_sub = self.create_subscription(TaskArrange, '/fleet_task', self.getTask, transient_qos, callback_group=task_cbg)
		self.status_report = BotStatus()
		self.status_report.bot_name = self.bot_name
		self.status_report.keepout_radius = keepout_radius
		self.status_report_pub = self.create_publisher(BotStatus, '/fleet_status', transient_qos, callback_group=task_cbg)

		tf_cbg = MutuallyExclusiveCallbackGroup()
		self.announced = False
		self.tf_buffer = Buffer()
		tf_trans = TransformListener(self.tf_buffer, self)
		self.req_tf_timer = self.create_timer(REQUEST_TF_PERIOD, self.req_tf, callback_group=tf_cbg)

	def passCtrlCmd	(self, msg):
		self.recv_cmd_time = time()
		if not self.collision_detected and not self.task_paused:
			if msg.linear.x != 0 or msg.angular.z != 0:
				self.is_stop = False

				vel_is_pos = (msg.linear.x >= 0)
				if vel_is_pos:
					vel_insuff = (LINEAR_THRES - msg.linear.x) / LINEAR_THRES
				else:
					vel_insuff = (LINEAR_THRES + msg.linear.x) / LINEAR_THRES

				theta_is_pos = (msg.angular.z >= 0)
				if theta_is_pos:
					theta_insuff = (ANGULAR_THRES - msg.angular.z) / ANGULAR_THRES
				else:
					theta_insuff = (ANGULAR_THRES + msg.angular.z) / ANGULAR_THRES

				if vel_insuff > 0 and theta_insuff > 0:
					if vel_insuff < theta_insuff:
						msg.linear.x = LINEAR_THRES if vel_is_pos else -LINEAR_THRES
						msg.angular.z = 0.0
					else:
						msg.linear.x = 0.0
						msg.angular.z = ANGULAR_THRES if theta_is_pos else -ANGULAR_THRES
			else:
				self.is_stop = True
			self.twist_msg_val = (msg.linear.x, msg.angular.z)

	def confCDState(self, msg):
		if self.task_paused:
			self.twist_msg_val = (0.0, 0.0)
			self.sendCmd()
			self.is_stop = True

		elif time() - self.recv_cmd_time <= 0.5:
			cd = dict(zip(msg.polygons, msg.detections))
			if cd[CD_POLY['left_front']]:
				self.collision_detected = True
				self.is_stop = False
				if cd[CD_POLY['left_back']] and cd[CD_POLY['right_back']]:
					self.twist_msg_val = (0.0, 0.0)
				elif cd[CD_POLY['left_back']]:
					self.twist_msg_val = (SPEED['backward'], SPEED['turn_left'] / 2)
				elif cd[CD_POLY['right_back']]:
					self.twist_msg_val = (0.0, SPEED['turn_right'])
				else:
					self.twist_msg_val = (SPEED['backward'], SPEED['turn_right'] / 2)

			elif cd[CD_POLY['right_front']]:
				self.collision_detected = True
				self.is_stop = False
				if cd[CD_POLY['left_back']] and cd[CD_POLY['right_back']]:
					self.twist_msg_val = (0.0, 0.0)
				elif cd[CD_POLY['right_back']]:
					self.twist_msg_val = (SPEED['backward'], SPEED['turn_right'] / 2)
				elif cd[CD_POLY['left_back']]:
					self.twist_msg_val = (0.0, SPEED['turn_left'])
				else:
					self.twist_msg_val = (SPEED['backward'], SPEED['turn_left'] / 2)

			elif cd[CD_POLY['left_back']] or cd[CD_POLY['right_back']]:
				self.collision_detected = True
				self.is_stop = False
				self.twist_msg_val = (SPEED['forward'], 0.0)

			else:
				self.collision_detected = False
			self.sendCmd()

		elif not self.is_stop:
			self.twist_msg_val = (0.0, 0.0)
			self.sendCmd()
			self.is_stop = True

	def sendCmd(self):
		twist_msg = Twist()
		twist_msg.linear.x = self.twist_msg_val[0]
		twist_msg.angular.z = self.twist_msg_val[1]
		self.motor_cmd_pub.publish(twist_msg)

	def assistGetStatus(self, msg):
		if msg.status_list[-1].status == GoalStatus.STATUS_CANCELING:
			self.nav_end_type = BotStatus.STATUS_CANCELLED
		elif msg.status_list[-1].status == GoalStatus.STATUS_ABORTED:
			self.nav_end_type = BotStatus.STATUS_REJECTED

	def _publishStatus(self):
		self.status_report_pub.publish(self.status_report)

	def getTask(self, msg):
		if msg.bot_name == self.bot_name and msg.task_cmd != TaskArrange.TASK_FINISH:
			if self.task is None and msg.task_cmd == TaskArrange.TASK_EXEC:
				count = 0
				while not self.nav_client.wait_for_server(timeout_sec=1.5):
					count += 1
					if count > 5:
						self.status_report.status = BotStatus.STATUS_REJECTED
						self._publishStatus()
						return

				self.task = msg
				self.nav_end_type = BotStatus.STATUS_SUCCEEDED

				nav_req = NavigateToPose.Goal()
				nav_req.pose.header.frame_id = 'map'
				nav_req.pose.pose = msg.goal
				send_future = self.nav_client.send_goal_async(nav_req, self._feedbackCallback)
				send_future.add_done_callback(self._sendGoalResult)
				self.get_logger().info(f'Navigating to {msg.goal.position.x, msg.goal.position.y}')
			else:
				if msg.task_cmd == TaskArrange.TASK_EXEC:
					self.task_paused = False
				elif msg.task_cmd == TaskArrange.TASK_PAUSE:
					self.task_paused = True
				elif msg.task_cmd == TaskArrange.TASK_CANCEL:
					if self.goal_handle is not None:
						count = 0
						while not self.nav_cancel_requester.wait_for_service(timeout_sec=1.5):
							count += 1
							if count >= 5:
								return
						_ = self.nav_cancel_requester.call(CancelGoal.Request())
						self.get_logger().info(f'Cancelling the task to {self.task.goal.position.x, self.task.goal.position.y}')

	def _feedbackCallback(self, msg):
		self.status_report.distance_remain = msg.feedback.distance_remaining
		self._publishStatus()

	def _sendGoalResult(self, future):
		request_permission = future.result()
		if request_permission.accepted:
			self.status_report.status = BotStatus.STATUS_EXECUTING
			self._publishStatus()
			self.goal_handle = request_permission.get_result_async()
			self.goal_handle.add_done_callback(self._finishTask)
		else:
			self.status_report.status = BotStatus.STATUS_REJECTED
			self._publishStatus()

	def _finishTask(self, future):
		self.status_report.status = self.nav_end_type
		self._publishStatus()
		self.task = None
		self.goal_handle = None
		self.get_logger().info(f'Task ended')

	def req_tf(self):
		try:
			tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
			self.status_report.current_pos.x = tf.transform.translation.x
			self.status_report.current_pos.y = tf.transform.translation.y
			if not self.announced:
				self.status_report.status = BotStatus.STATUS_ACTIVE
				self.announced = True
				self._publishStatus()
		except:
			pass

def main(args=None):
	rclpy.init()
	node = TaskClient()
	executor = MultiThreadedExecutor()
	executor.add_node(node)
	executor.spin()
#	try:
#		executor.add_node(node)
#		executor.spin()
#	except KeyboardInterrupt:
#		pass
#	except:
#		node.destroy_node()
#		rclpy.shutdown()

if __name__ == '__main__':
	main()

