import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist, Point32
from nav2_msgs.msg import CollisionDetectorState
from rclpy.action.client import ActionClient
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from fleet_msgs.msg import TaskCommand, TaskStatus, BotPose, BotReport
from fleet_msgs.srv import TaskArrange

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
NUM_FOOTPRINT_POINTS = 4

class TaskClient(Node):
	def __init__(self):
		super().__init__('task_client')

		self.declare_parameter('keepout_radius', 1.5)
		keepout_radius = self.get_parameter('keepout_radius').get_parameter_value().double_value

		self.bot_name = self.get_namespace().split('/')[-1]

		self.twist_msg_val = (0.0, 0.0) # (linear.x, angular.z)
		self.executing_task = False
		self.task_paused = False
		self.collision_detected = False
		nav_cbg = MutuallyExclusiveCallbackGroup()
		self.nav_cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.passCtrlCmd, 10, callback_group=nav_cbg)

		collision_cbg = MutuallyExclusiveCallbackGroup()
		self.motor_cmd_pub = self.create_publisher(Twist, 'motor_vel', 10, callback_group=collision_cbg)
		self.cd_sub = self.create_subscription(CollisionDetectorState, 'collision_detector_state', self.confCDState, 10, callback_group=collision_cbg)

		task_cbg = MutuallyExclusiveCallbackGroup()

		qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
		self.goal_handle = None
		self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=task_cbg, feedback_sub_qos_profile=qos)
		self.nav_cancel_requester = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal', callback_group=task_cbg)

		transient_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
		self.task_goal = None
		self.task_getter = self.create_service(TaskArrange, f'/{self.bot_name}/fleet_task', self.getTask, callback_group=task_cbg)
		self.status_report = BotReport()
		self.status_report.bot_name = self.bot_name
		self.status_report.task_status.status = TaskStatus.TASK_SUCCEEDED
		self.status_report.keepout_radius = keepout_radius
		self.status_report_pub = self.create_publisher(BotReport, '/fleet_status', transient_qos, callback_group=task_cbg)

		tf_cbg = MutuallyExclusiveCallbackGroup()
		self.pose_msg = BotPose()
		self.pose_msg.bot_name = self.bot_name
		self.pose_pub = self.create_publisher(BotPose, '/fleet_pose', 1, callback_group=tf_cbg)
		self.tf_buffer = Buffer()
		tf_trans = TransformListener(self.tf_buffer, self)
		self.req_tf_timer = self.create_timer(REQUEST_TF_PERIOD, self.req_tf, callback_group=tf_cbg)

	def passCtrlCmd	(self, msg):
		if not self.collision_detected and not self.task_paused:
			if msg.linear.x != 0 or msg.angular.z != 0:
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
			self.twist_msg_val = (msg.linear.x, msg.angular.z)

	def confCDState(self, msg):
		if self.task_paused:
			self.twist_msg_val = (0.0, 0.0)
			self.sendCmd()

		elif self.executing_task:
			if self.goal_handle is None:
				self.twist_msg_val = (0.0, 0.0)
				self.executing_task = False

			else:
				cd = dict(zip(msg.polygons, msg.detections))
				if cd[CD_POLY['left_front']]:
					self.collision_detected = True
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
					self.twist_msg_val = (SPEED['forward'], 0.0)

				else:
					self.collision_detected = False
			self.sendCmd()

	def sendCmd(self):
		twist_msg = Twist()
		twist_msg.linear.x = self.twist_msg_val[0]
		twist_msg.angular.z = self.twist_msg_val[1]
		self.motor_cmd_pub.publish(twist_msg)

	def _publishStatus(self):
		self.status_report_pub.publish(self.status_report)

	def getTask(self, request, response):
		if self.task_goal is None and request.task_cmd.task_cmd == TaskCommand.CMD_EXECUTE:
			count = 0
			while not self.nav_client.wait_for_server(timeout_sec=1.5):
				count += 1
				if count > 5:
					self.status_report.task_status.status = TaskStatus.TASK_ABORTED
					self._publishStatus()
					return

			self.status_report.task_status.status = TaskStatus.TASK_EXECUTING
			self.task_goal = request.goal

			nav_req = NavigateToPose.Goal()
			nav_req.pose.header.frame_id = 'map'
			nav_req.pose.pose = request.goal
			send_future = self.nav_client.send_goal_async(nav_req, self._feedbackCallback)
			send_future.add_done_callback(self._sendGoalResult)
			self.get_logger().info(f'Navigating to {request.goal.position.x, request.goal.position.y}')
		else:
			if request.task_cmd.task_cmd == TaskCommand.CMD_EXECUTE:
				self.task_paused = False
			elif request.task_cmd.task_cmd == TaskCommand.CMD_PAUSE:
				self.task_paused = True
			elif request.task_cmd.task_cmd == TaskCommand.CMD_CANCEL:
				if self.goal_handle is not None:
					count = 0
					while not self.nav_cancel_requester.wait_for_service(timeout_sec=1.5):
						count += 1
						if count >= 5:
							return
					_ = self.nav_cancel_requester.call_async(CancelGoal.Request())
					self.get_logger().info(f'Cancelling the task to {self.task_goal.position.x, self.task_goal.position.y}')

		response.accept = True
		return response

	def _feedbackCallback(self, msg):
		self.status_report.distance_remaining = msg.feedback.distance_remaining
		self._publishStatus()

	def _sendGoalResult(self, future):
		request_permission = future.result()
		if request_permission.accepted:
			self.status_report.task_status.status = TaskStatus.TASK_EXECUTING
			self._publishStatus()
			self.goal_handle = request_permission.get_result_async()
			self.goal_handle.add_done_callback(self._finishTask)
			self.executing_task = True
		else:
			self.status_report.task_status.status = TaskStatus.TASK_ABORTED
			self._publishStatus()

	def _finishTask(self, future):
		if future.result().status == GoalStatus.STATUS_SUCCEEDED:
			self.status_report.task_status.status = TaskStatus.TASK_SUCCEEDED
			self.get_logger().info(f'Task succeeded')
		elif future.result().status == GoalStatus.STATUS_CANCELED:
			self.status_report.task_status.status = TaskStatus.TASK_CANCELLED
			self.get_logger().info(f'Task cancelled')
		elif future.result().status == GoalStatus.STATUS_ABORTED or future.result().status == GoalStatus.STATUS_UNKNOWN:
			self.status_report.task_status.status = TaskStatus.TASK_ABORTED
			self.get_logger().info(f'Task aborted')
		self._publishStatus()
		self.task_goal = None
		self.goal_handle = None

	def req_tf(self):
		try:
			req_time = rclpy.time.Time()
			tf = self.tf_buffer.lookup_transform('map', 'base_footprint', req_time)
			prev_x = self.pose_msg.current_pose.position.x
			prev_y = self.pose_msg.current_pose.position.y

			self.pose_msg.current_pose.position.x = tf.transform.translation.x
			self.pose_msg.current_pose.position.y = tf.transform.translation.y
			self.pose_msg.current_pose.orientation.z = tf.transform.rotation.z
			self.pose_msg.current_pose.orientation.w = tf.transform.rotation.w

			self.pose_msg.footprint.points.clear()

			for idx in range(NUM_FOOTPRINT_POINTS):
				tf = self.tf_buffer.lookup_transform('map', f'footprint_{idx}', req_time)
				self.pose_msg.footprint.points.append(Point32(x=tf.transform.translation.x, y=tf.transform.translation.y))

			self.pose_pub.publish(self.pose_msg)
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

