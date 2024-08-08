import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.action.client import ActionClient
from action_msgs.msg import GoalStatusArray, GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from task_msgs.srv import TaskRequest, CancelRequest

CHECK_NAV2_PERIOD = 8.0

class GoalSender(Node):
	def __init__(self):
		super().__init__('navigation_requester')
		nav2_cbg = MutuallyExclusiveCallbackGroup()
		qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
		self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=nav2_cbg, feedback_sub_qos_profile=qos)
		self.nav_cancel_requester = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal', callback_group=nav2_cbg)

		self.nav2_alive = False # Whether receive feedback from bt_navigator
		self.prev_goal_msg = None # Prevent sending repeated goals
		self.goals = [] # Store goal coordinates with tuples
		self.preempt_goals = [] # Store the latest received task if it comes during the execution of the previous one
		self.cur_goal = None
		self.goal_handle = None
		self.end_succ = True # Determine the task is cancelled or succeeded

		cmd_cbg = MutuallyExclusiveCallbackGroup()
		self.cmd_listener = self.create_service(TaskRequest, 'nav_goals', self.arrangeTasks, callback_group=cmd_cbg)
		self.cancel_listener = self.create_service(CancelRequest, 'nav_cancel', self.cancelTasks, callback_group=cmd_cbg)

		self.feedback = String()
		self.nav_feedback_reporter = self.create_publisher(String, 'nav_feedback', 1, callback_group=cmd_cbg)

		wait_cbg = MutuallyExclusiveCallbackGroup()
		self.wait_clear_timer = self.create_timer(1.0, self._assignGoals, callback_group=wait_cbg)
		timer_cbg = MutuallyExclusiveCallbackGroup()
		self.check_nav2_timer = self.create_timer(CHECK_NAV2_PERIOD, self._checkNav2Callback, callback_group=timer_cbg)

	def arrangeTasks(self, req, res):
		self.preempt_goals = []
		for idx in range(0, len(req.goals), 2):
			goal = PoseStamped()
			goal.header.frame_id = 'map'
			goal.pose.position.x = req.goals[idx]
			goal.pose.position.y = req.goals[idx+1]
			self.preempt_goals.append(goal)

		self._sendCancelRequest()
		res.accept = True
		return res

	def cancelTasks(self, req, res):
		if req.cancel:
			self._sendCancelRequest()
		res.accept = True
		return res

	def _sendCancelRequest(self):
		if self.goal_handle is not None:
			count = 0
			while not self.nav_cancel_requester.wait_for_service(timeout_sec=1.5):
				count += 1
				if count >= 5:
					self._publishFeedback('Navigation cancel server has no response, please try again later')
					return
			self._publishFeedback('Sending cancel request...')
			self.goals = []
			_ = self.nav_cancel_requester.call(CancelGoal.Request())

	def _assignGoals(self):
		if self.goal_handle is None and self.preempt_goals:
			self.goals = self.preempt_goals.copy()
			self.preempt_goals = []

			count = 0
			while not self.nav_client.wait_for_server(timeout_sec=1.5):
				count += 1
				if count > 5:
					self._publishFeedback('Navigation server has no response, please try again later')
					return
			self._publishFeedback('Navigation server is ready')

			log = ''
			for goal in self.goals:
				if log != '':
					log += ', '
				log += f'({goal.pose.position.x}, {goal.pose.position.y})'
			self._publishFeedback(f'Sending goals: [{log}]')
			self._sendGoal()

	def _sendGoal(self):
		if len(self.goals) > 0:
			nav_req = NavigateToPose.Goal()
			nav_req.pose = self.goals.pop(0)
			if len(self.goals) == 0:
				self.prev_goal_msg = None
			self.cur_goal = (nav_req.pose.pose.position.x, nav_req.pose.pose.position.y)
			self.end_succ = True

			send_future = self.nav_client.send_goal_async(nav_req, self._feedbackCallback)
			send_future.add_done_callback(self._sendGoalResult)
			self.nav2_alive = True

	def _sendGoalResult(self, future):
		request_permission = future.result()
		if request_permission.accepted:
			self._publishFeedback(f'Navigation task is accepted: {self.cur_goal}')
			self.nav2_alive = True
			self.goal_handle = request_permission.get_result_async()
			self.goal_handle.add_done_callback(self._finishTask)
		else:
			self._publishFeedback('Navigation task is rejected')

	def _finishTask(self, future):
		if future.result().status == GoalStatus.STATUS_SUCCEEDED:
			self._publishFeedback(f'Successfully finished navigation task: {self.cur_goal}')
		elif future.result().status == GoalStatus.STATUS_CANCELED:
			self._publishFeedback(f'Navigation task to {self.cur_goal} is cancelled')
		elif future.result().status == GoalStatus.STATUS_ABORTED or future.result().status == GoalStatus.STATUS_UNKNOWN:
			self._publishFeedback(f'Navigation task to {self.cur_goal} is aborted')
		self.goal_handle = None
		self.prev_goal_msg = None
		self.cur_goal = None
		if self.goals:
			self._sendGoal()

	def _feedbackCallback(self, msg):
		self.nav2_alive = True
		if msg.feedback.number_of_recoveries > 13:
			self._publishFeedback('Navigation task is aborting')
			self._sendCancelRequest()
			return
#		cur_x = msg.feedback.current_pose.pose.position.x
#		cur_y = msg.feedback.current_pose.pose.position.y
		self._publishFeedback(f'Estimated time remaining: {msg.feedback.estimated_time_remaining.sec}.{msg.feedback.estimated_time_remaining.nanosec} s; Recoveries: {msg.feedback.number_of_recoveries}; Current goal: {self.cur_goal}')

	def _checkNav2Callback(self):
		if self.goal_handle is not None:
			if not self.nav2_alive:
				self._publishFeedback('Navigation server has no response, the task is going to abort')
				self._sendCancelRequest()
			self.nav2_alive = False

	def _publishFeedback(self, log):
		self.feedback.data = log
		self.nav_feedback_reporter.publish(self.feedback)
		self.get_logger().info(self.feedback.data)

def main(args=None):
	rclpy.init()
	node = GoalSender()
	executor = MultiThreadedExecutor()
	try:
		executor.add_node(node)
		executor.spin()
	except KeyboardInterrupt:
		pass
	except:
		node.destroy_node()
		executor.shutdown()

if __name__ == '__main__':
	main()

