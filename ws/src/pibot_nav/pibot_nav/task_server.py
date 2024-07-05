import rclpy
from rclpy.node import Node
from rclpy.clock import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from fleet_msgs.msg import TaskArrange, TaskStatus, BotPose, BotReport, TaskResponse
from fleet_msgs.srv import TaskRequest
from math import sqrt, inf
from time import time

FLEET_CHECK_TIMER = 0.2
GET_POSE_TOLERANCE = 3.0

class TaskServer(Node):
	def __init__(self):
		super().__init__('task_server')
		self.fleet_members = {}
		member_cbg = MutuallyExclusiveCallbackGroup()
		transient_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

		task_ui_cbg = MutuallyExclusiveCallbackGroup()
		self.task_pool = []
		self.srv_server = self.create_service(TaskRequest, '/task_request', self.getTask, callback_group=task_ui_cbg)

		bot_pose_cbg = MutuallyExclusiveCallbackGroup()
		self.bot_pose_sub = self.create_subscription(BotPose, '/fleet_pose', self.getPose, 10, callback_group=bot_pose_cbg)

		bot_status_cbg = MutuallyExclusiveCallbackGroup()
		self.task_status_sub = self.create_subscription(BotReport, '/fleet_status', self.getStatus, transient_qos, callback_group=bot_status_cbg)
		self.task_response_pub = self.create_publisher(TaskResponse, '/task_response', transient_qos, callback_group=bot_status_cbg)

		task_arrange_cbg = MutuallyExclusiveCallbackGroup()
		self.task_arrange_pub = self.create_publisher(TaskArrange, '/fleet_task', transient_qos, callback_group=task_arrange_cbg)
		self.fleet_check_timer = self.create_timer(FLEET_CHECK_TIMER, self.fleetCheck, callback_group=task_arrange_cbg)

	def getTask(self, request, response):
		if request.cancel:
			# Cancel an executing task
			if self.fleet_members.get(request.bot_name) is not None and self.fleet_members[request.bot_name]['task'] is not None and self.fleet_members[request.bot_name]['task'][0] == request.task_uuid:
				self.fleet_members[request.bot_name]['task'][1].task_cmd = TaskArrange.CMD_CANCEL
				self._publishTask(self.fleet_members[request.bot_name]['task'][1])
				self.get_logger().info(f'Cancelling the task of {request.bot_name}')
			# Cancel a task that is still waiting in self.task_pool
			else:
				for task in self.task_pool:
					if task[0] == request.task_uuid:
						self.task_pool.remove(task)
						self._publishResponse(task[0], request.bot_name, TaskStatus.TASK_CANCELLED)
						break
		else:
			new_task = TaskArrange()
			new_task.task_cmd = TaskArrange.CMD_EXECUTE
			new_task.goal = request.goal

			assigned_bot = None
			request_bot_not_exist = request.bot_name not in self.fleet_members.keys()
			if request_bot_not_exist:
				min_dist = inf
				x1, y1 = new_task.goal.position.x, new_task.goal.position.y
				for bot in self.fleet_members.keys():
					if self.fleet_members[bot]['task'] is None:
						x2, y2 = self.fleet_members[bot]['pos']
						dist = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
						if dist < min_dist:
							min_dist = dist
							assigned_bot = bot
			elif self.fleet_members[request.bot_name]['task'] is None:
				assigned_bot = request.bot_name

			if assigned_bot is None:
				new_task.bot_name = request.bot_name
				self.task_pool.append((request.task_uuid, new_task))
				self.get_logger().info(f'Unable to send {"an AMR" if request_bot_not_exist else request.bot_name} to {new_task.goal.position.x, new_task.goal.position.y} now, the task will start when it is available')
			else:
				occupied_bot = self.taskGoalOccupancy(assigned_bot, new_task.goal.position)
				if occupied_bot is None:
					new_task.bot_name = assigned_bot
					self.fleet_members[assigned_bot]['task'] = (request.task_uuid, new_task)
					self._publishTask(self.fleet_members[assigned_bot]['task'][1])
					self._publishResponse(self.fleet_members[assigned_bot]['task'][0], assigned_bot, TaskStatus.TASK_EXECUTING)
					self.get_logger().info(f'Arrange {assigned_bot} to {new_task.goal.position.x, new_task.goal.position.y}')
				else:
					self._publishResponse(request.task_uuid, request.bot_name, TaskStatus.TASK_CANCELLED)
					self.get_logger().info(f'Task to {new_task.goal.position.x, new_task.goal.position.y} is cancelled as {occupied_bot} is (going) there')

				

		response.accept = True
		return response

	def getPose(self, msg):
		bot = msg.bot_name
		if self.fleet_members.get(bot) is None:
			self.fleet_members[bot] = {
				'keepout_radius': 1.5,
				'pos': (msg.current_pose.position.x, msg.current_pose.position.y),
				'last_time': time(),
				'task': None,
				'distance_remaining': 0.0
			}
			self.get_logger().info(f'Register fleet member {bot}')

			self.searchTask(bot)
		else:
			self.fleet_members[bot]['pos'] = (msg.current_pose.position.x, msg.current_pose.position.y)
			self.fleet_members[bot]['last_time'] = time()

	def getStatus(self, msg):
		bot = msg.bot_name
		if self.fleet_members.get(bot) is not None:
			self.fleet_members[bot]['keepout_radius'] = msg.keepout_radius
			if msg.task_status.status == TaskStatus.TASK_EXECUTING:
				self.fleet_members[bot]['distance_remaining'] = msg.distance_remaining
			elif self.fleet_members[bot]['task'] is not None:
					self._publishResponse(self.fleet_members[bot]['task'][0], bot, msg.task_status.status)

					self.fleet_members[bot]['task'][1].task_cmd = TaskArrange.CMD_FINISH
					self._publishTask(self.fleet_members[bot]['task'][1])
					self.fleet_members[bot]['task'] = None

			if self.fleet_members[bot]['task'] is None:
				self.searchTask(bot)

	def searchTask(self, bot):
		for task in self.task_pool:
			if self.fleet_members.get(task[1].bot_name) is None or task[1].bot_name == bot:
				self.task_pool.remove(task)

				occupied_bot = self.taskGoalOccupancy(bot, task[1].goal.position)
				if occupied_bot is None:
					task[1].bot_name = bot
					self.fleet_members[bot]['task'] = task
					self._publishTask(self.fleet_members[bot]['task'][1])
					self._publishResponse(self.fleet_members[bot]['task'][0], bot, TaskStatus.TASK_EXECUTING)
					self.get_logger().info(f'Arrange {bot} to {task[1].goal.position.x, task[1].goal.position.y}')
					break
				else:
					self._publishResponse(task[0], task[1].bot_name, TaskStatus.TASK_CANCELLED)
					self.get_logger().info(f'Task to {task[1].goal.position.x, task[1].goal.position.y} as {occupied_bot} is (going) there')

	def taskGoalOccupancy(self, bot, goal_position):
		x1, y1 = goal_position.x, goal_position.y
		for other_bot in self.fleet_members.keys():
			if bot != other_bot:
				if self.fleet_members[other_bot]['task'] is None:
					x2, y2 = self.fleet_members[other_bot]['pos']
				else:
					x2, y2 = self.fleet_members[other_bot]['task'][1].goal.position.x, self.fleet_members[other_bot]['task'][1].goal.position.y
				dist = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
				if dist < self.fleet_members[bot]['keepout_radius']:
					return other_bot
		return None

	def _publishTask(self, msg):
		self.task_arrange_pub.publish(msg)

	def _publishResponse(self, task_uuid, bot_name, status):
		res_msg = TaskResponse()
		res_msg.task_uuid = task_uuid
		res_msg.bot_name = bot_name
		res_msg.task_status.status = status
		self.task_response_pub.publish(res_msg)

	def fleetCheck(self):
		current_time = time()
		bots = list(self.fleet_members.keys())
		for bot in bots:
			if current_time - self.fleet_members[bot]['last_time'] > GET_POSE_TOLERANCE:
				self.get_logger().info(f'Unregister {bot} from fleet member')
				_ = self.fleet_members.pop(bot)
				bots.remove(bot)
		
		clear_to_go = dict.fromkeys(bots, True)
		for idx_main in range(len(bots)):
			bot = bots[idx_main]
			if self.fleet_members[bot]['task'] is not None:
				for idx_other in range(idx_main+1, len(bots)):
					other_bot = bots[idx_other]
					if self.fleet_members[other_bot]['task'] is not None:
						# 1. Check if the two moving AMR have a distance of max(keepout_radius)
						x1, y1 = self.fleet_members[bot]['pos']
						x2, y2 = self.fleet_members[other_bot]['pos']
						dist = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
						safe_dist = max(self.fleet_members[bot]['keepout_radius'], self.fleet_members[other_bot]['keepout_radius'])
						if dist <= safe_dist:
							# 2. Make the AMR that has shorter distance to the goal go first
							if self.fleet_members[bot]['distance_remaining'] > self.fleet_members[other_bot]['distance_remaining']:
								clear_to_go[bot] = False
							else:
								clear_to_go[other_bot] = False

		for bot in bots:
			if self.fleet_members[bot]['task'] is not None:
				if not clear_to_go[bot] and self.fleet_members[bot]['task'][1].task_cmd == TaskArrange.CMD_EXECUTE:
					self.fleet_members[bot]['task'][1].task_cmd = TaskArrange.CMD_PAUSE
					self.task_arrange_pub.publish(self.fleet_members[bot]['task'][1])
					self.get_logger().info(f'Pause {bot}')
				elif clear_to_go[bot] and self.fleet_members[bot]['task'][1].task_cmd == TaskArrange.CMD_PAUSE:
					self.fleet_members[bot]['task'][1].task_cmd = TaskArrange.CMD_EXECUTE
					self.task_arrange_pub.publish(self.fleet_members[bot]['task'][1])
					self.get_logger().info(f'Resume {bot}')

def main(args=None):
	rclpy.init()
	node = TaskServer()
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
#		executor.shutdown()

if __name__ == '__main__':
	main()

