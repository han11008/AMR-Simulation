import rclpy
from rclpy.node import Node
from rclpy.clock import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from fleet_msgs.msg import TaskArrange, BotStatus
from math import sqrt, inf

FLEET_CHECK_TIMER = 0.2
NO_RESPONSE_COUNT_TOLERANCE = 8

class TaskServer(Node):
	def __init__(self):
		super().__init__('task_server')
		self.fleet_members = {}
		member_cbg = MutuallyExclusiveCallbackGroup()
		transient_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

		task_sub_cbg = MutuallyExclusiveCallbackGroup()
		self.task_pool = []
		self.task_sub = self.create_subscription(TaskArrange, '/task', self.getTask, 5, callback_group=task_sub_cbg)

		bot_status_cbg = MutuallyExclusiveCallbackGroup()
		self.task_status_sub = self.create_subscription(BotStatus, '/fleet_status', self.getStatus, transient_qos, callback_group=bot_status_cbg)

		task_arrange_cbg = MutuallyExclusiveCallbackGroup()
		self.task_arrange_pub = self.create_publisher(TaskArrange, '/fleet_task', transient_qos, callback_group=task_arrange_cbg)
		self.fleet_check_timer = self.create_timer(FLEET_CHECK_TIMER, self.fleetCheck, callback_group=task_arrange_cbg)

	def getTask(self, msg):
		if msg.task_cmd == TaskArrange.TASK_CANCEL:
			if self.fleet_members.get(msg.bot_name) is not None:
				self.fleet_members[msg.bot_name]['task'] = msg
				self._publishTask(self.fleet_members[msg.bot_name]['task'])
				self.get_logger().info(f'Cancelling the task of {msg.bot_name}')
		elif msg.task_cmd == TaskArrange.TASK_EXEC:
			assigned_bot = None
			if msg.bot_name not in self.fleet_members.keys():
				min_dist = inf
				x1, y1 = msg.goal.position.x, msg.goal.position.y
				for bot in self.fleet_members.keys():
					if self.fleet_members[bot]['task'] is None:
						x2, y2 = self.fleet_members[bot]['pos']
						dist = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
						if dist < min_dist:
							min_dist = dist
							assigned_bot = bot
			elif self.fleet_members[msg.bot_name]['task'] is None:
				assigned_bot = msg.bot_name

			if assigned_bot is None:
				self.task_pool.append(msg)
			else:
				msg.bot_name = assigned_bot
				self.fleet_members[assigned_bot]['task'] = msg
				self._publishTask(self.fleet_members[assigned_bot]['task'])
				self.get_logger().info(f'Arrange {assigned_bot} to {msg.goal.position.x, msg.goal.position.y}')

	def getStatus(self, msg):
		if self.fleet_members.get(msg.bot_name) is None:
			if msg.status != BotStatus.STATUS_INACTIVE:
				self.fleet_members[msg.bot_name] = {
					'keepout_radius': msg.keepout_radius,
					'pos': (msg.current_pos.x, msg.current_pos.y),
					'no_response_count': 0,
					'task': None,
					'distance_remain': 0.0
				}
				self.get_logger().info(f'Register fleet member {msg.bot_name}')
		else:
			if msg.status == BotStatus.STATUS_INACTIVE:
				del self.fleet_members[msg.bot_name]
			else:
				self.fleet_members[msg.bot_name]['pos'] = (msg.current_pos.x, msg.current_pos.y)
				if msg.status == BotStatus.STATUS_EXECUTING:
					self.fleet_members[msg.bot_name]['distance_remain'] = msg.distance_remain
				elif msg.status in [BotStatus.STATUS_SUCCEEDED, BotStatus.STATUS_CANCELLED, BotStatus.STATUS_REJECTED]:
					if self.fleet_members[msg.bot_name]['task'] is not None:
						self.fleet_members[msg.bot_name]['task'].task_cmd = TaskArrange.TASK_FINISH
						self._publishTask(self.fleet_members[msg.bot_name]['task'])
						self.fleet_members[msg.bot_name]['task'] = None

					for task in self.task_pool:
						if self.fleet_members.get(task.bot_name) is None or task.bot_name == msg.bot_name:
							self.task_pool.remove(task)
							task.bot_name = msg.bot_name
							self.fleet_members[msg.bot_name]['task'] = task
							self._publishTask(self.fleet_members[msg.bot_name]['task'])
							break

	def _publishTask(self, msg):
		self.task_arrange_pub.publish(msg)

	def fleetCheck(self):
		bots = list(self.fleet_members.keys())
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
							if self.fleet_members[bot]['distance_remain'] > self.fleet_members[other_bot]['distance_remain']:
								clear_to_go[bot] = False
							else:
								clear_to_go[other_bot] = False

		for bot in bots:
			if self.fleet_members[bot]['task'] is not None:
				if not clear_to_go[bot] and self.fleet_members[bot]['task'].task_cmd == TaskArrange.TASK_EXEC:
					self.fleet_members[bot]['task'].task_cmd = TaskArrange.TASK_PAUSE
					self.task_arrange_pub.publish(self.fleet_members[bot]['task'])
					self.get_logger().info(f'Pause {bot}')
				elif clear_to_go[bot] and self.fleet_members[bot]['task'].task_cmd == TaskArrange.TASK_PAUSE:
					self.fleet_members[bot]['task'].task_cmd = TaskArrange.TASK_EXEC
					self.task_arrange_pub.publish(self.fleet_members[bot]['task'])
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

