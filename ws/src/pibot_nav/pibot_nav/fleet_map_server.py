import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped
from fleet_msgs.msg import BotPose
from math import inf
from time import time
import numpy as np

GET_POSE_TOLERANCE = 3.0
MAP_PUB_PERIOD = 0.2
CELL_OCCUPIED = 100

class FleetMapServer(Node):
	def __init__(self):
		super().__init__('fleet_map_server')

		main_cbg = MutuallyExclusiveCallbackGroup()
		self.map_msg = OccupancyGrid()
		self.map_msg.header.frame_id = 'map'
		self.map_data = []
		self.origin_point = None
		self.fleet_map_pub = None
		self.map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
		self.original_map_sub = self.create_subscription(OccupancyGrid, '/map', self.getMap, self.map_qos, callback_group=main_cbg)

		self.bot_name = self.get_namespace().split('/')[1]
		self.fleet_members = {}
		self.sub_footprint_cbg = MutuallyExclusiveCallbackGroup()
		self.fleet_members_sub = self.create_subscription(BotPose, '/fleet_pose', self.getFleetMembers, 10, callback_group=main_cbg)

	def getFleetMembers(self, msg):
		member = msg.bot_name
		if member != self.bot_name:
			if self.fleet_members.get(member) is None:
				self.fleet_members[member] = {
					'footprint': [],
					'last_time': time()
				}
				self.getFootprint(member, msg.footprint.points)
				self.get_logger().info(f'Register fleet member {member}')
			else:
				self.fleet_members[member]['last_time'] = time()
				self.getFootprint(member, msg.footprint.points)

	def getFootprint(self, member, points):
		if self.origin_point is not None:
			min_x, max_x, min_y, max_y = inf, 0, inf, 0
			for point in points:
				x = round(self.origin_point[0] + (point.x / self.map_msg.info.resolution))
				y = round(self.origin_point[1] - (point.y / self.map_msg.info.resolution))
				min_x = min(x, min_x)
				max_x = max(x, max_x)
				min_y = min(y, min_y)
				max_y = max(y, max_y)
			rect = np.mgrid[min_x:max_x+1, min_y:max_y+1].reshape(2, -1).transpose()
			self.fleet_members[member]['footprint'] = []
			for coor in rect:
				x, y = coor
				self.fleet_members[member]['footprint'].append((x, y))

	def getMap(self, msg):
		self.map_msg.info.resolution = msg.info.resolution
		self.map_msg.info.width = msg.info.width
		self.map_msg.info.height = msg.info.height
		self.origin_point = (-msg.info.origin.position.x / msg.info.resolution, self.map_msg.info.height + msg.info.origin.position.y / msg.info.resolution)
		self.map_msg.info.origin = msg.info.origin
		self.map_data = msg.data.tolist()
		if self.fleet_map_pub is None:
			pub_map_cbg = MutuallyExclusiveCallbackGroup()
			self.fleet_map_pub = self.create_publisher(OccupancyGrid, 'map', self.map_qos, callback_group=pub_map_cbg)
			self.fleet_map_timer = self.create_timer(MAP_PUB_PERIOD, self.pubMap, callback_group=pub_map_cbg)

	def pubMap(self):
		if self.map_data != []:
			self.map_msg.data = self.map_data.copy()

			current_time = time()
			members = list(self.fleet_members.keys())
			for member in members.copy():
				if current_time - self.fleet_members[member]['last_time'] > GET_POSE_TOLERANCE:
					self.get_logger().info(f'Unregister {member} from fleet member')
					_ = self.fleet_members.pop(member)
					members.remove(member)

			for member in members:
				for coor in self.fleet_members[member]['footprint']:
					x, y = coor
					idx = self.map_msg.info.width * (self.map_msg.info.height - y - 1) + x
					self.map_msg.data[idx] = CELL_OCCUPIED
			self.fleet_map_pub.publish(self.map_msg)

def main(args=None):
	rclpy.init()
	node = FleetMapServer()
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

