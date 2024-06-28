import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped
from std_msgs.msg import String
from math import inf
import numpy as np

SELF_DECLARATION_PERIOD = 1.0
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
		self.fleet_members_footprint = {}
		self.sub_footprint_cbg = MutuallyExclusiveCallbackGroup()
		self.fleet_members_qos = QoSProfile(depth=5, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
		self.fleet_members_sub = self.create_subscription(String, '/fleet_members', self.getFleetMembers, self.fleet_members_qos, callback_group=main_cbg)

		self.self_declaration_msg = String()
		self.self_declaration_msg.data = self.bot_name
		self.self_declaration_pub = self.create_publisher(String, '/fleet_members', self.fleet_members_qos, callback_group=main_cbg)
		self.self_declaration_pub.publish(self.self_declaration_msg)

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
			for member, footprint_dict in self.fleet_members_footprint.items():
				for coor in footprint_dict['footprint']:
					x, y = coor
					idx = self.map_msg.info.width * (self.map_msg.info.height - y - 1) + x
					self.map_msg.data[idx] = CELL_OCCUPIED
			self.fleet_map_pub.publish(self.map_msg)

	def getFleetMembers(self, msg):
		member_name = msg.data
		if member_name != self.bot_name and member_name not in self.fleet_members_footprint.keys():
			self.fleet_members_footprint[member_name] = {
				'footprint': [],
				'sub': self.create_subscription(PolygonStamped, f'/{member_name}/global_costmap/published_footprint',
					lambda footprint_msg: self.getFootprint(footprint_msg, member_name), 1, callback_group=self.sub_footprint_cbg)
			}

	def getFootprint(self, msg, member):
		if self.origin_point is not None:
			min_x, max_x, min_y, max_y = inf, 0, inf, 0
			for point in msg.polygon.points:
				x = round(self.origin_point[0] + (point.x / self.map_msg.info.resolution))
				y = round(self.origin_point[1] - (point.y / self.map_msg.info.resolution))
				if x < min_x:
					min_x = x
				if x > max_x:
					max_x = x
				if y < min_y:
					min_y = y
				if y > max_y:
					max_y = y
			rect = np.mgrid[min_x:max_x+1, min_y:max_y+1].reshape(2, -1).transpose()
			self.fleet_members_footprint[member]['footprint'] = []
			for coor in rect:
				x, y = coor
				self.fleet_members_footprint[member]['footprint'].append((x, y))

def main(args=None):
	rclpy.init()
	node = FleetMapServer()
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

