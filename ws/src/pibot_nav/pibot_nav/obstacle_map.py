import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py.point_cloud2 import read_points_numpy, create_cloud_xyz32
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np

REQUEST_TF_PERIOD = 0.2
OBSTACLE_MAX_DIST = 1.5

class ObstacleMap(Node):
	def __init__(self):
		super().__init__('obstacle_map_node')
		map_cbg = MutuallyExclusiveCallbackGroup()
		self.last_stop_pos = None
		self.map_sub = self.create_subscription(PointCloud2, 'cloud_map', self.recvPointCloud, 1, callback_group=map_cbg)
		self.map_pub = self.create_publisher(PointCloud2, 'cloud_obstacles', 1, callback_group=map_cbg)

		clear_cbg = MutuallyExclusiveCallbackGroup()
		self.init_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.recvInitPose, 1, callback_group=clear_cbg)
		self.cleaner = self.create_service(Empty, 'clear_cloud_obstacles', self.recvClearReq, callback_group=clear_cbg)
		self.octo_client = self.create_client(Empty, 'octomap_server/reset', callback_group=clear_cbg)

		tf_cbg = MutuallyExclusiveCallbackGroup()
		self.pos = None
		self.tf_buffer = Buffer()
		tf_trans = TransformListener(self.tf_buffer, self)
		self.req_tf_timer = self.create_timer(REQUEST_TF_PERIOD, self.req_tf, callback_group=tf_cbg)

	def recvPointCloud(self, msg):
		if self.pos is None:
			pass
		elif self.last_stop_pos is None:
			self.map_pub.publish(msg)
		else:
			points = read_points_numpy(msg)
			dists = np.max(np.abs(np.vstack((points[:, 0] - self.pos[0], points[:, 1] - self.pos[1]))), axis=0)
			obstacle_points = points[dists <= OBSTACLE_MAX_DIST]

			header = Header()
			header.stamp = self.get_clock().now().to_msg()
			header.frame_id = 'map'
			obstacle_map = create_cloud_xyz32(header, obstacle_points)
			self.map_pub.publish(obstacle_map)
			if max(abs(self.last_stop_pos[0] - self.pos[0]), abs(self.last_stop_pos[1] - self.pos[1])) >= OBSTACLE_MAX_DIST:
				self.last_stop_pos = None

	def recvInitPose(self, msg):
		self.clearMap(exec_twice=True)

	def recvClearReq(self, req, res):
		self.clearMap()
		return res

	def clearMap(self, exec_twice=False):
		self.last_stop_pos = self.pos
		count = 0
		while not self.octo_client.wait_for_service(timeout_sec=1.5):
			count += 1
			if count >= 5:
				self.get_logger().warn('Octomap server has no response')
				return
		self.get_logger().info('Cleaning 3D map...')
		future = self.octo_client.call_async(Empty.Request())
		if exec_twice:
			future.add_done_callback(self._cleanAgain)

	def _cleanAgain(self, future): # previous pointcloud data might be still retained
		_ = self.octo_client.call_async(Empty.Request())

	def req_tf(self):
		try:
			tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
			self.pos = (tf.transform.translation.x, tf.transform.translation.y)
		except:
			pass

def main(args=None):
	rclpy.init()
	node = ObstacleMap()
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
	
