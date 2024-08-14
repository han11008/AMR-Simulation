import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty

class OctomapResetter(Node):
	def __init__(self):
		super().__init__('octomap_reset_node')
		_ = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.recvInitPose, 1)
		self.octo_client = self.create_client(Empty, 'octomap_server/reset')

	def recvInitPose(self, msg):
		count = 0
		while not self.octo_client.wait_for_service(timeout_sec=1.5):
			count += 1
			if count >= 5:
				self.get_logger().warn('Octomap server has no response')
				return
		self.get_logger().info('Cleaning 3D map...')
		future = self.octo_client.call_async(Empty.Request())
		future.add_done_callback(self._cleanAgain)

	def _cleanAgain(self, future): # previous pointcloud data might be still retained
		_ = self.octo_client.call_async(Empty.Request())

def main(args=None):
	rclpy.init()
	node = OctomapResetter()
	try:
		rclpy.spin(node)
		node.destroy_node()
		rclpy.shutdown()
	except:
		pass

if __name__ == '__main__':
	main()
