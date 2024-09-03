import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionDetectorState
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

class CollisionAvoidance(Node):
	def __init__(self):
		super().__init__('collision_avoidance_node')
		self.recv_cmd_time = time()
		self.twist_msg_val = (0.0, 0.0) # (linear.x, angular.z)
		self.is_stop = True
		self.collision_detected = False
		nav_cbg = MutuallyExclusiveCallbackGroup()
		self.nav_cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.passCtrlCmd, 10, callback_group=nav_cbg)

		collision_cbg = MutuallyExclusiveCallbackGroup()
		self.motor_cmd_pub = self.create_publisher(Twist, 'motor_vel', 10, callback_group=collision_cbg)
		self.cd_sub = self.create_subscription(CollisionDetectorState, 'collision_detector_state', self.confCDState, 10, callback_group=collision_cbg)

	def passCtrlCmd	(self, msg):
		self.recv_cmd_time = time()
		if not self.collision_detected:
			self.twist_msg_val = (msg.linear.x, msg.angular.z)

	def confCDState(self, msg):
		if time() - self.recv_cmd_time <= 0.5:
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
		elif not self.is_stop:
			self.twist_msg_val = (0.0, 0.0)
			self.sendCmd()

	def sendCmd(self):
		twist_msg = Twist()
		twist_msg.linear.x = self.twist_msg_val[0]
		twist_msg.angular.z = self.twist_msg_val[1]
		self.motor_cmd_pub.publish(twist_msg)
		self.is_stop = (self.twist_msg_val[0] == 0 and self.twist_msg_val[1] == 0)

def main(args=None):
	rclpy.init()
	node = CollisionAvoidance()
	executor = MultiThreadedExecutor()
	try:
		executor.add_node(node)
		executor.spin()
	except KeyboardInterrupt:
		pass
	except:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()

