import serial
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Pose, Twist

# define length in meters
WHEEL_DIAMETER = 0.08
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * math.pi
DIST_BETWEEN_WHEELS = 0.3

# define timer period in seconds
ODOM_PUB_PERIOD = 0.1
ENCD_GET_PERIOD = 0.1

#define wheel spinning direction
MOTOR_STOP = 0
MOTOR_FORWARD = 1
MOTOR_BACKWARD = -1

class DIFF_DRIVE:
	def __init__(self, encd_dev, motor_dev):
		self.encd = serial.Serial(encd_dev, 115200)
		self.motor = serial.Serial(motor_dev, 115200)
		self.left_dir = self.right_dir = 0
		self.left_pwm = self.right_pwm = 0

	def getRotSpeed(self):
		self.encd.flushInput()
		# format: <left_rev_per_sec>,<right_rev_per_sec>\r\n
		info = self.encd.readline().decode('utf-8').split(',')
		lrps = float(info[0])
		rrps = float(info[1].split('\r')[0])
		return lrps, rrps

	def sendCmd(self):
		self.motor.write(f'{self.left_dir},{self.left_pwm},{self.right_dir},{self.right_pwm}#\r\n'.encode('utf-8'))

	def pause(self):
		self.left_dir = self.right_dir = 0
		self.sendCmd()

	def stop(self):
		self.left_pwm = self.right_pwm = 0
		self.pause()

	def move(self, target_left_rps, target_right_rps):
		if target_left_rps == 0 or target_right_rps == 0:
			self.pause()
			return 0, 0
		if target_left_rps > 0:
			self.left_dir = MOTOR_FORWARD
		else:
			self.left_dir = MOTOR_BACKWARD
			target_left_rps = -target_left_rps
		if target_right_rps > 0:
			self.right_dir = MOTOR_FORWARD
		else:
			self.right_dir = MOTOR_BACKWARD
			target_right_rps = -target_right_rps

		########## calculate pwm ##########
		if target_left_rps <= 1.1:
			self.left_pwm = round(88.2 * target_left_rps + 49.5)
		else:
			self.left_pwm = round(218.4 * target_left_rps - 92.1)
		self.left_pwm = max(min(self.left_pwm, 255), 0)

		if target_right_rps <= 1.1:
			self.right_pwm = round(88.2 * target_right_rps + 49.5)
		else:
			self.right_pwm = round(218.4 * target_right_rps - 92.1)
		self.right_pwm = max(min(self.right_pwm, 255), 0)
		###################################
		self.sendCmd()
		return self.left_pwm, self.right_pwm

class Listener(Node):
	def __init__(self):
		super().__init__('motor_listener_node')
		self.initOdom()

		self.declare_parameter('encd_dev', '/dev/ttyUSB1')
		encd_dev = self.get_parameter('encd_dev').get_parameter_value().string_value
		self.declare_parameter('motor_dev', '/dev/ttyUSB2')
		motor_dev = self.get_parameter('motor_dev').get_parameter_value().string_value
		self.motor = DIFF_DRIVE(encd_dev=encd_dev, motor_dev=motor_dev)

		motor_encd_cbg = MutuallyExclusiveCallbackGroup()
		self.motor_encd_timer = self.create_timer(ENCD_GET_PERIOD, self.encdTimerCallback, callback_group=motor_encd_cbg)
		motor_cmd_cbg = MutuallyExclusiveCallbackGroup()
		self.vel_listener = self.create_subscription(Twist, 'motor_vel', self.getCmd, 10, callback_group=motor_cmd_cbg)

		self.get_logger().info('Start listening to commands...')

	def getCmd(self, msg):
		right_vel = msg.angular.z * DIST_BETWEEN_WHEELS / 2 + msg.linear.x
		left_vel = (2 * msg.linear.x - right_vel) / WHEEL_CIRCUMFERENCE
		right_vel /= WHEEL_CIRCUMFERENCE
		log = self.motor.move(left_vel, right_vel)
#		if 0 < left_vel < 0.6 or 0 < right_vel < 0.6:
#			self.get_logger().info(f'Target velocity: linear={msg.linear.x}, angular={msg.angular.z}; transform to rotating speed(rev/s)/pwm: ({left_vel},{right_vel})/{log}')

	def encdTimerCallback(self):
		# revolutions per second, negative if counterclockwise
		left_rps, right_rps = self.motor.getRotSpeed()
		# left wheel rotates counterclockwise to go forward
		left_vel = -left_rps * WHEEL_CIRCUMFERENCE
		right_vel = right_rps * WHEEL_CIRCUMFERENCE
		self.twist.linear.x = (left_vel + right_vel) / 2
		self.twist.angular.z = (right_vel - left_vel) / DIST_BETWEEN_WHEELS

	def initOdom(self):
		self.pose = Pose()
		self.twist = Twist()
		self.prev_time = self.get_clock().now()
		odom_cbg = MutuallyExclusiveCallbackGroup()
		self.odom_pub = self.create_publisher(Odometry, 'odom', 10, callback_group=odom_cbg)
		self.timer = self.create_timer(ODOM_PUB_PERIOD, self.publishOdom, callback_group=odom_cbg)

	def publishOdom(self):
		odom_msg = Odometry()
		cur_time = self.get_clock().now()
		period = (cur_time - self.prev_time).nanoseconds/1000000000

		_, _, yaw = self.quaternionToEuler(self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
		z_sin = math.sin(yaw)
		z_cos = math.cos(yaw)
		self.pose.position.x += self.twist.linear.x * period * z_cos
		self.pose.position.y += self.twist.linear.x * period * z_sin
		qx, qy, qz, qw = self.eulerToQuaternion(self.twist.angular.x * period, self.twist.angular.y * period, self.twist.angular.z * period)
		qx, qy, qz, qw = self.rotationAccumulate(qx, qy, qz, qw, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
		# While running on x-y plane, the quaternion values of x and y should not change.
		# self.pose.orientation.x = qx
		# self.pose.orientation.y = qy
		self.pose.orientation.z = qz
		self.pose.orientation.w = qw

		odom_msg.header.stamp = cur_time.to_msg()
		odom_msg.header.frame_id = 'odom'

		odom_msg.child_frame_id = 'base_footprint'

		odom_msg.pose.pose = self.pose
		odom_msg.twist.twist = self.twist

		self.odom_pub.publish(odom_msg)
		self.prev_time = cur_time

	def eulerToQuaternion(self, roll, pitch, yaw):
		qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
		qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
		qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		return qx, qy, qz, qw

	def quaternionToEuler(self, qx, qy, qz, qw):
		t0 = 2.0 * (qw * qx + qy * qz)
		t1 = 1.0 - 2.0 * (qx * qx + qy * qy)
		roll = math.atan2(t0, t1)
		t2 = 2.0 * (qw * qy - qz * qx)
		t2 = 1.0 if t2 > 1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch = math.asin(t2)
		t3 = 2.0 * (qw * qz + qx * qy)
		t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
		yaw = math.atan2(t3, t4)
		return roll, pitch, yaw

	def rotationAccumulate(self, x1, y1, z1, w1, x2, y2, z2, w2):
		qx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
		qy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
		qz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
		qw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
		return qx, qy, qz, qw

def main(args=None):
	rclpy.init(args=args)
	node = Listener()
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

