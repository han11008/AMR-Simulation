import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
	pkg_name = 'pibot_nav'
	pkg_share = get_package_share_directory(pkg_name)
	ld = LaunchDescription()

	pose_graph_path = LaunchConfiguration('pose_graph_path')
	default_pose_graph_path = os.path.join(pkg_share, 'stb_map/map')
	declare_pose_graph_path = DeclareLaunchArgument('pose_graph_path', default_value=default_pose_graph_path,
		description='Pose graph for slam_toolbox to load map')
	ld.add_action(declare_pose_graph_path)

	use_rviz = LaunchConfiguration('use_rviz')
	declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='False',
		description='Whether to use RViz')
	ld.add_action(declare_use_rviz)

	use_sim_time = LaunchConfiguration('use_sim_time', default='True')

	# ====================== robot setup =======================
	robot_path = os.path.join(pkg_share, 'urdf/sim_bot.urdf')
	world_path = os.path.join(pkg_share, 'world/test.sdf')

	gazebo_node = ExecuteProcess(
		cmd = f'gazebo {world_path} -s libgazebo_ros_init.so -s libgazebo_ros_factory.so'.split(' '),
		output = 'screen'
	)

	spawn_entity_node = Node(
		package = 'gazebo_ros',
		executable = 'spawn_entity.py',
		arguments = f'-entity robot -file {robot_path} -x 0 -y 0'.split(' ')
	)

	robot_state_publisher_node = Node(
		package = 'robot_state_publisher',
		executable = 'robot_state_publisher',
		arguments = [robot_path],
		parameters = [{'use_sim_time': use_sim_time}]
	)

	ld.add_action(gazebo_node)
	ld.add_action(spawn_entity_node)
	ld.add_action(robot_state_publisher_node)
	# ==========================================================

	# ====================== localization ======================
	slam_toolbox_config_path = os.path.join(pkg_share, 'param/stb_config.yaml')

	slam_toolbox_node = Node(
		package = 'slam_toolbox',
		executable = 'localization_slam_toolbox_node',
		name = 'slam_toolbox',
		parameters = [
			slam_toolbox_config_path,
			{'use_sim_time': use_sim_time,
			'mode': 'localization',
			'map_file_name': pose_graph_path,
			'tf_buffer_duration': 30.0}
		],
		output = 'screen'
	)

	ld.add_action(slam_toolbox_node)
	# ==========================================================

	# ==================== navigator_client ====================
	navigator_client_node = Node(
		package = pkg_name,
		executable = 'navigator_client',
		name = 'navigator_client',
		output = 'screen'
	)

	ld.add_action(navigator_client_node)
	# ==========================================================
	
	# ========================== RViz ==========================
	rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_with_cam.rviz')

	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression([use_rviz])),
			package = 'rviz2',
			executable = 'rviz2',
			name = 'rviz2',
			arguments = f'-d {rviz_config_path}'.split(' '),
			parameters = [{'use_sim_time': use_sim_time}]
		)
	)
	# ==========================================================

	return ld

