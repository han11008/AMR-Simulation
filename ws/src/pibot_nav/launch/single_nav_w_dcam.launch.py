import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess, DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
	pkg_name = 'pibot_nav'
	pkg_share = get_package_share_directory(pkg_name)
	ld = LaunchDescription()

	keepout_map_path = LaunchConfiguration('keepout_map_path')
	declare_keepout_map_path = DeclareLaunchArgument('keepout_map_path', default_value='',
		description='Provide keepout map (.yaml) to use keepout filter')
	ld.add_action(declare_keepout_map_path)

	use_rviz = LaunchConfiguration('use_rviz')
	declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='True',
		description='Whether to use RViz')
	ld.add_action(declare_use_rviz)

	use_sim_time = LaunchConfiguration('use_sim_time', default='True')
	autostart = LaunchConfiguration('autostart', default='True')
	bond_timeout = LaunchConfiguration('bond_timeout', default='6.0')
	remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

	# ======================== PARAMETERS =========================
	bot_name = 'amr'

	robot_path = os.path.join(pkg_share, 'urdf/bot_w_dcam.urdf')
	world_path = os.path.join(pkg_share, 'world/test_low_obstacles.sdf')

	rtab_param = RewrittenYaml(
		source_file = os.path.join(pkg_share, 'param/rtab_params.yaml'),
		root_key = bot_name,
		param_rewrites = {'use_sim_time': use_sim_time},
		convert_types = False
	)

	bt_path = os.path.join(pkg_share, 'bt/nav2pose_replan_invalid.xml')
	lifecycle_nodes = ['amcl',
			'controller_server',
			'planner_server',
			'behavior_server',
			'bt_navigator',
			'collision_detector']

	nav2_param = RewrittenYaml(
		source_file = os.path.join(pkg_share, 'param/nav2_params_dcam.yaml'),
		root_key = bot_name,
		param_rewrites = {'default_nav_to_pose_bt_xml': bt_path,
				'use_sim_time': use_sim_time},
		convert_types = True
	)
	planner_w_keepout_param = RewrittenYaml(
		source_file = os.path.join(pkg_share, 'param/planner_w_keepout.yaml'),
		root_key = bot_name,
		param_rewrites = {'use_sim_time': use_sim_time},
		convert_types = True
	)

	rviz_config_path = os.path.join(pkg_share, 'rviz/dcam.rviz')
	# =============================================================

	# ======================== robot setup ========================
	gazebo_node = ExecuteProcess(
		cmd = f'gazebo {world_path} -s libgazebo_ros_init.so -s libgazebo_ros_factory.so'.split(' '),
		output = 'screen'
	)
	ld.add_action(gazebo_node)

	ld.add_action(
		Node(
			package = 'gazebo_ros',
			executable = 'spawn_entity.py',
			arguments = f'-entity robot -file {robot_path} -entity {bot_name} -robot_namespace {bot_name} -x 0.0 -y 0.0'.split(' ')
		)
	)

	ld.add_action(
		Node(
			package = 'robot_state_publisher',
			executable = 'robot_state_publisher',
			arguments = [robot_path],
			parameters = [{'use_sim_time': use_sim_time}],
			namespace = bot_name,
			remappings = remappings
		)
	)

	ld.add_action(
		Node(
			package = pkg_name,
			executable = 'navigator_client',
			name = 'navigator_client',
			namespace = bot_name,
			remappings = remappings,
			output = 'screen'
		)
	)

	ld.add_action(
		Node(
			package = pkg_name,
			executable = 'collision_avoidance',
			name = 'collision_avoidance',
			namespace = bot_name,
			remappings = remappings,
		        output = 'screen'
		)
	)
	# =============================================================

	# ======================== map server =========================
	ld.add_action(GroupAction(actions=[
		PushRosNamespace(namespace=bot_name),

		Node(
			package = 'rtabmap_slam',
			executable = 'rtabmap',
			name = 'rtabmap',
			arguments = ['--delete_db_on_start'],
			parameters = [rtab_param],
			remappings = remappings + [
				('rgb/image', 'color/image_raw'), ('rgb/camera_info', 'color/camera_info'),
				('depth/image', 'depth/image_raw'),
				('map', 'rtabmap_map')
			]
		)
	]))

	ld.add_action(GroupAction(actions=[
			PushRosNamespace(namespace=bot_name),

			Node(
				condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" == ""'])),
				package = 'nav2_lifecycle_manager',
				executable = 'lifecycle_manager',
				name = 'lifecycle_manager_map_server',
				emulate_tty = True,
				respawn = True,
				respawn_delay = 2.0,
				parameters = [{
					'use_sim_time': use_sim_time,
					'autostart': autostart,
					'node_names': ['map_server'],
					'bond_timeout': bond_timeout
				}],
				output = 'screen'
			),

			Node(
				condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" != ""'])),
				package = 'nav2_lifecycle_manager',
				executable = 'lifecycle_manager',
				name = 'lifecycle_manager_map_server',
				emulate_tty = True,
				respawn = True,
				respawn_delay = 2.0,
				parameters = [{
					'use_sim_time': use_sim_time,
					'autostart': autostart,
					'node_names': ['map_server', 'filter_map_server', 'costmap_filter_info_server'],
					'bond_timeout': bond_timeout
				}],
				output = 'screen'
			),

			Node(
				package = 'nav2_map_server',
				executable = 'map_server',
				name = 'map_server',
				emulate_tty = True,
				respawn = True,
				respawn_delay = 2.0,
				parameters = [{
					'use_sim_time': use_sim_time,
					'yaml_filename': os.path.join(pkg_share, 'map/map.yaml'),
					'topic_name': 'map',
					'frame_id': 'map'
				}],
				remappings = remappings,
				output = 'screen'
			),

			Node(
				condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" != ""'])),
				package = 'nav2_map_server',
				executable = 'map_server',
				name = 'filter_map_server',
				emulate_tty = True,
				respawn = True,
				respawn_delay = 2.0,
				parameters = [{
					'use_sim_time': use_sim_time,
					'yaml_filename': os.path.join(pkg_share, 'map/keepout.yaml'),
					'topic_name': 'keepout_filter',
					'frame_id': 'map'
				}],
				remappings = remappings,
				output = 'screen'
			),

			Node(
				condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" != ""'])),
				package = 'nav2_map_server',
				executable = 'costmap_filter_info_server',
				name = 'costmap_filter_info_server',
				emulate_tty = True,
				respawn = True,
				respawn_delay = 2.0,
				parameters = [{
					'use_sim_time': use_sim_time,
					'type': 0,
					'filter_info_topic': 'costmap_filter_info',
					'mask_topic': 'keepout_filter',
					'base': 0.0,
					'multiplier': 1.0
				}],
				remappings = remappings,
				output = 'screen'
			)
	]))
	# =============================================================

	
	# ======================== Nav2 stack =========================
	ld.add_action(GroupAction(actions=[
		PushRosNamespace(namespace=bot_name),
		SetRemap(src=['/scan'], dst=['/', bot_name, '/scan']),
		SetRemap(src=['/cloud_obstacles'], dst=['/', bot_name, '/cloud_obstacles']),
		SetRemap(src=['/modified_map'], dst=['/', bot_name, '/map']),

		Node(
			package = 'nav2_lifecycle_manager',
			executable = 'lifecycle_manager',
			name = 'lifecycle_manager_navigation',
			emulate_tty = True,
			respawn = True,
			respawn_delay = 2.0,
			parameters = [{
				'use_sim_time': use_sim_time,
				'autostart': autostart,
				'node_names': lifecycle_nodes,
				'bond_timeout': bond_timeout
			}],
			output = 'screen'
		),

		Node(
			package = 'nav2_amcl',
			executable = 'amcl',
			name = 'amcl',
			emulate_tty = True,
			respawn = True,
			respawn_delay = 2.0,
			parameters = [nav2_param],
			remappings = remappings,
			output = 'screen'
		),

		Node(
			package = 'nav2_controller',
			executable = 'controller_server',
			name = 'controller_server',
			emulate_tty = True,
			respawn = True,
			respawn_delay = 2.0,
			parameters = [nav2_param],
			remappings = remappings,
			output = 'screen'
		),

		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" == ""'])),
			package = 'nav2_planner',
			executable = 'planner_server',
			name = 'planner_server',
			emulate_tty = True,
			respawn = True,
			respawn_delay = 2.0,
			parameters = [nav2_param],
			remappings = remappings,
			output = 'screen'
		),

		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" != ""'])),
			package = 'nav2_planner',
			executable = 'planner_server',
			name = 'planner_server',
			emulate_tty = True,
			respawn = True,
			respawn_delay = 2.0,
			parameters = [planner_w_keepout_param],
			remappings = remappings,
			output = 'screen'
		),

		Node(
			package = 'nav2_behaviors',
			executable = 'behavior_server',
			name = 'behavior_server',
			emulate_tty = True,
			respawn = True,
			respawn_delay = 2.0,
			parameters = [nav2_param],
			remappings = remappings,
			output = 'screen'
		),

		Node(
			package = 'nav2_bt_navigator',
			executable = 'bt_navigator',
			name = 'bt_navigator',
			emulate_tty = True,
			respawn = True,
			respawn_delay = 2.0,
			parameters = [nav2_param],
			remappings = remappings,
			output = 'screen'
		),

		Node(
			package = 'nav2_collision_monitor',
			executable = 'collision_detector',
			name = 'collision_detector',
			emulate_tty = True,
			respawn = True,
			respawn_delay = 2.0,
			parameters = [nav2_param],
			remappings = remappings,
			output = 'screen'
		)
	]))
	# =============================================================

	# =========================== RViz ============================
	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression([use_rviz])),
			package = 'rviz2',
			executable = 'rviz2',
			name = 'rviz2',
			arguments = ['-d', rviz_config_path, '--ros-args', '--log-level', 'error'],
			parameters = [{'use_sim_time': use_sim_time}],
			namespace = bot_name,
			remappings = remappings + [('/goal_pose', 'goal_pose'), ('/clicked_point', 'clicked_point'), ('/initialpose', 'initialpose')]
		)
	)
	# =============================================================

	return ld
