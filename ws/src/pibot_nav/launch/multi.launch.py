import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess, DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from nav2_common.launch import RewrittenYaml

ROBOTS = {
	'bot1': [-0.2, 0.2],
	'bot2': [1.0, 3.0]
}

def generate_launch_description():
	pkg_name = 'pibot_nav'
	pkg_share = get_package_share_directory(pkg_name)
	ld = LaunchDescription()

#	bot_name = LaunchConfiguration('bot_name', default='bot1')
#	declare_namespace = DeclareLaunchArgument('bot_name', default_value='bot1',
#		description='Define namespace')
#	ld.add_action(declare_namespace)

	use_rviz = LaunchConfiguration('use_rviz')
	declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='True',
		description='Whether to use RViz')
	ld.add_action(declare_use_rviz)

	use_sim_time = LaunchConfiguration('use_sim_time', default='True')
	autostart = LaunchConfiguration('autostart', default='True')
	bond_timeout = LaunchConfiguration('bond_timeout', default='6.0')
	remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

	bt_path = os.path.join(pkg_share, 'bt/nav2pose_pibot.xml')
	lifecycle_nodes = ['amcl',
			'controller_server',
			'planner_server',
			'behavior_server',
			'bt_navigator']

	# ======================== robot setup ========================
	robot_path = os.path.join(pkg_share, 'urdf/sim_bot.urdf')
	world_path = os.path.join(pkg_share, 'world/sw.sdf')

	gazebo_node = ExecuteProcess(
		cmd = f'gazebo {world_path} -s libgazebo_ros_init.so -s libgazebo_ros_factory.so'.split(' '),
		output = 'screen'
	)
	ld.add_action(gazebo_node)

	for bot_name in ROBOTS.keys():
		ld.add_action(
			Node(
				package = 'gazebo_ros',
				executable = 'spawn_entity.py',
				arguments = f'-entity robot -file {robot_path} -entity {bot_name} -robot_namespace {bot_name} -x {ROBOTS[bot_name][0]} -y {ROBOTS[bot_name][1]}'.split(' '),
				namespace = bot_name
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
	# =============================================================

	# ======================== map server =========================
	lifecycle_node = Node(
		package = 'nav2_lifecycle_manager',
		executable = 'lifecycle_manager',
		name = 'lifecycle_manager_map_server',
		emulate_tty = True,
		respawn = True,
		respawn_delay = 2.0,
		parameters = [{'use_sim_time': use_sim_time},
			{'autostart': autostart},
			{'node_names': ['map_server']},
			{'bond_timeout': bond_timeout}],
		output = 'screen'
	)

	map_server_node = Node(
		package = 'nav2_map_server',
		executable = 'map_server',
		name = 'map_server',
		emulate_tty = True,
		respawn = True,
		respawn_delay = 2.0,
		parameters = [
			{'use_sim_time': use_sim_time},
			{'yaml_filename': os.path.join(pkg_share, 'map/map.yaml')},
			{'topic_name': '/map'},
			{'frame_id': 'map'}
		],
		output = 'screen'
	)

	ld.add_action(lifecycle_node)
	ld.add_action(map_server_node)

	for bot_name in ROBOTS.keys():
		ld.add_action(
			Node(
				package = pkg_name,
				executable = 'fleet_map_server',
				name = 'fleet_map_server',
				namespace = bot_name,
				remappings = remappings,
				output = 'screen'
			)
		)
	# =============================================================

	
	# ======================== Nav2 stack =========================
	for bot_name in ROBOTS.keys():
		nav2_param_path = RewrittenYaml(
			source_file = os.path.join(pkg_share, 'param/nav2_params_multi.yaml'),
			root_key = bot_name,
			param_rewrites = {'default_nav_to_pose_bt_xml': bt_path,
					'use_sim_time': use_sim_time},
			convert_types = True
		)

		nav2_stack_group = GroupAction(
			actions = [
				PushRosNamespace(namespace=bot_name),
				SetRemap(src=['/scan'], dst=['/', bot_name, '/scan']),
				SetRemap(src=['/modified_map'], dst=['/', bot_name, '/map']),

				Node(
					package = 'nav2_lifecycle_manager',
					executable = 'lifecycle_manager',
					name = 'lifecycle_manager_navigation',
					emulate_tty = True,
					respawn = True,
					respawn_delay = 2.0,
					parameters = [{'use_sim_time': use_sim_time},
						{'autostart': autostart},
						{'node_names': lifecycle_nodes},
						{'bond_timeout': bond_timeout}],
					output = 'screen'
				),

				Node(
					package = 'nav2_amcl',
					executable = 'amcl',
					name = 'amcl',
					emulate_tty = True,
					respawn = True,
					respawn_delay = 2.0,
					parameters = [nav2_param_path],
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
					parameters = [nav2_param_path],
					remappings = remappings,
					output = 'screen'
				),

				Node(
					package = 'nav2_planner',
					executable = 'planner_server',
					name = 'planner_server',
					emulate_tty = True,
					respawn = True,
					respawn_delay = 2.0,
					parameters = [nav2_param_path],
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
					parameters = [nav2_param_path],
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
					parameters = [nav2_param_path],
					remappings = remappings,
					output = 'screen'
				)
			]
		)

		ld.add_action(nav2_stack_group)
	# =============================================================

	# ==================== collision avoidance ====================
	for bot_name in ROBOTS.keys():
		cd_param_path = RewrittenYaml(
			source_file = os.path.join(pkg_share, 'param/nav2_collision_detector_params.yaml'),
			root_key = bot_name,
			param_rewrites = {'use_sim_time': use_sim_time},
			convert_types = True
		)

		collision_avoidance_group = GroupAction(
			actions = [
				PushRosNamespace(namespace=bot_name),

				Node(
					package = 'nav2_lifecycle_manager',
					executable = 'lifecycle_manager',
					name = 'lifecycle_manager_collision',
					emulate_tty = True,
					respawn = True,
					respawn_delay = 2.0,
					parameters = [{'use_sim_time': use_sim_time},
						{'autostart': autostart},
						{'node_names': ['collision_detector']},
						{'bond_timeout': bond_timeout}],
					output = 'screen'
				),

				Node(
					package = 'nav2_collision_monitor',
					executable = 'collision_detector',
					name = 'collision_detector',
					emulate_tty = True,
					respawn = True,
					respawn_delay = 2.0,
					parameters = [cd_param_path],
					remappings = remappings,
					output = 'screen'
				)
			]
		)

		ld.add_action(collision_avoidance_group)
	# =============================================================

	# ===================== fleet management ======================
	ld.add_action(
		Node(
			package = pkg_name,
			executable = 'task_server',
			name = 'task_server',
			output = 'screen'
		)
	)

	for bot_name in ROBOTS.keys():
		ld.add_action(
			Node(
				package = pkg_name,
				executable = 'task_client',
				name = 'task_client',
				parameters = [
					{'keepout_radius': 1.5}
				],
				namespace = bot_name,
				remappings = remappings,
				output = 'screen'
			)
		)
	# =============================================================

	# =========================== RViz ============================
	rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_with_cam.rviz')

	for bot_name in ROBOTS.keys():
		ld.add_action(
			Node(
				condition = IfCondition(PythonExpression([use_rviz])),
				package = 'rviz2',
				executable = 'rviz2',
				name = 'rviz2',
				arguments = ['-d', rviz_config_path],
				parameters = [{'use_sim_time': use_sim_time}],
				namespace = bot_name,
				remappings = remappings + [('/goal_pose', 'goal_pose'), ('/clicked_point', 'clicked_point'), ('/initialpose', 'initialpose')]
			)
		)
	# =============================================================

	return ld
