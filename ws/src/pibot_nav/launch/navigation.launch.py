import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
	pkg_name = 'pibot_nav'
	pkg_share = get_package_share_directory(pkg_name)
	ld = LaunchDescription()

	use_sim_time = LaunchConfiguration('use_sim_time', default='True')
	autostart = LaunchConfiguration('autostart', default='True')
	bond_timeout = LaunchConfiguration('bond_timeout', default='6.0')
	remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/modified_map', '/map')]

	keepout_map_path = LaunchConfiguration('keepout_map_path')
	declare_keepout_map_path = DeclareLaunchArgument('keepout_map_path', default_value='',
		description='Provide keepout map (.yaml) to use keepout filter')
	ld.add_action(declare_keepout_map_path)

	# ---------- declare parameters with/without keepout filter ----------
	bt_path = os.path.join(pkg_share, 'bt/nav2pose_pibot.xml')
	nav2_param_path = RewrittenYaml(
		source_file = os.path.join(pkg_share, 'param/nav2_params.yaml'),
		param_rewrites = {'default_nav_to_pose_bt_xml': bt_path,
				'use_sim_time': use_sim_time},
		convert_types = True
	)
	nav2_param_w_keepout_path = RewrittenYaml(
		source_file = os.path.join(pkg_share, 'param/nav2_params_with_keepout.yaml'),
		param_rewrites = {'default_nav_to_pose_bt_xml': bt_path,
				'yaml_filename': keepout_map_path,
				'use_sim_time': use_sim_time},
		convert_types = True
	)
	lifecycle_nodes = ['controller_server',
			'planner_server',
			'behavior_server',
			'bt_navigator']
	lifecycle_nodes_w_keepout = ['controller_server',
			'planner_server',
			'behavior_server',
			'bt_navigator',
			'map_server', 'costmap_filter_info_server']
	# --------------------------------------------------------------------

	# ==================== Nav2 stack WITHOUT keepout filter ====================
	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" == ""'])),
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
		)
	)

	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" == ""'])),
			package = 'nav2_controller',
			executable = 'controller_server',
			name = 'controller_server',
			emulate_tty = True,
			respawn = True,
		        respawn_delay = 2.0,
			parameters = [nav2_param_path],
			remappings = remappings,
		        output = 'screen'
		)
	)

	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" == ""'])),
			package = 'nav2_planner',
			executable = 'planner_server',
			name = 'planner_server',
			emulate_tty = True,
			respawn = True,
		        respawn_delay = 2.0,
			parameters = [nav2_param_path],
			remappings = remappings,
		        output = 'screen'
		)
	)

	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" == ""'])),
			package = 'nav2_behaviors',
			executable = 'behavior_server',
			name = 'behavior_server',
			emulate_tty = True,
			respawn = True,
		        respawn_delay = 2.0,
			parameters = [nav2_param_path],
			remappings = remappings,
		        output = 'screen'
		)
	)

	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" == ""'])),
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
	)
	# ===========================================================================

	# ===================== Nav2 stack WITH keepout filter ======================
	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" != ""'])),
			package = 'nav2_lifecycle_manager',
			executable = 'lifecycle_manager',
			name = 'lifecycle_manager_navigation',
			emulate_tty = True,
			respawn = True,
		        respawn_delay = 2.0,
			parameters = [{'use_sim_time': use_sim_time},
				{'autostart': autostart},
				{'node_names': lifecycle_nodes_w_keepout},
				{'bond_timeout': bond_timeout}],
		        output = 'screen'
		)
	)

	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" != ""'])),
			package = 'nav2_controller',
			executable = 'controller_server',
			name = 'controller_server',
			emulate_tty = True,
			respawn = True,
		        respawn_delay = 2.0,
			parameters = [nav2_param_w_keepout_path],
			remappings = remappings,
		        output = 'screen'
		)
	)

	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" != ""'])),
			package = 'nav2_planner',
			executable = 'planner_server',
			name = 'planner_server',
			emulate_tty = True,
			respawn = True,
		        respawn_delay = 2.0,
			parameters = [nav2_param_w_keepout_path],
			remappings = remappings,
		        output = 'screen'
		)
	)

	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" != ""'])),
			package = 'nav2_behaviors',
			executable = 'behavior_server',
			name = 'behavior_server',
			emulate_tty = True,
			respawn = True,
		        respawn_delay = 2.0,
			parameters = [nav2_param_w_keepout_path],
			remappings = remappings,
		        output = 'screen'
		)
	)

	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" != ""'])),
			package = 'nav2_bt_navigator',
			executable = 'bt_navigator',
			name = 'bt_navigator',
			emulate_tty = True,
			respawn = True,
		        respawn_delay = 2.0,
			parameters = [nav2_param_w_keepout_path],
			remappings = remappings,
		        output = 'screen'
		)
	)

	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" != ""'])),
			package = 'nav2_map_server',
			executable = 'map_server',
			name = 'map_server',
			emulate_tty = True,
			respawn = True,
		        respawn_delay = 2.0,
			parameters = [nav2_param_w_keepout_path],
			remappings = remappings,
		        output = 'screen'
		)
	)

	ld.add_action(
		Node(
			condition = IfCondition(PythonExpression(['"', keepout_map_path,  '" != ""'])),
			package = 'nav2_map_server',
			executable = 'costmap_filter_info_server',
			name = 'costmap_filter_info_server',
			emulate_tty = True,
			respawn = True,
		        respawn_delay = 2.0,
			parameters = [nav2_param_w_keepout_path],
			remappings = remappings,
		        output = 'screen'
		)
	)
	# ===========================================================================

	# =========================== collision avoidance ===========================
	cd_param_path = RewrittenYaml(
		source_file = os.path.join(pkg_share, 'param/nav2_collision_detector_params.yaml'),
		param_rewrites = {'use_sim_time': use_sim_time},
		convert_types = True
	)

	collision_lifecycle_manager = Node(
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
	)

	collision_detector = Node(
		package = 'nav2_collision_monitor',
		executable = 'collision_detector',
		name = 'collision_detector',
		emulate_tty = True,
		respawn = True,
                respawn_delay = 2.0,
		parameters = [cd_param_path],
		output = 'screen'
	)

	ld.add_action(collision_lifecycle_manager)
	ld.add_action(collision_detector)	

	collision_avoidance = Node(
		package = pkg_name,
		executable = 'collision_avoidance',
		name = 'collision_avoidance',
		emulate_tty = True,
		respawn = True,
                respawn_delay = 2.0,
                output = 'screen'
	)

	ld.add_action(collision_avoidance)
	# ===========================================================================

	return ld

