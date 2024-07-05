import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
	pkg_name = 'pibot_nav'
	pkg_share = get_package_share_directory(pkg_name)
	ld = LaunchDescription()

	stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

	use_sim_time = LaunchConfiguration('use_sim_time', default='True')
	autostart = LaunchConfiguration('autostart', default='True')
	bond_timeout = LaunchConfiguration('bond_timeout', default='6.0')
	remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

	# ======================== Nav2 stack =========================
	bt_path = os.path.join(pkg_share, 'bt/nav2pose_pibot.xml')
	nav2_param_path = RewrittenYaml(
		source_file = os.path.join(pkg_share, 'param/nav2_params.yaml'),
		param_rewrites = {'default_nav_to_pose_bt_xml': bt_path, 
				'use_sim_time': use_sim_time},
		convert_types = True
	)
	lifecycle_nodes = ['controller_server',
			'planner_server',
			'behavior_server',
			'bt_navigator']

	lifecycle_manager = Node(
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

	controller_server = Node(
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

	planner_server = Node(
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

	behavior_server = Node(
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

	bt_navigator = Node(
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

	ld.add_action(lifecycle_manager)
	ld.add_action(controller_server)
	ld.add_action(planner_server)
	ld.add_action(behavior_server)
	ld.add_action(bt_navigator)
	# =============================================================

	# ==================== collision avoidance ====================
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
	# =============================================================

	return ld
