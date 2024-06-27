from textual import on, work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.screen import Screen
from textual.widgets import Footer, Button, Label, Log
from select import select
import subprocess, shlex, re, os
import json, serial

class SelectModeScreen(Screen):
	def __init__(self):
		super().__init__()

	def compose(self) -> ComposeResult:
		yield Label('Select launch mode')
		yield Button('SLAM', id='slam')
		yield Button('Navigation', id='nav')

	@on(Button.Pressed, '#slam')
	def slamStage(self):
		self.app.launchROS2(True)

	@on(Button.Pressed, '#nav')
	def navStage(self):
		self.app.launchROS2(False)

class ROS2HelperApp(App):
	BINDINGS = [
		Binding('q', 'toggle_quit', 'Quit'),
		Binding('ctrl+c', 'toggle_quit', 'Quit', show=False)
	]
	CSS = '''
	Screen {
		align: center middle;
	}
	SelectModeScreen {
		layout: grid;
		grid-size: 2 2;
		align: center bottom;
	}
	SelectModeScreen > Label {
		margin: 1;
		text-align: center;
		column-span: 2;
		width: 1fr;
	}
	SelectModeScreen Button {
		margin: 2;
		width: 1fr;
	}
	#log {
		border: round;
	}
	'''

	def on_mount(self) -> None:
		self.ros_ws = str(__file__).split('executables')[0] + 'ws'
		self.set_env = f'source ~/rp_ws/install/setup.bash ; source ~/slam_and_nav_ws/install/setup.bash ; source {self.ros_ws}/install/setup.bash'
		if os.environ.get('SSH_CLIENT') is None:
			self.push_screen(SelectModeScreen())
		else:
			self.app.launchROS2(False, True)

	def compose(self) -> ComposeResult:
		yield Footer()
		yield Log(id='log', auto_scroll=True)

	def action_toggle_quit(self) -> None:
		if hasattr(self, 'running'):
			self.running = False
		if hasattr(self, 'dds_proc'):
			self.dds_proc.terminate()
			self.dds_proc.wait()
		if hasattr(self, 'proc_step1'):
			self.proc_step1.terminate()
			self.proc_step1.wait()
		if hasattr(self, 'proc_step2'):
			self.proc_step2.terminate()
			self.proc_step2.wait()
		if hasattr(self, 'opened_processes'):
			for proc in self.opened_processes:
				_ = subprocess.Popen(f'pgrep {proc[:15]} | xargs kill -9 2> /dev/null', stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
		_ = subprocess.Popen('fastdds shm clean', stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True).wait()
		self.exit()

	def action_toggle_save_map(self) -> None:
		self.saveMap()

	@work(exclusive=True, thread=True)
	def saveMap(self):
		if not hasattr(self, 'is_saving_map'):
			self.is_saving_map = True
			self.query_one("#log").write_line(f'Saving map...')
			# ---------- Save pose-graph for slam_toolbox ----------
			cmd = shlex.split(f'bash -c "{self.set_env} ; ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph"')
			cmd[-1] += ' "{filename: ' + self.ros_ws + '/src/pibot_nav/stb_map/map}"'
			try:
				_ = subprocess.Popen(cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE).wait(timeout=5)
			except subprocess.TimeoutExpired:
				self.query_one("#log").write_line(f'Request to save map has reached timeout, please try again later.')
				self.map_is_not_saved = True

			if hasattr(self, 'map_is_not_saved'):
				del self.map_is_not_saved
			else:
				# ------- Save map image for UI and keepout zones -------
				cmd = shlex.split(f'bash -c "{self.set_env} ; ros2 run nav2_map_server map_saver_cli -f {self.ros_ws}/src/pibot_nav/map/map --fmt png"')
				proc = subprocess.Popen(cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
				while proc.poll() is None:
					readables, _, _ = select([proc.stderr, proc.stdout], [], [], 0.2)
					if readables != []:
						log = readables[0].readline().decode('utf-8')
						if log.find('Map saved successfully') != -1:
							self.query_one("#log").write_line(f'Map saved successfully.')
							break

				self.query_one("#log").write_line(f'Opening map tool...')
				map_tool_path = self.ros_ws.rsplit('ws', 1)[0] + 'simple_ui/map_tool/'
				cmd = shlex.split(f'bash -c "{self.set_env} ; cd {map_tool_path} ; python3 start.py"')
				_ = subprocess.Popen(cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
			del self.is_saving_map

	def action_toggle_launch_ui(self) -> None:
		self.launchUI()

	@work(exclusive=True, thread=True)
	def launchUI(self):
		self.query_one("#log").write_line(f'Opening UI...')
		ui_path = str(__file__).split('executables')[0] + 'simple_ui/'
		cmd = shlex.split(f'bash -c "{self.set_env} ; cd {ui_path} ; python3 start.py"')
		_ = subprocess.Popen(cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE)

	def launchROS2(self, is_slam_stage, is_remote=False):
		if not is_remote:
			self.pop_screen()
			if is_slam_stage:
				self.bind('s', 'toggle_save_map', description='Save Map')
			else:
				self.bind('u', 'toggle_launch_ui', description='Launch UI')
		self.opened_processes = []
		self.query_one("#log").border_title = 'Log'
		self._launchROS2(is_slam_stage, is_remote)

	@work(exclusive=True, thread=True)
	def _launchROS2(self, is_slam_stage, is_remote=False):
		self.running = True

		self.setupDiscoveryServer()
		self.checkPackage()
		self.execStep1(is_slam_stage, is_remote)

		waiting_nav2_stack = self.nav2_stack.copy()
		ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

		while self.running:
			if not hasattr(self, 'proc_step2'):
				readables, _, _ = select([self.proc_step1.stderr, self.proc_step1.stdout], [], [], 0.2)
				for readable in readables:
					log = str(readable.readline(), encoding='utf-8').split('\r')[0].split('\n')[0]
					if log.find('Registering sensor') != -1:
						self.query_one("#log").write_line('Finished setting up robot and localization, launching Nav2...')
						self.opened_processes += ['lifecycle_manager', 'collision_avoidance']
						self.opened_processes += self.nav2_stack
						self.execStep2()
					elif log.find('camera calibration') != -1:
						pass
					elif log.find('[ERROR]') != -1:
						if log.find('rplidar_node') != -1:
							err_msg = self.formatLog(log)
							self.query_one("#log").write_line(f'Failed to launch slam_toolbox, please try to re-plug lidar then re-launch. Error message: {err_msg}')
						elif log.find('rviz') == -1:
							self.query_one("#log").write_line(f'{self.formatLog(log)}')
					elif log.find('Fatal error') != -1:
						err_msg = self.formatLog(log)
						self.query_one("#log").write_line(f'Failed to load devices, please check the status of devices then re-launch. Error message: {err_msg}')
			else:
				readables, _, _ = select([self.proc_step1.stderr, self.proc_step1.stdout, self.proc_step2.stderr, self.proc_step2.stdout], [], [], 0.2)
				for readable in readables:
					log = str(readable.readline(), encoding='utf-8').split('\r')[0].split('\n')[0]
					log = ansi_escape.sub('', log)
					if log.find('lifecycle_manager') != -1 and log.find('Activating') != -1:
						serv = log.split(' ')[-1]
						if serv in waiting_nav2_stack:
							waiting_nav2_stack.remove(serv)
						if len(waiting_nav2_stack) == 0:
							self.query_one("#log").write_line('Nav2 is available.')
							self.no_traj_count = 0
					elif log.find('Aborting bringup') != -1 or log.find('client will not receive response') != -1 or log.find('Shutting down') != -1:
						err_msg = self.formatLog(log)
						self.query_one("#log").write_line(f'Failed to launch Nav2, retrying... Error message: {err_msg}')
						# -------- Clean failed Nav2 process --------
						self.proc_step2.terminate()
						self.proc_step2.wait()
						for proc in self.nav2_stack + ['lifecycle_manager', 'collision_avoidance']:
							_ = subprocess.Popen(f'pgrep {proc[:15]} | xargs kill -9 2> /dev/null', stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
						_ = subprocess.Popen(shlex.split('bash -c "fastdds shm clean"'), stderr=subprocess.PIPE, stdout=subprocess.PIPE).wait()
						# -------- Re-launch Nav2 stack --------
						waiting_nav2_stack = self.nav2_stack.copy()
						self.execStep2()
					elif log.find('Passing new path to controller') != -1:
						self.no_traj_count = 0
					elif log.find('[ERROR]') != -1:
						if log.find('rviz') == -1 and log.find('camera calibration') == -1:
							err_msg = self.formatLog(log)
							if re.search("[Ee]xtrapolation", log) is not None:
								splitted = err_msg.split('time ')
								t1 = float(splitted[1].split(' ')[0].split(',')[0])
								t2 = float(splitted[1].split(' ')[0].split(',')[0])
								if abs(t1 - t2) > 2:
									self.query_one("#log").write_line(f'{err_msg}')
							elif err_msg.find('No valid trajectories') != -1:
								if self.no_traj_count < 3:
									self.no_traj_count += 1
								elif self.no_traj_count >= 3:
									self.query_one("#log").write_line('The AMR is probably stuck in an invalid area, please try to use teleop.')
							elif err_msg.find('Failed to get result for compute_path_to_pose') != -1:
								self.query_one("#log").write_line(f'Localization might failed.')
							else:
								self.query_one("#log").write_line(f'{err_msg}')

	def setupDiscoveryServer(self):
		cmd = shlex.split('bash -c "hostname -I | grep -o "')
		cmd[-1] += '"192.168.[0-9.]\\+"'
		proc = subprocess.Popen(cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
		ip_addr = proc.stdout.readline().decode('utf-8').split(' ')[0].split('\n')[0]
		self.set_env += f' ; export ROS_DISCOVERY_SERVER={ip_addr}:11811'
		_ = subprocess.Popen(shlex.split('bash -c "ros2 daemon stop"'), stderr=subprocess.PIPE, stdout=subprocess.PIPE)
		_ = subprocess.Popen(shlex.split('bash -c "fastdds shm clean"'), stderr=subprocess.PIPE, stdout=subprocess.PIPE)
		self.dds_proc = subprocess.Popen(shlex.split(f'bash -c "fastdds discovery -i 0 -l {ip_addr} -p 11811"'), stderr=subprocess.PIPE, stdout=subprocess.PIPE)
		while self.dds_proc.poll() is None:
			readables, _, _ = select([self.dds_proc.stderr, self.dds_proc.stdout], [], [], 0.2)
			if readables != []:
				log = readables[0].readline().decode('utf-8')
				if log.find('Server is running') != -1:
					self.query_one("#log").write_line(f'FastDDS discovery server is running on {ip_addr}:11811')
					break
		self.opened_processes.append('fast-discovery')

	def checkPackage(self):
		proc = subprocess.Popen(shlex.split(f'bash -c "ls {self.ros_ws}"'), stderr=subprocess.PIPE, stdout=subprocess.PIPE)
		list_ws = proc.stdout.read().decode('utf-8').split('\n')
		if 'build' not in list_ws or 'install' not in list_ws:
			self.query_one("#log").write_line(f'Building ROS2 package...')
			_ = subprocess.Popen(shlex.split(f'bash -c "source {self.ros_ws}/make_amr.bash"'), stderr=subprocess.PIPE, stdout=subprocess.PIPE).wait()

	def execStep1(self, is_slam_stage, is_remote):
		if is_slam_stage:
			step1_cmd = shlex.split(f'bash -c "{self.set_env} ; ros2 launch pibot_nav slam_setup.launch.py"')
			self.opened_processes += ['robot_state', 'navigator_cli', 'async_slam', 'rviz', 'gazebo', 'gzserver', 'gzclient']
			self.step2_cmd = shlex.split(f'bash -c "{self.set_env} ; ros2 launch pibot_nav slam_with_nav2.launch.py"')
			self.nav2_stack = ['bt_navigator', 'behavior_server', 'controller_server', 'planner_server', 'collision_detector']
		else:
			step1_cmd = shlex.split(f'bash -c "{self.set_env} ; ros2 launch pibot_nav robot_setup.launch.py pose_graph_path:={self.ros_ws}/src/pibot_nav/stb_map/map"')
			if not is_remote:
				step1_cmd[-1] += ' use_rviz:=True'
			self.opened_processes += ['robot_state', 'navigator_cli', 'localization', 'rviz', 'gazebo', 'gzserver', 'gzclient']
			self.step2_cmd = shlex.split(f'bash -c "{self.set_env} ; ros2 launch pibot_nav navigation.launch.py keepout_map_path:={self.ros_ws}/src/pibot_nav/map/keepout.yaml"')
			self.nav2_stack = ['bt_navigator', 'behavior_server', 'controller_server', 'planner_server', 'collision_detector', 'map_server', 'costmap_filter_info_server']
		self.proc_step1 = subprocess.Popen(step1_cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE)

	def execStep2(self):
		self.proc_step2 = subprocess.Popen(self.step2_cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE)

	def formatLog(self, log):
		re_search = re.search("\[[\-_a-zA-Z0-9]+\]: .*", log)
		if re_search is not None:
			return log[re_search.start():]
		sep = log.split(': ', 1)
		if len(sep) > 1:
			return sep[1]
		else:
			return log

if __name__ == "__main__":
	app = ROS2HelperApp()
	app.run()
