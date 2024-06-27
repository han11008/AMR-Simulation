from select import select
import subprocess, shlex, os

def main():
#	res = input('Update map configurations from the AMR? [y/n] ')
#	if res == 'y' or res == 'Y':
#		cmd = shlex.split('bash -c "scp bot@amr:~/amr_ws/simple_ui/resources/\{conf.json,map.png\} ~/amr_ws/simple_ui/resources/"')
#		_ = subprocess.Popen(cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE).wait()
#		cmd = shlex.split('bash -c "scp -r bot@amr:~/amr_ws/ws/src/pibot_nav/map/map.* ~/amr_ws/ws/src/pibot_nav/map/"')
#		_ = subprocess.Popen(cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE).wait()

#	proc = subprocess.Popen('getent hosts amr | grep -o "[0-9.]\+"', stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
	proc = subprocess.Popen('hostname -I | grep -o "192.168.[0-9.]\+"', stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
	dds_server = proc.communicate(timeout=1.0)[0].decode('utf-8').split('\n')[0] + ':11811'
	print(f'FastDDS discovery server on AMR: {dds_server}')
	env = os.environ
	env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
	env['ROS_DISCOVERY_SERVER'] = dds_server

	print('Opening UI...')
	ui_path = str(__file__).split('executables')[0] + 'simple_ui/'
	cmd = shlex.split(f'bash -c "cd {ui_path} ; python3 start.py"')
	_ = subprocess.Popen(cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE, env=env).wait()

if __name__ == "__main__":
	main()

