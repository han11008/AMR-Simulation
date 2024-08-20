from setuptools import find_packages, setup
#from Cython.Build import cythonize
from glob import glob
import os

package_name = 'pibot_nav'
#files = package_name + '/*.py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'param'), glob('param/*')),
        (os.path.join('share', package_name, 'bt'), glob('bt/*')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        (os.path.join('share', package_name, 'stb_map'), glob('stb_map/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
    ],
#    ext_modules=cythonize(files, compiler_directives={'language_level': '3'}, force=True, quiet=True),
#    install_requires=['setuptools', 'wheel',  'Cython'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='XXX',
    maintainer_email='XXX@gmail.com',
    description='Control AMR and visualization',
    license='Apache Licence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		f'odom_footprint_tf = {package_name}.odom_footprint_tf_pub:main',
		f'navigator_client = {package_name}.navigator_client:main',
		f'collision_avoidance = {package_name}.collision_avoidance:main',
		f'fleet_map_server = {package_name}.fleet_map_server:main',
		f'task_server = {package_name}.task_server:main',
		f'task_client = {package_name}.task_client:main',
		f'obstacle_map = {package_name}.obstacle_map:main',
        ],
    },
)


