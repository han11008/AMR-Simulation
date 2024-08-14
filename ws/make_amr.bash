WORK_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $WORK_DIR

source ~/slam_and_nav_ws/install/setup.bash
source ~/octo_ws/install/setup.bash
colcon build

echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> install/setup.bash

source install/setup.bash

