set -e

colcon build
source ./install/setup.sh
conf='./conf'
ros2 launch simulator sim.launch.py $(<"$conf")
