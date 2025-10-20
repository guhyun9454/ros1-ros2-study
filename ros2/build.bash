cd ~/colcon_ws
rm -rf build install log
colcon build
source install/setup.bash