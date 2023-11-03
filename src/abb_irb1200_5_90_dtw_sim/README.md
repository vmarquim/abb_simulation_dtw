# abb_irb1200_simulation_dtw
Gazebo Simulation with Moveit Controls in Rviz and DTW Plot comparing simulated and theoretical trajectory

1. Install ROS Noetic [Tutorial here](https://wiki.ros.org/noetic/Installation/Ubuntu)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
```

2. Install Moveit [Tutorial here](https://moveit.ros.org/install/)
```
sudo apt install ros-noetic-moveit
```

3. Install Pandas, NumPy and DTW
```
pip install pandas dtw-python numpy
```

4. To start the Robot Simulation in Gazebo and the RViz with Moveit, Run the simulation launch file
```
roslaunch abb_irb1200_simulation_dtw simulation_dtw_program.launch
```

5. Run the run and compare launch file
```
roslaunch abb_irb1200_simulation_dtw run_and_compare.launch run_mode:=[run_mode]
```

6. Possible values for run_mode (default is run_mode:="square")
```
"square"
"circle"
"line"
"random-valid"
```
