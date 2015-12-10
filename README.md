# cyton_alpha_pick_place
cyton_alpha package to grasp objects using MoveIt! and moveit_simple_grasps

To gripper configuration see config/cyton_grasp_data.yaml 
(more information: https://github.com/davetcoleman/moveit_simple_grasps/tree/hydro-devel)

## Setup

Packages required:

- Moveit
```bash
  $ sudo apt-get install ros-hydro-moveit-full
  $ sudo apt-get install ros-hydro-moveit-core
```

- cyton_alpha
```bash
cd ~/catkin_ws/src
git clone https://github.com/LCAD-UFES/cyton_alpha.git
```


- moveit_simple_grasps

Install From Source

Clone this repository into a catkin workspace, then use the rosdep install tool to automatically download its dependencies. (this steps are for Ros Hydro use:

```bash
cd ~/catkin_ws/src
git clone https://github.com/davetcoleman/moveit_simple_grasps.git -b hydro-devel

source /opt/ros/$(rosversion -d)/setup.bash
rosdep install --from-paths src -iy
source devel/setup.bash 
rosdep install --from-paths src --ignore-src --rosdistro hydro
catkin_make -j4
```


## Demo mode

Run the following instead:

``` bash
roslaunch cyton_alpha_moveit_config demo.launch
roslaunch cyton_alpha_pick_place grasp_generator_server.launch
rosrun cyton_alpha_pick_place pick_lego.py
```

## Simulation mode

```bash
roslaunch cyton_alpha_gazebo cyton_empty_world.launch
roslaunch cyton_alpha_moveit_config moveit_rviz.launch config:=true
roslaunch cyton_alpha_pick_place grasp_generator_server.launch
rosrun cyton_alpha_pick_place pick_lego.py

```	

