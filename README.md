# Simulation
The aim of this project is to simulate the prosthesis using a physics engine that allows real-time control via electromyography (EMG), electroencephalography signals (EEG) and computer vision (CV). 

## Demonstration
### Rviz
Visualization of the prosthesis in the Rviz tool.
Moving joints via GUI.

<img width="1231" height="846" alt="image" src="https://github.com/user-attachments/assets/ab08f4cc-8ec9-4fe3-a6e3-a03b8954f456" />

### Gazebo
Visualization of the prosthesis in the Gazebo physics engine.

<img width="1847" height="1125" alt="image" src="https://github.com/user-attachments/assets/c1fac227-d56c-4e2a-bba0-c626e0a26529" />

## Requirements
- Operating system : Linux Ubuntu 24.04
- ROS2 Jazzy-jalisco version for robot control and communication
- Gazebo Harmonics for simulation
- ros_gz_bridge
- Xacro / URDF for robot description

## Usage
### Installation guide
1. Clone the repository
   ```
   git clone [https://github.com/](https://github.com/N-Pulse/ARM-SOFT-Simulation.git)
   cd ARM-SOFT-Simulation
   ```
3. Install dependencies using rosdep tool
   ```
   sudo apt-get install python3-rosdep
   sudo rosdep init
   rosdep update
   rosdep install -i --from-path src --rosdistro jazzy -y
   ``` 
   
### User guide
#### Rviz
To visualize the prosthesis in Rviz tool :
```
colcon build
source install/setup.bash
ros2 launch prosthesis_description rviz.launch.py
```
#### Gazebo
To sart the simulation in Gazebo go into ros2_ws and run :
```
colcon build
source install/setup.bash
ros2 launch prosthesis_description gazebo.launch.py
```
To control the prosthesis trajectory open a new terminal and run :
```
source install/setup.bash
ros2 run prosthesis_description trajectory_control.py
```

To move the prosthesis base (arm) with your keyboard open a new terminal and run : 
```
source install/setup.bash
ros2 run prosthesis_description keyboard_control.py
``` 
Then follow the printed instructions.


To move the prosthesis base (arm) with a gamepad controller open a new terminal and run :
```
ros2 run joy joy_node --ros-args --params-file src/prosthesis_description/config/joy_node.yaml
```
Then follow the printed instructions.


To move each joint of the prothesis with a GUI, open a new terminal and run :
```
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```


## License 
