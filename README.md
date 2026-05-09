# Simulation

## About the project 
The aim of this project is to simulate the prosthesis using a physics engine that allows real-time control via electromyography (EMG), electroencephalography signals (EEG) and computer vision (CV). The simulation is based on ROS2 and Gazebo and includes a model of the prosthesis, as well as tools for visualizing and controlling the prosthesis in real-time.
<img width="656" height="378" alt="Screenshot from 2026-05-10 00-24-41" src="https://github.com/user-attachments/assets/f0265cb8-bad2-442c-becc-10d9b63f78e6" />

## Structure of the repository 

The repository is organized into several directories, each containing specific components of the project:
- `src/`: Contains the source code for the ROS2 nodes, including control algorithms, sensor data processing, and communication interfaces.
- `launch/`: Contains ROS2 launch files for starting the simulation and related nodes.
- `model/`: Contains the URDF (Unified Robot Description Format) files that describe the physical structure and properties of the prosthesis.
- `meshes/`: Contains the 3D models of the prosthesis components used in the simulation.
- `config/`: Contains configuration files for ROS2 nodes, including parameters for control algorithms and sensor processing.
- `scripts/` : Contains executable scripts for controlling the prosthesis and processing sensor data.
- `worlds/`: Contains Gazebo world files for simulating the environment in which the prosthesis operates.

## Disclaimer 

This project is provided for research, educational, and experimental purposes only. It is **not** a medically approved device, system, or application, and it has not been evaluated, certified, or authorized by any regulatory body for clinical or diagnostic use.

By using, testing, or interacting with this project, you acknowledge and agree that any use is entirely at your own risk. The contributors and maintainers of this project make no representations or warranties of any kind regarding its safety, accuracy, reliability, or suitability for any purpose.

Under no circumstances shall the contributors, maintainers, or associated parties be held liable for any direct, indirect, incidental, or consequential damages, including but not limited to personal injury, health-related issues, or any other harm arising from the use or misuse of this project.

If you require medical advice, diagnosis, or treatment, consult a qualified healthcare professional. Do not rely on this project for medical decisions.

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

To move the prosthesis base (arm) with your keyboard open a new terminal and run : 
```
source install/setup.bash
ros2 run prosthesis_description keyboard_control.py
``` 
Then follow the printed instructions.


Or to move the prosthesis base (arm) with a gamepad controller open a new terminal and run :
```
source install/setup.bash
ros2 run joy joy_node --ros-args --params-file src/prosthesis_description/config/joy_node.yaml
```
Open another terminal and run :
```source install/setup.bash
ros2 run prosthesis_description gamepad_control.py
```
Then follow the printed instructions.


Or to move each joint of the prothesis with a GUI, open a new terminal and run :
```
source install/setup.bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```


To send back the resulting current on each finger and wrist on a dedicated topic, open a new terminal and run :
```
source install/setup.bash
ros2 run prosthesis_description sensor_data_extractor.py
```


## Contributions
We welcome all contributions. Please open an issue in the repo or fork, edit, and open a pull request. Please read section [CONTRIBUTING](profile/CONTRIBUTING.md).

## License
This project is licensed under [Mozilla Public Licence 2.0](https://choosealicense.com/licenses/mpl-2.0/) for code and [CC-BY-4.0](https://choosealicense.com/licenses/cc-by-4.0/) for hardware and documentation.

