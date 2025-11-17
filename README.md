# Setting up the SummitXL simulation in Windows
- This guide should get you up and running with the ROS Docker environment
- In the end, is my suggestion of what to check up on regarding ROS, and some of the packages SummitXL uses
### Installing Docker Desktop for Windows and an X server
Docker Desktop can be downloaded through this link: https://docs.docker.com/desktop/setup/install/windows-install/

Open windows command prompts (CMD): press windows key and type `cmd`

Update linux subsystems for windows:
```sh
wsl --update
```

Install VcXserv from: https://vcxsrv.com/

Remember to start the X server by launching `XLaunch` in windows, before starting the docker container.

### Clone git repository
If you dont have a gitHub account go set that up: https://github.com/

Open windows command prompts (CMD): press windows key and type `cmd`

Navigate to the directory you want to clone the git repo to, example

```sh
cd Documents
```
Then clone this repo:
```sh
git clone https://github.com/BurningSegway/P7_V2.git
```
### Build Docker container only done once, unless you edit Dockerfile
**Note, you need around 10 GB to build the container**

First navigate to the P7_V2 directory:
```sh
cd P7_V2
```
Build docker container:
```sh
docker build -t summit -f docker/Dockerfile .
```
This takes some time to complete.

### Run the Docker container
Navigate to the P7 directory, here you find the script `run_summit_sim.bat`
Open your favorite code editor like VScode, then edit line 42 to match your own path, which can be seen in the cmd if you have cd'ed into the P7 directory.

Run the script to start the Docker container in cmd:

```sh
run_summit_sim.bat
```
Make sure you run the file with .bat extension and not .sh. bat is for windows and sh is for linux.
The container should now start

**THE FOLLOWING STEPS SHOULD ONLY BE DONE ONCE**

- First time starting the container, the directory `SafeReinforcementLearning` should appear in the P7 directory. Copy the folder `src` into `SafeReinforcementLearning`.
- Build the ROS environment, make sure you are in the root of the ROS workspace, the command prompt path should be `/home/ros_workspace` (defualt directory when entering the container):
  ```sh
  catkin_make
  ```
- You also need to build some of the drone related stuff.
  In the root of the ros workspace clone the autopilot library
  ```sh
  git clone https://github.com/PX4/PX4-Autopilot.git
  ```
  Once finished continue to build
  ```sh
  cd PX4-Autopilot
  ```
  ```sh
  make px4_sitl_default gazebo
  ```
  Once the simulation has started and you see a drone in gazebo just close it agin by pressing ctrl+C in the terminal.

Every time you start a container, or enter a running container, you must source the ROS workspace from the root of the ROS workspace:
```sh
source devel/setup.bash
```
### Notes after running the container for the first time
After the container has been started for the first time and you have built the ROS workspace, here are some notes on running the containers and ROS.
- Make sure the X server is running on windows: press windows key and search `XLaunch`
- Once the container has been started, multiple terminals can enter the container
- You can check to see if the container is already running:
  ```sh
  docker ps
  ```
  If the container is already running you can enter it by:
  ```sh
  docker exec -it summit bash
  ```
- remember to source the ROS workspace every time you enter the container:
  ```sh
  source devel/setup.bash
  ```
- You can develop your own ROS packages to include, by adding them to the `src` directory from the root of the ROS workspace https://wiki.ros.org/ROS/Tutorials/CreatingPackage. Remember to build the workspace when adding new things again from the root of the workspace: `catkin_make` and source `source devel/setup.bash`

### Trying out the simulation
To run the simulation of the summit and get the data visualized in Rviz run:
```sh
roslaunch summit_xl_sim_bringup summit_xl_complete_test.launch
```
This will bringup a configuration where you can map the environment shown in Gazebo. 

To control the robot, you need to launch a teleop node in another terminal:
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot/cmd_vel
```
The node will instruct you how to generate velocity, and you can see the result in Gazebo and Rviz.
In another terminal launch the autopilet:
```sh
cd PX4-Autopilot
```
```sh
make px4_sitl_default none
```
In another terminal launch the drone by:
```sh
roslaunch x500_py x500.launch
```
Once the drone has spawned in and the PX4 terminal says it has connected and says ready for takeoff, then continue.
Then in another terminal you will need to get the localization up and running:
roslaunch mapping multi_map_toolbox.launch
```sh
roslaunch mapping multi_map_toolbox.launch
```

### For drone developement
Now the drone is launched an ready to fly.
Look in src/offboard_py/scripts for a node called offb_node.py, this is an example of velocity control of the drone in the drone frame. You may need to modify it using position commands, which it is already set up for, to get it in the air easily. You need to make a controller that outputs velocity commands to the topic /mavros/setpoint_velocity/cmd_vel note that these are in the world frame, unless transformed like the script does.
ROS works nicely with frames to describe where everything is in relation to eachother. The frames contains information about pose, which can be extracted and used, I recommend following this tutorial for extracting frame info: https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29
You can look up the frames and their relation by typing the following in a terminal:
```sh
rosrun tf view_frames
```
This will generate a PDF with the frames visualised. For the drone controller you might want to use the robot position, which is encoded in the frame robot_base_link.
Start by developing the controller that can make the drone fly in zig zag, and then start working on the CBF.

You should also develop this in your own package that is added to the workspace.
For QP optimization one could use cvxopt:
```sh
curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py
python2 get-pip.py
python2 -m pip install cvxopt==1.2.0
```
### ROS reading links
You can follow the tutorial if you want, and apply the commands and tools in the summit container to explore around, dont necesarily install the tutorial package. Tutorial link: https://wiki.ros.org/ROS/Tutorials

For most of ROS tools to work, the roscore must be started, but for a more fun and interactive experience start the simulation of summit. Note that roscore is automatically launched when running the simulation script. 

Must read links:
- Configuring ROS environment (you have done this in this guide): https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
- Understanding nodes: https://wiki.ros.org/ROS/Tutorials/UnderstandingNodes
- Understanding topics: https://wiki.ros.org/ROS/Tutorials/UnderstandingTopics
- Understanding Services: https://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams
- RQT console and roslaunch: https://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch

Important packages used by the summit - provided links, and perhaps you might want to look up youtube videos as well:
- Navigation stack: https://wiki.ros.org/navigation
- gmapping: https://wiki.ros.org/gmapping
- move_base: https://wiki.ros.org/move_base

This process, and developing in general is much easier in ubuntu in my opinion, but not as user friendly. If you want in the future to look into using ubuntu, you have the option of dual booting your PC, this will enable you to choose at startup, to boot into windows or ubuntu. Here is a link for a guide (or google yourselfes especially for which version of ubuntu): https://www.instructables.com/Dual-Booting-Windows-and-Ubuntu/

#
Written by Pierre ROB7 161
#







