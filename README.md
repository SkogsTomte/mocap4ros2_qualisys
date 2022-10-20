# mocap4ros_qualisys

## Installation

### Normal

Requisites: qualisys_cpp_skd

- Just clone recurseivelly this repo to get https://github.com/qualisys/qualisys_cpp_sdk, or set QualisysSDK_PATH
- To clone recurseivelly:
```
git clone --recurse-submodules git@github.com:SkogsTomte/mocap4ros2_qualisys.git
```
- If you build qualisys_cpp_sdk in your workspace, the first time you build the workspace, exclude qualisys_driver:
```
colcon build --symlink-install --packages-skip qualisys_driver
colcon build --symlink-install
```

### With Docker
- Install docker engine
- Download the file dockerstuff.zip
- Extract the files
- To make the scripts executable:
```
chmod +x build.sh
chmod +x run.sh
```
- The path given in run.sh should be the path to usb-devices to use in the container.
- Build the image:
```
./build.sh
```
- To create an run a bash terminal in a container:
```
./run.sh
```
- To open more terminal windows for the same container, open a new terminal and run:
``` sudo docker ps ```
- Check the id of the container and then run:
```
sudo docker exec -it <CONTAINER_ID> bash
```
where you replace <CONTAINER_ID> with the id of your container.

## Usage
- Open qualisys and define your rigid bodies.
- Source you ros environment
```
source <ROS_WS_DIR>/install/setup.bash
```
Where of course you replace <ROS_WS_DIR> with the path to your workspace, which would be /root/ros2_ws if you use docker

- In the file ros2_ws/src/mocap4ros2_qualisys/qualisys_driver/config/qualisys_driver_params.yaml make sure the right parameters are choosen. All parameters except maybe the ip (host_name) can probably be left on default.
- Launch the node:
```
ros2 launch qualisys_driver qualisys.launch.py
```
- If connected properly, you should be able to see the rigid bodies being published to /rigid_bodies
```
ros2 topic echo /rigid_bodies --no-daemon
```
(for some reason the --no-daemon option needs to be added while the qualisys node is running)
