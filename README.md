# Franka Panda - Pick and Place
Custom Franka Panda ROS packages for pick and place operations

![alt text](./data/pick&place.gif "Pick and Place")

## How to run the simulation
Clone the repo in your ROS workspace and install libfranka library 
```
mkdir -p panda_ws/src
cd panda_ws/src
git clone git@github.com:cdbharath/franka_panda.git
cd ..
sudo apt install ros-<distro>-libfranka
catkin build
source devel/setup.bash
```
Use the following commands as per the requirement after building and sourcing the workspace
```
roslaunch panda_simulation panda_simulation.launch          # For position based joint controllers 
roslaunch panda_simulation panda_simulation_effort.launch   # For effort based joint controllers
roslaunch panda_simulation panda_eye_in_hand.launch         # For effort based joint controllers with eye in hand camera
roslaunch pick_and_place kinect.launch                      # For overhead camera 
```
## How to use pick and place package

```
from pick_and_place.pick_and_place import PickAndPlace

pick_and_place = PickAndPlace(0.05, 0.5)             # Arguments: gripper z offset, intermediate vertical z stop
    
pick_and_place.setPickPose(0.4, 0.0, 0.1, 0, pi, 0)  # Arguments: x, y, z, roll, pitch, yaw 
pick_and_place.setDropPose(0.0, 0.4, 0.4, 0, pi, 0)  # Arguments: x, y, z, roll, pitch, yaw
pick_and_place.setGripperPose(0.01, 0.01)            # Arguments: finger1 linear movement, finger2 linear movement  
    
pick_and_place.execute_pick_and_place()              # For execution in joint space
pick_and_place.execute_cartesian_pick_and_place()    # For execution in cartesian space
```

## References

1. [Erdals Blog](https://erdalpekel.de/?p=55 "Erdals Blog")
2. [Grasp Plugin](https://github.com/JenniferBuehler/gazebo-pkgs "Grasp Plugin")
