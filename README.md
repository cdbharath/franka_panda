# Franka Panda - Pick and Place
Custom Franka Panda ROS packages for personal use

![alt text](./data/pick&place.gif "Pick and Place")

## How to run the simulation
Clone the repo in your ROS workspace and install libfranka library
```
sudo apt install ros-<distro>-libfranka
```
Run the following command after building and sourcing the workspace
```
roslaunch panda_simulation panda_simulation_effort.launch
```
## How to use pick and place package

```
from pick_and_place.pick_and_place import PickAndPlace

pick_and_place = PickAndPlace(0.05, 0.5)             # Arguments: gripper offset, intermediate vertical z stop
    
pick_and_place.setPickPose(0.4, 0.0, 0.1, 0, pi, 0)  # Arguments: x, y, z, roll, pitch, yaw
pick_and_place.setDropPose(0.0, 0.4, 0.4, 0, pi, 0)  # Arguments: x, y, z, roll, pitch, yaw
pick_and_place.setGripperPose(0.01, 0.01)            # Arguments: finger1 linear movement, finger2 linear movement  
    
pick_and_place.execute_pick_and_place()
```

## References

1. [Erdals Blog](https://erdalpekel.de/?p=55 "Erdals Blog")
2. [Grasp Plugin](https://github.com/JenniferBuehler/gazebo-pkgs "Grasp Plugin")
3. Eye in hand depth sensor reference code - Vision-based Robotic Manipulation course, Prof. Berk Calli, Worcester Polytechnic Institute
