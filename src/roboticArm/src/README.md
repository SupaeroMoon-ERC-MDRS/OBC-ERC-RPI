
# Robotic Arm program src

Source files of the Robotic arm. At the moment:
- rover_arm_description : Package used to create and interact with the URDF/Xacro files
- rover_arm_moveit_auto_config : Package generated automaticaly by MoveIt assistant. Used to simulate the robitc arm with the possibility to edit position and plan path directly inside an Rviz interface.
- hello_moveit : Package that was used to control the robotic arm by giving a position. Then MoveIt plan the path and execute it. But no longer works after the implementation of the end effector part on the arm, currently in the process of correcting this. The package was constructed following MoveIt tutorial.

## How to install

You need to build packages by running the following command
```
colcon build --packages-select rover_arm_description rover_arm_moveit_auto_config rover_arm_control rover_arm_cpp_control --mixin debug
```
After building, package are accessible through 
```
ros2 run|launch <package_name> <script_name>
```

You just need to don't forget to source
```
source install/setup.bash
```

## What to start

#### rover_arm_description

To see the URDF inside Rviz, execute the following command
```
ros2 launch rover_arm_description test.launch.py
```
After executing this, you should see two windows open. One allows you to control joints rotation, and the other allows you to view the model position.

#### rover_arm_moveit_auto_config

To start the simulation, execute the following command inside a first terminal
```
ros2 launch rover_arm_moveit_auto_config demo.launch.py
```
Inside a second terminal, use this command
```
ros2 launch rover_arm_moveit_auto_config move_group.launch.py
```

After this, you sould have an Rivz windows that appear. Inside, the robotic arm should be visible, with a panel named "MotionPlanning" on the rigth.

Inside this panel, you should be able to use the **plan** tab in order to plan a movement, that can be defined by selecting the right panning group (either end_effector or panda_arm) and move the joints inside the **Joints** tab.

When you have a position, just use **plan & execute** button from the **plan** tab to plan and execute the trajectory.

#### hello_moveit

Still solving the error. Will write the documentation when it will be finished. Plus, this folder was used as base and a more appropriate one will be put in place once the problem has been solved.


## To do

- Solve the **Unable to sample any valid states for goal tree** error, then push the rover_arm_controller package (will replace hello_moveit package).
- Write the part related previous package.
- Push the package related to servo motors command execution.
- Write a proper explanation of node interaction and scripts
- Add pictures