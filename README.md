# 1. Requirements
Here is solution needed for using Universal Robot 5 robot arm in Gazebo simulation and on real arm.

**NB! If you are not familiar with ROS, then I recommend starting with ROS tutorials** [HERE](http://wiki.ros.org/ROS/Tutorials).

## For simulation
- [Ubuntu 16.04](http://releases.ubuntu.com/16.04/)
- [ROS (Robot Operating System) Kinetic](http://wiki.ros.org/kinetic)
- [Leap Motion controller](https://www.leapmotion.com/)
- [Leap Motion SDK v2](https://developer.leapmotion.com/sdk/v2)
- [leapd service](https://forums.leapmotion.com/t/tip-ubuntu-systemd-and-leapd/2118)
- [leap_motion](http://wiki.ros.org/leap_motion)
- [MoveIt!](http://moveit.ros.org/install/)
- [Universal Robot](http://wiki.ros.org/universal_robot)
- [jog_arm](https://github.com/ut-ims-robotics/jog_arm)

## For Universal Robot 5 robot arm

 - Previous simulation requirements
 - [ur_modern_driver](https://github.com/willcbaker/ur_modern_driver)
 - [UR5 robot arm](https://www.universal-robots.com/products/ur5-robot/)

# 2. Setup
 1. Create your catkin workspace. I use folder named ***catkin_ws***
 2. Go to catkin workspace *src* folder: ***cd ~/catkin_ws/src***
 3. Clone and install everything needed in previously given order
 4. leapd service needs to be created, so use "***sudo***" command.
 5. Clone solution to workspace: ***git clone https://github.com/martinhallist/leap_teleoperations.git***
 6. Go to catkin workspace: ***cd ~/catkin_ws/***
 7. Use command: ***catkin_make***

Now setup should be done.

# 3. Starting simulation

 1. Go to catkin workspace: ***cd ~/catkin_ws/***
 2. Source your code: ***source devel/setup.bash***
 3. Connect Leap Motion controller to computer's USB 3 port for better results. If device is detected, the control panel light will turn green. If it still doesn't work use command: ***sudo service leapd restart***
 4. In order to enable Leap SDK, open terminal and use command:
    ***sudo leapd***
 5. Next launch UR5 gazebo (starting everything together causes UR5 arm to fall): ***roslaunch ur_gazebo ur5.launch limited:=true***
 6. Next launch our launch file: ***roslaunch leap_teleoperations ur5_simu.launch***

Use your left hand to control robot manipulator.

# 4. Using real robot

 1. Go to catkin workspace: ***cd ~/catkin_ws/***
 2. Source your code: ***source devel/setup.bash***
 3. Connect Leap Motion controller to computer's USB 3 port for better results. If device is detected, the control panel light will turn green. If it still doesn't work use command: ***sudo service leapd restart***
 4. In order to enable Leap SDK, open terminal and use command: ***sudo leapd***
 5. Launch ur_modern_driver: ***roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=ROBOT_IP_ADDRESS*** where ROBOT_IP_ADDRESS is your robot network IP address.
 6. Next launch our launch file: ***roslaunch leap_teleoperations ur5_real.launch***

Use your left hand to control robot manipulator.

# 5. Common problems

 - **UR5 robot arm doesn't move**
 Check if control panel has green Leap Motion icon. If not, use command ***sudo service leapd restart***. It aslo may be caused by overheat and thus it won't send out signal for few seconds.
 - **UR5 arm is not properly attached to ground in simulation**
Wait for arm to appear in Gazebo, then move to next step
 - **Unable to find manipulator when controlling real robot**
Are you using same network as robot arm? Have you entered correct IP?
 - **Arm does almost full spin for some positions**
It's common problem with MoveIt! that jog_arm uses. Movements are restricted to [-?, ?], so it doesn't have 360 degrees movement freedom.
 - **error: ‘const struct hardware_interface::ControllerInfo’ has no member named ‘hardware_interface’ after catkin_make**
 Apply this fix: https://github.com/iron-ox/ur_modern_driver/commit/883070d0b6c0c32b78bb1ca7155b8f3a1ead416c
 - **Roslaunch does not find my files**
Make files executable.

# Experimental solution
There is also experimental source code for dual arm control, that doesn't work right now.
