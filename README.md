[![Build Status](https://travis-ci.org/LGD-Fr/hand_control.svg)](https://travis-ci.org/LGD-Fr/hand_control)

Copyright © 2015 CentraleSupélec

Permission is granted to copy, distribute and/or modify this document
under the terms of the GNU Free Documentation License, Version 1.3
or any later version published by the Free Software Foundation;
with no Invariant Sections, no Front-Cover Texts, and no Back-Cover Texts.


La documentation originale est en français : LISEZMOI.md. La documentation ci-dessous n’est qu’une traduction.

The original manual is the french "LISEZMOI.md", you read below a translation of this file.

# Video demonstration #

A video demonstration is [available here](https://archive.org/details/hand_control). It has been achived with a student who didn't know how the project works.

# Installation #

This package was developped with the Indigo version of ROS.

## Dependencies installation ##
```
#!sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full ros-indigo-freenect-stack ros-indigo-ardrone-autonomy libncursesw5-dev
```
## Package installation ##

### Catkin workspace creation ###

For instance :

```
#!sh
source /opt/ros/indigo/setup.bash
mkdir -p ~/hand_control_ws/src
cd ~/hand_control_ws/src
catkin_init_workspace
```

### Code location ###

If necessary, rename the folder with the file named `hand_control`, and move it in `~/hand_control_ws/src/` or in the subfolder `src` of your catkin workspace.

## Compilation ##

You're now able to compile :

```
#!sh
cd ~/hand_control_ws # or your catkin workspace
catkin_make
```

Then you can run the following commands to be able to use the ROS commands. If necessary replace "hand_control_ws" by the name of your catkin workspace.

```
#!sh
source /opt/ros/indigo/setup.bash
source ~/hand_control_ws/devel/setup.bash
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
echo "source ~/hand_control_ws/devel/setup.bash" >> ~/.bashrc
```

# Use #

## Connection and setting up of the Kinect ##

1. Connect the Kinect (under voltage) to the computer via USB ;
2. Put the Kinect on the ground, pointed toward the roof ; be aware that your arm must be perpendicular to the Kinect in order to control the drone properly ;
2. Launch the "launchfile" kinect_commander.launch : `roslaunch hand_control kinect_commander.launch` ;
3. Check the drone parameters :
    - launch rviz :  `rosrun rqt_rviz rqt_rviz`
    - display the output of the filtering (topic : `/filter/output` ; frame : `/camera_depth_optical_frame`) and locate the hand ;
    - launch rqt_reconfigure : `rosrun rqt_reconfigure rqt_reconfigure` in order to :
      - change the filter parameters until you only see the points of the hand/glove/panel on rviz (see above).
      - change the parameter `neutral_alt` of the node `commander` to the wanted height (in meters), correponding to the height of the hand for which the height of the drone will be stable.
    
### Parameters of the filter ###

The parameters of the filter (that can be changed thanks to `dynamic_reconfigure` and in particular `rqt_reconfigure`) are :

* `z_max` : in meters, maximal height of the hand. It must be lower than the height of the roof.
* for a glove or a *colored* panel (green, blue, etc.), we generaly have :
    - `hue` : for example 220 (sky blue) or 150 (green) or 0 (pink/red) ;
    - `delta_hue` : between 10 and 20 ;
    - `sat/val_min` : 0.0 ;
    - `sat/val_max` : 1.0 ;
* for a *black* glove :
    - `hue` : 0 ;
    - `delta_hue` : 180 ;
    - `sat_min` : 0.0 ;
    - `sat_max` : 1.0 ;
    - `val_min` : 0.0 ;
    - `val_max` : 0.3 (at your convenience);

### Other parameters ###

Always with `rqt_reconfigure`, but with the `estimator` node :
- `reverse` : swap x and y, the axes of the Kinect (default : false, ie. unchecked)
- `reverse_angle` : change the angle choosen for the compute of the angle of the hand (default : false, ie. unchecked)

## Connection to the drone and steering ##

* Connect the computer to the wifi network of the drone ;
* Launch the "launchfile" ardrone.launch : `roslaunch hand_control ardrone.launch` ;
* Taking off : 
    - whether `rostopic pub /ardrone/takeoff std_msgs/Empty` ;
    - or launch the node keyboard_cmd : `rosrun hand_control keyboard_cmd` and use *t* on the keyboard.
* Landing :
    - whether `rostopic pub /ardrone/land std_msgs/Empty` ;
    - or, launch the node keyboard_cmd, and use *b* on the keyboard.
* Emergency stop :
    - whether `rostopic pub /ardrone/reset std_msgs/Empty` ;
    - or, launch the node keyboard_cmd, and use *g* on the keyboard.

### Hand steering ###

* Forward/backward & side translations : hand tilt ;
* Rotate (around the vertical axis z) : angle of the hand with the the axis parallel to the ground and perpendicular to the kinect ;
* go up/go down : hand height.

### Options and parameters of the command ###

To edit the options of the command, change (if not already) `rosrun rqt_reconfigure rqt_reconfigure` :

- `max_curvature` : not used for the moment ;
- `x/y/z/theta_minimal_deviation` : thresholds required above which the movement of the hand is not taken into account. If all are 0.0, the drone responds linearly.
    * x, y : between 0. and 1. (corresponding to the x and y of the normal to the plane);
    * z : in meters ;
    * theta : in degrees.
- `neutral_alt` : height of the hand for the immobility of the height of the drone ;
- `min_points_number` : minimal number of points (for the point cloud used for the regression) necessary in order to send a command to the drone ;
- `angle/x/y/z_vel` : proportionality coefficients to apply to the inputs in order to establish the command sent to the drone. Increase it will increase the speed of the drone ;
- `up_fact` : proportionality coefficients to apply to the command that increases the height of the drone, compared to the equivalent command to reduce it (in order to correct the effect of gravity).

### About `keyboard_cmd` ###

It allows you to publish commands on the topic `cmd_vel` and so to steer the drone. It is scheduled for azerty keyboards. To launch it, run :

```
#!sh
rosrun hand_control keyboard_cmd
```

To increase/decrease the speed (there is an explication on the controlpanel) : a,z,e,r and w,x,c,v

The informations of the drone are updated when a key is pressed.

To quit : CTRL+C and press "Enter" to return to the console.

# Problems - Possible improvements #

- If commands are published on `cmd_vel` (from the Kinect for instance) after the launch of `ardrone.launch` and before the takeoff, then, after the takeoff, the drone seems to obey to commands published before the takeoff.

- As written above, the display of navigation data on `keyboard_cmd` is only updated when a key is pressed, and can therefore stay fixed when the keyboard commander is not used.

- The takeoff/landing is not controllable with the hand. The keyboard must be used (`keyboard_cmd` or `rostopic pub`) instead. We can correct this by creating two new thresholds, minimal and maximal, for the hand height : a very low hand would make the drone land and a very high hand would make the drone take off.
