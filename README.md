### A GUI for dual camera feeds and controlelr input

The following must be run as root due to xbox controller permissions
ensure your ROS variables and $PYTHONPATH are correctly configured for root.
also if not done at root bash script:

`source /opt/ros/hydro/setup.sh`

####To open, use: 
```
sudo su
./controllerGUI_pub.py
```
to load the camera feeds, the webcams must be plugged in directly to the USB ports of the laptop because the USB hub does not have enough throughput for two webcams.

Once the GUI has opened this should have activated xboxdrv to capture xbox controlelr commands.  you should see the picture of the xbox controller update as the joysticks are moved.
Next, you must connect these joystick events with ROS by running the 'joy_node' from the 'joy' pacakge:
(ensure the ROS master is already running)
in a new terminal, type:

`rosrun joy joy_node`

If the script returns a red error about unable to open '/dev/input/js*' you may need to change the rosparameters using the 'rosparam set ...' command.
Refer to the ROS joy package and tutorials on the wiki.

Finally, in order to achieve saved position states and rumble feedback, you must run the listener.
The listener is in the '~/catkin_ros_joy/src/scripts' directory

this too must be run as root:
```
cd ~/catkin_ros_joy/src/scripts
sudo su
source /opt/ros/hydro/setup.sh #this sources the ROS environment
./listener.py
```

