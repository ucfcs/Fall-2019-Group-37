# Fall-2019-Group-37
To compile this, make sure you've already copied and built [the eGOAT catkin package](https://github.com/ucfcs/Fall-2019-Group-37/tree/catkin) (for the ROS message). To make the header files, refer to [Generating Message Header File](http://wiki.ros.org/rosserial/Tutorials/Adding%20Other%20Messages) on the rosserial Tutorial:

>To add new messages to the library you have to delete (if already created) the whole ros_lib folder in your arduino libraries subpath and generate all from scratch.
>```
>rosrun rosserial_client make_libraries path_to_libraries
>```
>For instance, suppose I need to generate headers for message definitions contained in my crazy_msgs package, and ros_lib is located at '~/Arduino/libraries/ros_lib':
>```
>rm -r ~/Arduino/libraries/ros_lib
>rosrun rosserial_client make_libraries ~/Arduino/libraries
>```

To test (typically 3 terminals):
```
roscore
```
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
```
rostopic pub motors egoat/SetMotorSpeed '{left_motor: 100, right_motor: 100}' --once
```

Replacing `/dev/ttyACM0` with the serial port of the Arduino device and using the desired values for the left and right motor speeds.
