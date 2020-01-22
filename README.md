# Fall-2019-Group-37

This project requires `rosserial-arduino`. Install the binaries by running:

```
sudo apt-get install ros-melodic-rosserial-arduino
sudo apt-get install ros-melodic-rosserial
```

Then install the catkin code, where <ws> represents your catkin workspace:

```
cd <ws>/src
git clone https://github.com/ros-drivers/rosserial.git
cd <ws>
catkin_make
catkin_make install
```

Lastly, copy this code into your catkin workspace as well:

```
cd <ws>/src
git clone --branch catkin git@github.com:ucfcs/Fall-2019-Group-37.git egoat
cd <ws>
catkin_make
catkin_make install
```