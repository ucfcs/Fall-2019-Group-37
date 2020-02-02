# Fall-2019-Group-37
It looks like after installing and setting up the basic Jetson Nano image, I downloaded Arduino 1.8.10 using the browser and extracted it to `Downloads/arduino-1.8.10-linuxaarch64/arduino-1.8.10/`. From there, my .bash_history is basically:

```bash
cd Downloads/arduino-1.8.10-linuxaarch64/arduino-1.8.10/
sudo ./install.sh
cd ~
sudo apt update
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install ros-melodic-rosserial-arduino
sudo apt-get install ros-melodic-rosserial
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
cd src
git clone https://github.com/ros-drivers/rosserial.git
cd ..
catkin_make
catkin_make install
cd ~/catkin_ws/src/
git clone https://github.com/robopeak/rplidar_ros.git
cd ..
catkin_make
cd ~/catkin_ws/src/
git clone --branch catkin https://github.com/ucfcs/Fall-2019-Group-37.git egoat
cd ..
catkin_make
cd ~/Arduino/libraries
rosrun rosserial_client make_libraries .
```
Note that `echo $ROS_PACKAGE_PATH` at this point should include `home/[username]/catkin_ws/src` and `/opt/ros/melodic/share`
This should work as long as your Arduino sketchbook location in the GUI IDE is set to `~/Arduino`

After all that, download the Arduino file from GitHub here:
```
cd ~/Arduino
git clone --branch Arduino https://github.com/ucfcs/Fall-2019-Group-37.git egoat
```
Open it in the IDE and compile.
