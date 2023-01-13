# hera_control
 This package is a implementation of the HERA control system.

# Dependencies:
 * [hera_description](https://github.com/Home-Environment-Robot-Assistant/hera_description)

# How to use this repository
1. Follow instruction on [how to use this repository](https://github.com/Home-Environment-Robot-Assistant/hera_description#how-to-use-this-repository) of the [hera_description](https://github.com/Home-Environment-Robot-Assistant/hera_description) package.
2. Git clone this repository and install all dependencies.
```bash
cd src
git clone https://github.com/Home-Environment-Robot-Assistant/hera_control.git
sudo ./hera_control/install_dependencies.sh
```
3. Compile you catkin workspace.
```bash
cd <catkin_workspace>/
catkin_make
source devel/setup.bash
```
4. Now you are ready to use the [rosrun](http://wiki.ros.org/rosbash#rosrun) to start the simulated control.

```bash
rosrun  hera_control gazebo_control.py
```

Them, try to send a command by [rostopic](http://wiki.ros.org/rostopic):
```bash
rostopic pub --once /control/head std_msgs/Float64 "data: 0.5"
```

Use rostopic list to see available controls:
```bash
rostopic list /control/
```
