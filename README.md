# 环视仿真

---

## 安装

### 0. Dependencies

ROS Kinetic(on Ubuntu 16.04)

### 1. Datasped ADAS Kit Gazebo

```bash
bash <(wget -q -O - https://bitbucket.org/DataspeedInc/ros_binaries/raw/default/scripts/setup.bash)
sudo apt-get install ros-$ROS_DISTRO-dbw-mkz-simulator
```

### 2. Upgrade Gazebo

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo7
```

备注：ROS Kinetic 默认已经安装Gazebo7.0，但该版本有一些bug，建议升级到最新的7.13

## 编译&运行

```bash
cd surround_view_sim
catkin_make
```

```bash
source devel/setup.bash
roslaunch surround_view_sim smooth_road.launch
```

