# sneaky-sky-stalker
## 克隆仓库并编译
```shell
#到home目录下
cd ~
git clone git@github.com:FranCOOlin/sneaky-sky-stalker.git
cd ~/sneaky-sky-stalker
catkin clean
catkin build
echo "source ~/sneaky-sky-stalker/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

```
## 开发自己的package
```shell
cd ~/sneaky-sky-stalker/src
catkin_create_pkg your_pkg roscpp rospy std_msgs sensor_msgs
```
