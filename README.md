# sneaky-sky-stalker
## 1.配置ssh key
首先在本地电脑配置ssh key,使得可以通过ssh key登录到github账户,这在clone仓库以及push代码的时候会有用
## 2.克隆仓库并编译
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
## 3.开发自己的package
首先创建自己的分支
```shell
git checkout -b your-branch-name
```
不同的功能在不同的package内实现,类似的功能或内部依赖的功能在同一package内实现,创建package的指令类似如下
```shell
cd ~/sneaky-sky-stalker/src
catkin_create_pkg your_pkg roscpp rospy std_msgs sensor_msgs
```
最后提交修改
```shell
git add . && git commit -m "..."
git push origin your-branch-name
```
