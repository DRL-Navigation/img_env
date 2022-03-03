# drlnav_env

这是实验室基于传感器sensor map的导航仿真环境, 适配gym的api

安装ubuntu20.04 和 ros 见 [20.04StartUP](https://git.ustc.edu.cn/drl_navigation/startup/-/blob/master/%E7%B3%BB%E7%BB%9F%E7%8E%AF%E5%A2%83%E9%83%A8%E7%BD%B2/ubuntu20.04startup.md)


### Quick Start

```
sudo apt-get install libedit-dev

git clone git@github.com:DRL-Navigation/img_env.git

cd drlnav_env
```

#### 编译:

```
catkin_make --only-pkg-with-deps img_env
```

添加依赖

```
echo "source `pwd`/devel/setup.bash" >> ~/.bashrc
```

#### 配置yaml

进入envs/cfg 对要跑的yaml进行配置， 如果当前配置要开几个一起跑，设置env_num参数。

#### 生成一个launch文件

可以手写,也可以用`create_launch.py`脚本, 当你 ROS node比较多的时候,建议用脚本, 用法如下:

`python create_launch.py test envs/cfg/test.yaml`

第一个参数为task name（也是生成的launch文件名），之后的参数为yaml文件， 如果要跑好几个yaml环境，

`python create_launch.py test A.yaml B.yaml C.yaml`

生成的launch文件位于`src/drl_nav/img_env/tmp`下

#### 启动ROS

新开一个终端

`roslaunch img_env test.launch`



#### 跑python

`python env_test.py`



### 说明

#### State 状态类
`envs/state/state.py`
以下所有值类型为numpy， 均为env下的所有robots的属性， 即第一个维度为robot_num
```
    vector_states,  [robot_num, state_dim] 向量形式的状态，state_dim=3时候，分别表示距离终点x距离 y距离和yaw
    sensor_maps, [robot_num, batch, 48, 48] 以robot为中心的激光图
    is_collisions, [robot_num] 判断碰撞，0表示没撞,1撞障碍物,2撞行人 ,3撞别的小车
    is_arrives, [robot_num] 判断到达，0表示没达到，1到达
    lasers, [robot_num, laser_range_total] 一维原始激光数据
    ped_vector_states, [robot_num, n] 一维行人向量信息，维度n = 1 + vector_number * max_ped_number, 开头的1表示实际ped number
    ped_maps, [robot_num, 3, 48, 48]三通道行人图，分别表示行人x速度，y速度以及位置
    step_ds, [robot_num] 每一步靠近target的距离
    ped_min_dists, [robot_num] 一个值，最近行人距离
```



#### Wrapper

修改reward, 动作空间等操作, 请依照gym的规范, 进行wrapper封装

**!!!** NOTE: 在yaml里面填入wrapper的顺序非常重要,  第一个封装在最里层, 最先执行,最后一个封装在最外层, 最后执行,这里不明白的想象一下树遍历的先序和后序



#### 环境参数

当前, 所有参数设置均在yaml里,  由python读取后, 发给C++



### GUI 运行期间如何关闭打开

参数服务器

```
#关闭
rosparam set test/show_gui false
#开启
rosparam set test/show_gui true
#注意这里的test改成你想开启的node name， 具体为yaml里设置的env_name+env_id
#可通过rosnode list 查看所有node
```



#### 行人模型

行人控制方法分为 pedsim, rvo , ervo, 代码均为开源, 已下载并放在3rd_party

rvo为orca控制,  ervo 为情感类(emitional)的rvo , 在rvo的基础上进行了一些修改, 具体可见ervo代码

修改行人模型在yaml文件里, `ped_sim:type` 下面可选 `pedscene`,`rvoscene`,`ervoscene` 





please cite：

```
@Article{chen2020distributed,
title = {Distributed Non-Communicating Multi-Robot Collision Avoidance via Map-Based Deep Reinforcement Learning},
author = {Chen, Guangda and Yao, Shunyi and Ma, Jun and Pan, Lifan and Chen, Yu'an and Xu, Pei and Ji, Jianmin and Chen, Xiaoping},
journal = {Sensors},
volume = {20},
number = {17},
pages = {4836},
year = {2020},
publisher = {Multidisciplinary Digital Publishing Institute},
doi = {10.3390/s20174836},
url = {https://www.mdpi.com/1424-8220/20/17/4836}
}
```
