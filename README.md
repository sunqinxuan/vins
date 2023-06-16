# 分支说明:
本代码RES-MAL工程代码，融合环视，GNSS，imu,轮速计数据。
# 1.环境
- ubuntu 16.04 (ubuntu 20.04)
- ros kinectic (ros noetic)
- pcl 1.7 (pcl 1.10)
- Eigen 3.3.7
- ceres 1.14

首先安装相关依赖。

```
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.4 libgflags-dev 
sudo apt-get install libgoogle-glog-dev libgtest-dev libatlas-base-dev  libeigen3-dev
```

下载[稳定版本](https://github.com/ceres-solver/ceres-solver)，然后安装。

```bash
mkdir build && cd build
cmake .. && make -j4
sudo make install
```

- Sophus a621ff

```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build
cd build
cmake ..
make
sudo make install
```



# 2.编译

1.）第三方库环境准备

​	第三方库目前都集成到AVP-SLAM-LIB工程中，在该工程下编译生成third_party包含依赖的第三方库文件与头文件。将其拷贝至本工程根目录下即可。

2.）模块库编译与使用

​	模块库的使用通过USE_CODE的变量(默认为true）控制。变量为真表示使用源码编译工程，正常编译即可。

```
catkin_make 
```

   如果想将生成的库文件打包到指定目录（工程根目录lib文件夹内），则可使用以下指令编译。

```
catkin_make install
```

  如果想使用库文件编译工程，则在库文件已生成的情况下，可使用以下指令。

```
cankin_make -DUSE_CODE=off
```




# 3.运行
1) 建图

```
roslaunch vision_localization mapping.launch 
```

1.1)手动闭环

首先，通过KeyFrameID对应的Maker编号，确定手动闭环的关键帧编号，并执行下面服务指令。

```
rosservice call /search_loop <key_frame_id> #关键帧编号
```

然后，选中Rviz菜单栏中"2D Pose Estimate"图标，在rviz界面中提供该关键帧相对于map的估计位置与姿态。

最后，在经过一系列校验后，可以成功添加手动闭环，并发送到后端。

2) 匹配定位

```
roslaunch vision_localization matching.launch
```



# 4.评估
1）安装EVO

```
pip install evo --upgrade --no-binary evo
```

快速下载

```
pip install evo --upgrade --no-binary evo -i https://pypi.tuna.tsinghua.edu.cn/simple
```

2）使用EVO评价数据
相对误差（衡量里程计精度）

```
evo_rpe tum ground_truth.txt vision_odom.txt -r trans_part -va --plot --plot_mode xyz --delta 100 -u m
```

绝对误差（衡量建图和定位精度）

```
evo_ape tum ground_truth.txt optimized.txt  -r full -va --plot --plot_mode xyz
```

画轨迹,六个自由度误差分析

```
evo_traj tum optimized.txt vision_odom.txt --ref=ground_truth.txt -p --plot_mode=xyz
```



# 5.排版

1.) 安装排版工具

```
sudo apt-get install clang-format
```

2.) 执行排版

​	在每次提交前,先对工程中所有程序文件做一次排版.

```
./clang-format.sh
```
