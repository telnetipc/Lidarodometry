# Lidarodometry
Lidarodometry using icp,ndt,and simple loop closure

![Screenshot](/result.png)

How to build with catkin:

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/laboshinl/loam_velodyne.git
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
$ source ~/catkin_ws/devel/setup.bash
```

Running:
```
roslaunch lidarodometry lidarodometry.launch
```

