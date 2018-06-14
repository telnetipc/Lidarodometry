# Lidarodometry
Lidarodometry using icp,ndt,and simple loop closure

Vedio: https://v.youku.com/v_show/id_XMzY2MTgzODc0OA==.html?spm=a2h3j.8428770.3416059.1

![Screenshot](/result.png)

How to build with catkin:

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/telnetipc/Lidarodometry.git
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
$ source ~/catkin_ws/devel/setup.bash
```

Running:
```
roslaunch lidarodometry lidarodometry.launch
```

