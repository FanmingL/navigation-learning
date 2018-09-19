# navigation-learning
## Dependencies
* Ubuntu 16.04
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Pybullet
* Python2.7
* Google Protobuf
* ROS [move_base](http://wiki.ros.org/move_base) package ([GitHub](https://github.com/ros-planning/navigation))
* OpenCV 3.4
* Eigen3


```bash
sudo apt-get install protobuf-compiler libprotobuf-dev cmake libeigen3-dev
sudo pip2 install pybullet numpy
```
## Effect
### Simple PID Controller

![image](./image/effect.gif)

### Move Base

![image](./image/effect_movebase.gif)

### Kalman Filter

![image](./image/effect_kalman_filter.gif)

### Video Upload Test
<iframe width="860" height="400" src="https://box.nju.edu.cn/seafhttp/files/4eea7c73-1d78-4d82-91df-dffe08322bf6/double%20UAVs%20control%20processed.mp4" frameborder="0"  allowfullscreen></iframe>