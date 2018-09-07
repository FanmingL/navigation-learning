# navigation-learning
## Dependencies
* Ubuntu 16.04
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Pybullet
* Python2.7
* Google Protobuf
* ROS [move_base](http://wiki.ros.org/move_base) package

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