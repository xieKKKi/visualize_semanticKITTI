# visualize_semanticKITTI
C++ visualization tools for semanticKITTI.

![visualize](./visualize.gif "visualize")
## Usage
```shell
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/xieKKKi/visualize_semanticKITTI.git
cd ..
catkin_make
. devel/setup.bash
```
change the pathKITTI and targetSequence in config/param.yaml     
Run visualization:
```shell
roslaunch visualizeSemanticKITTI visualize.launch
```
