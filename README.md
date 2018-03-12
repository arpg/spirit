# spirit
spirit is a library to control a non-holonomic vehicle with a Model-based predictive controller

##Dependencies():
- Eigen3
- [Sophus](https://github.com/arpg/Sophus)
- [Pangolin](https://github.com/arpg/Pangolin)
- [SceneGraph](https://github.com/arpg/SceneGraph)
- [CarPlanner](https://github.com/arpg/CarPlanner)
- GLOG

##Applications
- ninja_gui : application to control our custom rc  car

## Other Instructions:
- TBD

## Bicycle Model notes
We would like to create a bicycle model which is kinematic and fixed to
a mesh, using a RK45 integrator rather than bullet. Here are some of the
notes for this.

![](bikeSimMATLAB/filenamebike1.jpg)
