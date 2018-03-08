# spirit
spirit is a library to control one or group of ground vehicle with a Model-based predictive controller

## Dependencie
- Eigen3
- [HAL](https://github.com/arpg/HAL) : if examples used
- [Pangolin](https://github.com/arpg/Pangolin)
- [SceneGraph](https://github.com/arpg/SceneGraph)
- [Bullet2.84](https://github.com/bulletphysics/bullet3/archive/2.84.zip)
- [ceres-solver](https://github.com/ceres-solver/ceres-solver)

## Examples
- sim_car : application showing simple vehicle simulation on flat ground
- bvp_example : simple boundary value solver example showing a local planning problem between two waypoints
- car_robot : application to test internals of spirit
- car_calib : application to calibrate vehicle parameters (requires arpg's parkour car robot to run)
-  gamepad_drive : application to drive parkour car with a gamepad (requires arpg's parkour car robot to run)
-  mpc_car : application using spirit's internal mpc controller (requires arpg's parkour car robot to run)

## Developers
- [Sina Aghli](https://github.com/sinaaghli)

## Publications
- Sina Aghli and Christoffer Heckman, “Online System Identification and Calibration of Dynamic Models for Autonomous Ground Vehicles” IEEE International Conference On Robotics and Automation 2018
- H Ravanbakhsh, S Aghli, C Heckman, S Sankaranarayanan, “Path-Following though Control Lyapunov Functions” International Conference on Intelligent Robots 2018

