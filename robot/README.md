# SO-101 model

`so101_new_calib.urdf` and the STL meshes in `assets/` come from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), `Simulation/SO101`,
under that repository's license. The URDF uses relative mesh paths.

"New calibration": every joint's zero sits in the middle of its range, as LeRobot calibrates
the arm today.

`so101.srdf` disables collision checks between adjacent links, which always touch.
