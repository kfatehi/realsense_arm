# realsense_arm

[![Watch the video](https://img.youtube.com/vi/qem4urOBqU0/maxresdefault.jpg)](https://youtu.be/qem4urOBqU0)


Small CMake Linux program that talks to two USB devices:

* Intel RealSense t265 pose tracker
* LewanSoul LoBot 6DoF robotic arm

It takes the pose information from the t265 and applies it with inverse kinematics to the effector of the robot arm

It is not done, but serves as a nice starting point for the concept being explored.

## Usage

Make sure you compile, make, and install [librealsense](https://github.com/IntelRealSense/librealsense) first.

Then, use the same standard cmake workflow:

```
mkdir build
cd build
cmake ..
make
```

Then run it with `sudo ./realsense_arm`


