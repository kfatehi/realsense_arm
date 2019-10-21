// SPDX-License-Identifier: GPL-2.0
/**
 * Adapted from code by:
 *
 * Copyright (c) 2010 Alan Ott <alan@signal11.us>
 * Copyright (c) 2010 Signal 11 Software
 *
 * Specifically, the Hidraw Userspace Example:
 * https://github.com/torvalds/linux/blob/master/samples/hidraw/hid-example.c
 *
 * The code may be used by anyone for any purpose,
 */

/* Linux */
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>

/*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
 */
#ifndef HIDIOCSFEATURE
#warning Please have your distro update the userspace kernel headers
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

/* Unix */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* C */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "LobotServoController.h"
#include "ik.h"
#include <chrono>
#include <thread>

#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>


int main(int argc, char **argv)
{
  int fd;
  const char *device = "/dev/hidraw0";
  fd = open(device, O_RDWR|O_NONBLOCK);

  if (fd < 0) {
    perror("unable to open device");
    return 1;
  }

  LobotServoController arm = LobotServoController(fd);
  //SetArmPosition(arm, 0,0,0,0);
  // 1: gripper, 1500 -> 2465
  // 2: wrist rotation, 500 -> 2500
  // 3: wrist position, 500 -> 2500
  // 4: elbow, 500 -> 2500
  // 5: shoulder, 500 -> 2500
  // 6: base, 500 -> 2500
  arm.moveServo(1, 1500, 1000);
  arm.moveServo(2, 1500, 1000);
  arm.moveServo(3, 1500, 1000);
  arm.moveServo(4, 1500, 1000);
  arm.moveServo(5, 1500, 1000);
  arm.moveServo(6, 1500, 1000);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;
  // Add pose stream
  cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  // Start pipeline with chosen configuration
  pipe.start(cfg);

  // Main loop
  while (true)
  {
    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();
    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    // Cast the frame to pose_frame and get its data
    auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

    int mult = 1000;

    auto x = pose_data.translation.x * mult;
    auto y = pose_data.translation.y * mult;
    auto z = pose_data.translation.z * mult;


    // Print the x, y, z values of the translation, relative to initial position
    std::cout << "Device Position: " << std::setprecision(0) << std::fixed << x << " " <<
      y << " " << z << " (mm)\n";


    SetArmPosition(arm, x, y, z, 0);


    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  close(fd);
  return 0;
}
