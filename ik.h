/**
 * Adapted from code by Oleg Mazurov
 * https://github.com/felis/USB_Host_Shield/blob/master/examples/arm_mouse.pde
 */
#include "LobotServoController.h"
#include <math.h>

#ifndef IK_H
#define IK_H

/* Arm dimensions( mm ) */
#define BASE_HGT 70 //base hight 2.65"
#define HUMERUS 102 //shoulder-to-elbow "bone" 5.75"
#define ULNA 135 //elbow-to-wrist "bone" 7.375"
#define GRIPPER 175 //gripper (incl.heavy duty wrist rotate mechanism) length 3.94"

#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  //float to long conversion

/* Servo names/numbers */
/* Base servo HS-485HB */
#define BAS_SERVO 6
/* Shoulder Servo HS-5745-MG */
#define SHL_SERVO 5
/* Elbow Servo HS-5745-MG */
#define ELB_SERVO 4
/* Wrist servo HS-645MG */
#define WRI_SERVO 3
/* Wrist rotate servo HS-485HB */
#define WRO_SERVO 2
/* Gripper servo HS-422 */
#define GRI_SERVO 1

void SetArmPosition(LobotServoController arm, float x, float y, float z, float grip_angle_d);
#endif
