/**
 * Adapted from code by Oleg Mazurov
 * https://github.com/felis/USB_Host_Shield/blob/master/examples/arm_mouse.pde
 */
#include "ik.h"
#include <stdio.h>

/* pre-calculations */
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;
   
float radians(float degrees) {
  return ( degrees * M_PI ) / 180;
}

float degrees(float radians) {
  return ( radians * 180 ) / M_PI;
}

/* arm positioning routine utilizing inverse kinematics */
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive */
void SetArmPosition(LobotServoController arm, float x, float y, float z, float grip_angle_d)
{
  float grip_angle_r = radians( grip_angle_d );    //grip angle in radians for use in calculations
  /* Base angle and radial distance from x,y coordinates */
  float bas_angle_r = atan2( x, y );
  float rdist = sqrt(( x * x ) + ( y * y ));
  /* rdist is y coordinate for the arm */
  y = rdist;
  /* Grip offsets calculated based on grip angle */
  float grip_off_z = ( sin( grip_angle_r )) * GRIPPER;
  float grip_off_y = ( cos( grip_angle_r )) * GRIPPER; 
  /* Wrist position */
  float wrist_z = ( z - grip_off_z ) - BASE_HGT;
  float wrist_y = y - grip_off_y;
  /* Shoulder to wrist distance ( AKA sw ) */
  float s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
  float s_w_sqrt = sqrt( s_w );
  /* s_w angle to ground */
  //float a1 = atan2( wrist_y, wrist_z ); 
  float a1 = atan2( wrist_z, wrist_y );
  /* s_w angle to humerus */
  float a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt ));
  /* shoulder angle */
  float shl_angle_r = a1 + a2;
  float shl_angle_d = degrees( shl_angle_r ); 
  /* elbow angle */
  float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ));
  float elb_angle_d = degrees( elb_angle_r );
  float elb_angle_dn = -( 180.0 - elb_angle_d );
  /* wrist angle */
  float wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d;
 
  /* Servo pulses */
  float bas_servopulse = 1500.0 - (( degrees( bas_angle_r )) * 11.11 );
  float shl_servopulse = 1500.0 + (( shl_angle_d - 90.0 ) * 6.6 );
  float elb_servopulse = 1500.0 -  (( elb_angle_d - 90.0 ) * 6.6 );
  float wri_servopulse = 1500 + ( wri_angle_d  * 11.1 );

  /* Set servos */
  //servos.setposition( BAS_SERVO, ftl( bas_servopulse ));
  //servos.setposition( WRI_SERVO, ftl( wri_servopulse ));
  //servos.setposition( SHL_SERVO, ftl( shl_servopulse ));
  //servos.setposition( ELB_SERVO, ftl( elb_servopulse ));

  printf("move: %d %ld\n", BAS_SERVO, ftl(bas_servopulse));
  printf("move: %d %ld\n", WRI_SERVO, ftl(wri_servopulse));
  printf("move: %d %ld\n", SHL_SERVO, ftl(shl_servopulse));
  printf("move: %d %ld\n", ELB_SERVO, ftl(elb_servopulse));

  arm.moveServo(BAS_SERVO, ftl(bas_servopulse), 100);
  arm.moveServo(WRI_SERVO, ftl(wri_servopulse), 100);
  arm.moveServo(SHL_SERVO, ftl(shl_servopulse), 100);
  arm.moveServo(ELB_SERVO, ftl(elb_servopulse), 100);
}
