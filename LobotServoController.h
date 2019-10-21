/**
 * Adapted from code by Lewan Soul
 * http://lewansoul.com/
 */
#include <sys/types.h>
#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>

#ifndef LOBOTSERVOCONTROLLER_H
#define LOBOTSERVOCONTROLLER_H

#define FRAME_HEADER            0x55
#define CMD_SERVO_MOVE          0x03
#define CMD_ACTION_GROUP_RUN    0x06
#define CMD_ACTION_GROUP_STOP   0x07
#define CMD_ACTION_GROUP_SPEED  0x0B
#define CMD_GET_BATTERY_VOLTAGE 0x0F 


#define BATTERY_VOLTAGE       0x0F  
#define ACTION_GROUP_RUNNING  0x06
#define ACTION_GROUP_STOPPED  0x07
#define ACTION_GROUP_COMPLETE 0x08

#define GET_LOW_BYTE(A) (u_int8_t)((A))
#define GET_HIGH_BYTE(A) (u_int8_t)((A) >> 8)
#define BYTE_TO_HW(A, B) ((((u_int16_t)(A)) << 8) | (u_int8_t)(B))

struct LobotServo { 
  u_int8_t  ID; 
  u_int16_t Position;
};

class LobotServoController {
  public:
    LobotServoController(int fd);
    void moveServo(u_int8_t servoID, u_int16_t Position, u_int16_t Time);

    u_int16_t getBatteryVolt(void);
    u_int16_t getBatteryVolt(u_int32_t timeout);
    bool waitForStopping(u_int32_t timeout);
    void sendCMDGetBatteryVolt(void);
    bool isRunning(void);
    void receiveHandle(void);

  public:
    u_int8_t  numOfActinGroupRunning; 
    u_int16_t actionGroupRunTimes;

  private:
    int fd;
    bool isUseHardwareSerial;
    bool isGetBatteryVolt;
    u_int16_t batteryVoltage;
    bool isRunning_;
};
#endif
