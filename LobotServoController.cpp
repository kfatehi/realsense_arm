/******************************************************
 * FileName:      LobotServoController.cpp
 ** Company:       Lewan Soul 
 * Date:           2016/07/02  16:53
 *Last Modification Date: 201706281636
* www.lewansoul.com
 *****************************************************/

#include "LobotServoController.h"
#include <stdio.h>
#include <chrono>

auto progstart = std::chrono::high_resolution_clock::now();

int millis()
{
    using namespace std::chrono;
    auto now = high_resolution_clock::now();
    int msec = duration_cast<milliseconds>(now - progstart).count();
    return msec;
}

LobotServoController::LobotServoController(int _fd)
{
	numOfActinGroupRunning = 0xFF;
	actionGroupRunTimes = 0;
	isGetBatteryVolt = false;
	isRunning_ = false;
	batteryVoltage = 0;
	isUseHardwareSerial = false;
  fd = _fd;
}

void LobotServoController::moveServo(u_int8_t servoID, u_int16_t Position, u_int16_t Time)
{
	char buf[11];
	if (servoID > 31 || !(Time > 0)) {
		return;
	}
	buf[0] = FRAME_HEADER;             
	buf[1] = FRAME_HEADER;
	buf[2] = 8;                             
	buf[3] = CMD_SERVO_MOVE;              
	buf[4] = 1;                         
	buf[5] = GET_LOW_BYTE(Time);        
	buf[6] = GET_HIGH_BYTE(Time);        
	buf[7] = servoID;             
	buf[8] = GET_LOW_BYTE(Position);        
	buf[9] = GET_HIGH_BYTE(Position);        

  write(fd, buf, 10);
}

void LobotServoController::sendCMDGetBatteryVolt()
{
	u_int8_t buf[4];
	buf[0] = FRAME_HEADER;
	buf[1] = FRAME_HEADER;
	buf[2] = 2;                 
	buf[3] = CMD_GET_BATTERY_VOLTAGE; 
	isGetBatteryVolt = false;
  write(fd, buf, 4);
}

u_int16_t LobotServoController::getBatteryVolt(void)
{
	if(isGetBatteryVolt)
	{
		isGetBatteryVolt = false;
		return batteryVoltage;
	}else{
		return -1;
	}
}

u_int16_t LobotServoController::getBatteryVolt(u_int32_t timeout)
{
	isGetBatteryVolt = false;
	sendCMDGetBatteryVolt();
	timeout += millis();
	while(!isGetBatteryVolt)
	{
		if(timeout < millis())
		{
			return -1;
		}
		receiveHandle();
	}
	return batteryVoltage;
}
bool LobotServoController::isRunning()
{
	return isRunning_;
}

bool LobotServoController::waitForStopping(u_int32_t timeout)
{
	u_int8_t rx;
	while(read(fd, &rx, 1) > 0) { }
	timeout += millis();
	while(isRunning_)
	{
		if(timeout < millis())
		{
			return false;
		}
		receiveHandle();
	}
	return true;
}

void LobotServoController::receiveHandle()
{
	u_int8_t rx;
	static u_int8_t buf[16];
	static bool isGetFrameHeader = false;
	static u_int8_t frameHeaderCount = 0;
	static u_int8_t dataLength = 2;
	static u_int8_t dataCount = 0;

	while(read(fd, &rx, 1) > 0)
	{
    printf("read something\n");
		if(!isGetFrameHeader)
		{
			if(rx == 0x55)
			{
				frameHeaderCount++;
				if(frameHeaderCount == 2)
				{
					frameHeaderCount = 0;
					isGetFrameHeader = true;
					dataCount = 1;
				}
			} else {
				isGetFrameHeader = false;
				dataCount = 0;
				frameHeaderCount = 0;
			}
		}
		if(isGetFrameHeader)
		{
			buf[dataCount] = rx;
			if(dataCount == 2)
			{
				dataLength = buf[dataCount];
				if(dataLength < 2 || dataLength > 8)
				{
					dataLength = 2;
					isGetFrameHeader = false;
				}
			}
			dataCount++;
			if(dataCount == dataLength + 2)
			{
				isGetFrameHeader = false;
				switch(buf[3])
				{
					case BATTERY_VOLTAGE:
						batteryVoltage = BYTE_TO_HW(buf[5], buf[4]);
						isGetBatteryVolt = true;
						break;
					case ACTION_GROUP_RUNNING:
						isRunning_ = true;
						break;
					case ACTION_GROUP_COMPLETE:
					case ACTION_GROUP_STOPPED:
						isRunning_ = false;
						break;
					default:
						break;
				}
			}
		}
	}
}
