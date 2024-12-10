/*

   Inspired by work done here
   https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>
   https://github.com/opentx/opentx/tree/2.3/radio/src/telemetry from the OpenTX team

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* 
   FRSKY Telemetry library
*/
//扭转舵机

/*#include "AP_TorsionServo.h"


#include <AP_SerialManager/AP_SerialManager.h>


#define AP_SERIALMANAGER_TORSIONSERVO_BAUD         115200
#define AP_SERIALMANAGER_TORSIONSERVO_BUFSIZE_RX       64
#define AP_SERIALMANAGER_TORSIONSERVO_BUFSIZE_TX       64
#define LOBOT_SERVO_POS_READ                           28

extern const AP_HAL::HAL& hal;


AP_TorsionServo::AP_TorsionServo(void)
{
    _port = NULL;
    _step = 0;
}

/*
 * init - perform required initialisation
 */

// init - perform require initialisation including detecting which protocol to use
/*void AP_TorsionServo::init(const AP_SerialManager& serial_manager)
{
    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_TORSIONSERVO, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_SERIALMANAGER_TORSIONSERVO_BAUD, AP_SERIALMANAGER_TORSIONSERVO_BUFSIZE_RX, AP_SERIALMANAGER_TORSIONSERVO_BUFSIZE_TX);
    }
}

bool AP_TorsionServo::readposition()
{
    if(_port == NULL)
        return false;

    int16_t numc = _port->available();
    uint8_t data;
    uint8_t checksum = 0;


}

校验和
byte LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

//解析接收到的数据包信息，并返回
int AP_TorsionServo::ServoReceiveHandle(HardwareSerial &SerialX, byte *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  while (SerialX.available()) {
    rxBuf = SerialX.read();
    delayMicroseconds(100);
    if (!frameStarted) {
      if (rxBuf == 0x55) {
        frameCount++;
        if (frameCount == 2) {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted) {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3) {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7) {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3) {

        if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
}

    //读取舵机位置
int AP_TorsionServo::ServoReadPosition(uint8_t id)
{
    int count = 10000;
    int ret;
    byte buf[6];

    buf[0] = buf[1] = 0x55;//帧头
    buf[2] = id;
    buf[3] = 3;//数据长度
    buf[4] = LOBOT_SERVO_POS_READ;//指令
    buf[5] = LobotCheckSum(buf);//校验和
    for(int16_t i=0;i<6;i++)
    {
        _port->write(buf[i],sizeof(buf[i]));
    }
  
    while (_port->available())
        _port->read();

    while (!_port->available()) {
        count -= 1;
        if (count < 0)
        return -2048;
    }

    if (ServoReceiveHandle(SerialX, buf) > 0)
        ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
    else
        ret = -2048;
    return ret;
}
*/




