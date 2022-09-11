#include "AIRO_XARM.h"

//#define DEBUG_MODE    1   /*Debug ：print debug value*/

byte CheckSumGen(byte buf[])
{
    byte i, chk=0;
    uint16_t temp = 0;

    for (i = 2; i < buf[3] + 2; i++)
        temp += buf[i];

    temp = ~temp;
    chk = (byte)temp;

    return chk;
}

void ServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time)
{
    byte buf[10];
    if(position < 0)
        position = 0;
    if(position > 1000)
        position = 1000;

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = 7;
    buf[4] = SERVO_MOVE_TIME_WRITE;
    buf[5] = GET_LOW_BYTE(position);
    buf[6] = GET_HIGH_BYTE(position);
    buf[7] = GET_LOW_BYTE(time);
    buf[8] = GET_HIGH_BYTE(time);
    buf[9] = CheckSumGen(buf);
    SerialX.write(buf, 10);
}

void ServoStopMove(HardwareSerial &SerialX, uint8_t id)
{
    byte buf[6];
    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = 3;
    buf[4] = SERVO_MOVE_STOP;
    buf[5] = CheckSumGen(buf);
    SerialX.write(buf, 6);
}

void ServoLoad(HardwareSerial &SerialX, uint8_t id)
{
    byte buf[7];
    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = 4;
    buf[4] = SERVO_LOAD_OR_UNLOAD_WRITE;
    buf[5] = 1;
    buf[6] = CheckSumGen(buf);
    SerialX.write(buf, 7);
  
#ifdef DEBUG_MODE
    Serial.println("SERVO LOAD WRITE");
    int debug_value_i = 0;
    for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
    {
        Serial.print(buf[debug_value_i], HEX);
        Serial.print(":");
    }
    Serial.println(" ");
#endif
}

void ServoUnload(HardwareSerial &SerialX, uint8_t id)
{
    byte buf[7];
    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = 4;
    buf[4] = SERVO_LOAD_OR_UNLOAD_WRITE;
    buf[5] = 0;
    buf[6] = CheckSumGen(buf);
    SerialX.write(buf, 7);
  
#ifdef DEBUG_MODE
    Serial.println("SERVO LOAD WRITE");
    int debug_value_i = 0;
    for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
    {
        Serial.print(buf[debug_value_i], HEX);
        Serial.print(":");
    }
    Serial.println(" ");
#endif
}

int ServoReceiveHandle(HardwareSerial &SerialX, byte *ret)
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
            if (rxBuf == SERVO_FRAME_HEADER) {
                frameCount++;
                if (frameCount == 2) {
                    frameCount = 0;
                    frameStarted = true;
                    dataCount = 1;
                    recvBuf[0] = 0x55;  //의미는 없지만, 해줌!
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
        
#ifdef DEBUG_MODE
                Serial.print("RECEIVE DATA:");
                for (i = 0; i < dataCount; i++) {
                    Serial.print(recvBuf[i], HEX);
                    Serial.print(":");
                }
                Serial.println(" ");
#endif

                if (CheckSumGen(recvBuf) == recvBuf[dataCount - 1]) {
          
#ifdef DEBUG_MODE
                    Serial.println("Check SUM OK!!");
                    Serial.println("");
#endif

                    frameStarted = false;
                    memcpy(ret, recvBuf + 2, dataLength);
                    return 1;
                }
                return -1;
            }
        }
    }
}

int ServoReadPosition(HardwareSerial &SerialX, uint8_t id)
{
    int count = 10000;
    int ret;
    byte buf[6];

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = 3;
    buf[4] = SERVO_POS_READ;
    buf[5] = CheckSumGen(buf);

#ifdef DEBUG_MODE
    Serial.println("SERVO Pos READ");
    int debug_value_i = 0;
    for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
    {
        Serial.print(buf[debug_value_i], HEX);
        Serial.print(":");
    }
    Serial.println(" ");
#endif

    while (SerialX.available())
        SerialX.read();

    SerialX.write(buf, 6);

    while (!SerialX.available()) {
        count -= 1;
        if (count < 0)
            return -1;
    }

    if (ServoReceiveHandle(SerialX, buf) > 0)
        ret = BYTE_TO_16B(buf[4], buf[3]);
    else
        ret = -1;

#ifdef DEBUG_MODE
    Serial.println(ret);
#endif

    return ret;
}

//======================== X-Arm function ========================//
