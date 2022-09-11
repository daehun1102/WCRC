#include "Arduino.h"

#define SERVO_FRAME_HEADER          0x55
#define SERVO_MOVE_TIME_WRITE       1
#define SERVO_MOVE_TIME_READ        2
#define SERVO_MOVE_TIME_WAIT_WRITE  7
#define SERVO_MOVE_TIME_WAIT_READ   8
#define SERVO_MOVE_START            11
#define SERVO_MOVE_STOP             12
#define SERVO_ID_WRITE              13
#define SERVO_ID_READ               14
#define SERVO_ANGLE_OFFSET_ADJUST   17
#define SERVO_ANGLE_OFFSET_WRITE    18
#define SERVO_ANGLE_OFFSET_READ     19
#define SERVO_ANGLE_LIMIT_WRITE     20
#define SERVO_ANGLE_LIMIT_READ      21
#define SERVO_VIN_LIMIT_WRITE       22
#define SERVO_VIN_LIMIT_READ        23
#define SERVO_TEMP_MAX_LIMIT_WRITE  24
#define SERVO_TEMP_MAX_LIMIT_READ   25
#define SERVO_TEMP_READ             26
#define SERVO_VIN_READ              27
#define SERVO_POS_READ              28
#define SERVO_OR_MOTOR_MODE_WRITE   29
#define SERVO_OR_MOTOR_MODE_READ    30
#define SERVO_LOAD_OR_UNLOAD_WRITE  31
#define SERVO_LOAD_OR_UNLOAD_READ   32
#define SERVO_LED_CTRL_WRITE        33
#define SERVO_LED_CTRL_READ         34
#define SERVO_LED_ERROR_WRITE       35
#define SERVO_LED_ERROR_READ        36

//-- Macro Functions --//
//get lower 8 bits of A
#define GET_LOW_BYTE(A)     (uint8_t)((A))
//get higher 8 bits of A
#define GET_HIGH_BYTE(A)    (uint8_t)((A) >> 8)
//put A as higher 8 bits, B as lower 8 bits which amalgamated into 16 bits integer
#define BYTE_TO_16B(A, B)   ((((uint16_t)(A)) << 8) | (uint8_t)(B))

//LX-15D control Function
byte CheckSumGen(byte buf[]);
void ServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time);
void ServoStopMove(HardwareSerial &SerialX, uint8_t id);
void ServoLoad(HardwareSerial &SerialX, uint8_t id);
void ServoUnload(HardwareSerial &SerialX, uint8_t id);
int ServoReceiveHandle(HardwareSerial &SerialX, byte *ret);
int ServoReadPosition(HardwareSerial &SerialX, uint8_t id);

//X-Arm Function
