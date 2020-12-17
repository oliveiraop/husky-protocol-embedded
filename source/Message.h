#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#include <stdint.h>

typedef struct {
	uint8_t const soh = 0xAA;
	uint8_t length;
	uint8_t lengthCompliment;
	uint8_t const version = 0x1;
	uint32_t timeStamp;
	uint8_t flags;
	uint16_t messageType;
	uint8_t const stx = 0x55;
	uint8_t payload[64]; // coloquei um valor fixo que seria um valor m�ximo para ser utilizado, pois o malloc nao funciona bem no pic32
	uint16_t checksum;
} Message;

void setMessage(Message *nova, uint8_t *payload, uint8_t length, uint16_t messageType, uint32_t timeStamp, uint8_t flags);

uint16_t getChecksum(Message nova);

void MessageSerializeToChecksum(uint8_t *data, Message nova);

void MessageSerialize(uint8_t *data, Message nova);


enum MessageTypes
  {
    /*
     * Set commands
     */
    SET_PLATFORM_NAME = 0x0002,
    SET_PLATFORM_TIME = 0x0005,
    SET_SAFETY_SYSTEM = 0x0010,
    SET_DIFF_WHEEL_SPEEDS = 0x0200,
    SET_DIFF_CTRL_CONSTS = 0x0201,
    SET_DIFF_WHEEL_SETPTS = 0x0202,
    SET_ACKERMANN_SETPT = 0x0203,
    SET_VELOCITY_SETPT = 0x0204,
    SET_TURN_SETPT = 0x0205,
    SET_MAX_SPEED = 0x0210,
    SET_MAX_ACCEL = 0x0211,
    SET_GEAR_SETPOINT = 0x0212,
    SET_GPADC_OUTPUT = 0x0300,
    SET_GPIO_DIRECTION = 0x0301,
    SET_GPIO_OUTPUT = 0x0302,
    SET_PTZ_POSITION = 0x0400,

    /*
     * Command commands
     */
    CMD_PROCESSOR_RESET = 0x2000,
    CMD_RESTORE_SETTINGS = 0x2001,
    CMD_STORE_SETTINGS = 0x2002,

    /*
     * Request commands
     */
        REQUEST_ECHO = 0x4000,
    REQUEST_PLATFORM_INFO = 0x4001,
    REQUEST_PLATFORM_NAME = 0x4002,
    REQUEST_FIRMWARE_INFO = 0x4003,
    REQUEST_SYSTEM_STATUS = 0x4004,
    REQUEST_POWER_SYSTEM = 0x4005,
    REQUEST_SAFETY_SYSTEM = 0x4010,
    REQUEST_DIFF_WHEEL_SPEEDS = 0x4200,
    REQUEST_DIFF_CTRL_CONSTS = 0x4201,
    REQUEST_DIFF_WHEEL_SETPTS = 0x4202,
    REQUEST_ACKERMANN_SETPTS = 0x4203,
    REQUEST_VELOCITY_SETPT = 0x4204,
    REQUEST_TURN_SETPT = 0x4205,
    REQUEST_MAX_SPEED = 0x4210,
    REQUEST_MAX_ACCEL = 0x4211,
    REQUEST_GEAR_SETPT = 0x4212,
    REQUEST_GPADC_OUTPUT = 0x4300,
    REQUEST_GPIO_STATUS = 0x4301,
    REQUEST_GPADC_INPUT = 0x4303,
    REQUEST_PTZ_POSITION = 0x4400,
    REQUEST_DISTANCE_DATA = 0x4500,
    REQUEST_DISTANCE_TIMING = 0x4501,
    REQUEST_ORIENT = 0x4600,
    REQUEST_ROT_RATE = 0x4601,
    REQUEST_ACCEL = 0x4602,
    REQUEST_6AXIS = 0x4603,
    REQUEST_6AXIS_ORIENT = 0x4604,
    REQUEST_ENCODER = 0x4800,
    REQUEST_ENCODER_RAW = 0x4801,

    /*
     * Data commands
     */
        DATA_ECHO = 0x8000,
    DATA_PLATFORM_INFO = 0x8001,
    DATA_PLATFORM_NAME = 0x8002,
    DATA_FIRMWARE_INFO = 0x8003,
    DATA_SYSTEM_STATUS = 0x8004,
    DATA_POWER_SYSTEM = 0x8005,
    DATA_PROC_STATUS = 0x8006,
    DATA_SAFETY_SYSTEM = 0x8010,
    DATA_DIFF_WHEEL_SPEEDS = 0x8200,
    DATA_DIFF_CTRL_CONSTS = 0x8201,
    DATA_DIFF_WHEEL_SETPTS = 0x8202,
    DATA_ACKERMANN_SETPTS = 0x8203,
    DATA_VELOCITY_SETPT = 0x8204,
    DATA_TURN_SETPT = 0x8205,
    DATA_MAX_SPEED = 0x8210,
    DATA_MAX_ACCEL = 0x8211,
    DATA_GEAR_SETPT = 0x8212,
    DATA_GPADC_OUTPUT = 0x8300,
    DATA_GPIO_STATUS = 0x8301,
    DATA_GPADC_INPUT = 0x8303,
    DATA_PTZ_POSITION = 0x8400,
    DATA_DISTANCE_DATA = 0x8500,
    DATA_DISTANCE_TIMING = 0x8501,
    DATA_ORIENT = 0x8600,
    DATA_ROT_RATE = 0x8601,
    DATA_ACCEL = 0x8602,
    DATA_6AXIS = 0x8603,
    DATA_6AXIS_ORIENT = 0x8604,
    DATA_MAGNETOMETER = 0x8606,
    DATA_ENCODER = 0x8800,
    DATA_ENCODER_RAW = 0x8801,
    DATA_CURRENT_RAW = 0xA110,
    DATA_VOLTAGE_RAW = 0xA111,
    DATA_TEMPERATURE_RAW = 0xA112,
    DATA_ORIENT_RAW = 0xA113,
    DATA_GYRO_RAW = 0xA114,
    DATA_ACCEL_RAW = 0xA115,
    DATA_MAGNETOMETER_RAW = 0xA116
  }; // enum MessageTypes

#endif _MESSAGE_H_
