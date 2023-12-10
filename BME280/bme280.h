#ifndef __BME280_H
#define __BME280_H

#define DEVICE_ID 0x76
#define DEVICE_PATH "/dev/i2c-1"

#define DEVICE_IDENT 0xD0
#define SOFT_RESET_CMD 0xE0
#define HUMIDITY_CTRL 0xF2
#define STATUS_REG 0xF3
#define MEASUREMENT_CTRL 0xF4
#define CONFIGURATION 0xF5

#define DATA_START_ADDR 0xF7
#define DATA_LENGTH 8


#endif

