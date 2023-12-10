#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include "i2c_utils.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <errno.h>

// Device configurations
#define DEVICE_ADDRESS 0x76
#define DEVICE_PATH "/dev/i2c-1"
#define LOCAL_HEIGHT_ASL 25.0 // Local height above sea level in meters

// Register addresses
#define CHIP_IDENTIFIER 0xD0
#define SOFT_RESET 0xE0
#define CTRL_HUMIDITY 0xF2
#define STATUS_REG 0xF3
#define CTRL_MEASURE 0xF4
#define CONFIG_REG 0xF5

#define DATA_START_ADDR 0xF7
#define DATA_LENGTH 8

// Constants for sea level pressure conversion
#define GRAVITY 9.80665
#define MOLAR_MASS 0.0289644
#define STANDARD_TEMP 288.15
#define GAS_CONSTANT 8.3144598

double calculateSeaLevelPressure(double station_pressure);

#define CALIB_DATA0_START_ADDR 0x88
#define CALIB_DATA0_LENGTH 25
#define CALIB_DATA1_START_ADDR 0xE1
#define CALIB_DATA1_LENGTH 7

double compensateTemperature(int32_t temp_adc);
double compensatePressure(int32_t pressure_adc);
double compensateHumidity(int32_t humidity_adc);
void initializeCompensation(int file_descriptor);

static int32_t fine_temperature = 0;

static uint16_t calib_T1 = 0;
static int16_t calib_T2 = 0;
static int16_t calib_T3 = 0;

static uint16_t calib_P1 = 0;
static int16_t calib_P2 = 0;
static int16_t calib_P3 = 0;
static int16_t calib_P4 = 0;
static int16_t calib_P5 = 0;
static int16_t calib_P6 = 0;
static int16_t calib_P7 = 0;
static int16_t calib_P8 = 0;
static int16_t calib_P9 = 0;

static uint8_t calib_H1 = 0;
static int16_t calib_H2 = 0;
static uint8_t calib_H3 = 0;
static int16_t calib_H4 = 0;
static int16_t calib_H5 = 0;
static int8_t calib_H6 = 0;

double compensateTemperature(int32_t temp_adc) {
    // temperature sensor data compensation
    double var1, var2, T;
    var1 = (((double)temp_adc) / 16384.0 - ((double)calib_T1) / 1024.0) * ((double)calib_T2);
    var2 = ((((double)temp_adc) / 131072.0 - ((double)calib_T1) / 8192.0) *
            (((double)temp_adc) / 131072.0 - ((double)calib_T1) / 8192.0)) *
           ((double)calib_T3);
    fine_temperature = (int32_t)(var1 + var2);
    T = (var1 + var2) / 5120.0;
    return T;
}

double compensatePressure(int32_t pressure_adc) {
    // pressure sensor data compensation
    double var1, var2, p;
    var1 = ((double)fine_temperature / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)calib_P6) / 32768.0;
    var2 = var2 + var1 * ((double)calib_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)calib_P4) * 65536.0);
    var1 = (((double)calib_P3) * var1 * var1 / 524288.0 + ((double)calib_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calib_P1);
    /* avoid exception caused by division by zero */
    if (var1 == 0.0) {
        return 0;
    }
    p = 1048576.0 - (double)pressure_adc;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)calib_P9) * p * p / 2147483648.0;
    var2 = p * ((double)calib_P8) / 32768.0;
    p = p + (var1 + var2 + ((double)calib_P7)) / 16.0;
    return p;
}

double compensateHumidity(int32_t humidity_adc) {
    // humidity sensor data compensation
    double var_H;
    var_H = (((double)fine_temperature) - 76800.0);
    var_H = (humidity_adc - (((double)calib_H4) * 64.0 + ((double)calib_H5) / 16384.0 * var_H)) *
            (((double)calib_H2) / 65536.0 *
             (1.0 + ((double)calib_H6) / 67108864.0 * var_H *
                        (1.0 + ((double)calib_H3) / 67108864.0 * var_H)));
    var_H = var_H * (1.0 - ((double)calib_H1) * var_H / 524288.0);
    if (var_H > 100.0)
        var_H = 100.0;
    else if (var_H < 0.0)
        var_H = 0.0;
    return var_H;
}

void initializeCompensation(int file_descriptor) {
    // Calibration of sensor data reading and parameter initialization
    uint8_t calData0[25];
    uint8_t calData1[7];

    /* read calibration data */
    i2c_smbus_read_i2c_block_data(file_descriptor, CALIB_DATA0_START_ADDR, CALIB_DATA0_LENGTH, calData0);
    i2c_smbus_read_i2c_block_data(file_descriptor, CALIB_DATA1_START_ADDR, CALIB_DATA1_LENGTH, calData1);

    /* trimming parameters */
    calib_T1 = calData0[1] << 8 | calData0[0];
    calib_T2 = calData0[1] << 8 | calData0[0];
    calib_T3 = calData0[1] << 8 | calData0[0];

    calib_P1 = calData0[7] << 8 | calData0[6];
    calib_P2 = calData0[9] << 8 | calData0[8];
    calib_P3 = calData0[11] << 8 | calData0[10];
    calib_P4 = calData0[13] << 8 | calData0[12];
    calib_P5 = calData0[15] << 8 | calData0[14];
    calib_P6 = calData0[17] << 8 | calData0[16];
    calib_P7 = calData0[19] << 8 | calData0[18];
    calib_P8 = calData0[21] << 8 | calData0[20];
    calib_P9 = calData0[23] << 8 | calData0[22];

    calib_H1 = calData0[24];
    calib_H2 = calData1[1] << 8 | calData1[0];
    calib_H3 = calData1[2];
    calib_H4 = calData1[3] << 4 | (calData1[4] & 0xF);
    calib_H5 = calData1[5] << 4 | (calData1[4] >> 4);
    calib_H6 = calData1[6];
}

double calculateSeaLevelPressure(double station_pressure) {
    return station_pressure * exp((-MOLAR_MASS * GRAVITY * -LOCAL_HEIGHT_ASL) / (GAS_CONSTANT * STANDARD_TEMP));
}

void readSensorData(int file_descriptor) {
    uint8_t data_block[8];
    int32_t temp_raw = 0;
    int32_t pressure_raw = 0;
    int32_t humidity_raw = 0;
    double station_pressure = 0.0;

    while (1) {
        // Sleep for 1 second for demonstration purposes.
        sleep(1);

        // Check if data is ready to read
        if ((i2c_smbus_read_byte_data(file_descriptor, STATUS_REG) & 0x9) != 0) {
            printf("%s\n", "Error: Data not ready");
            continue;
        }

        // Read data registers
        i2c_smbus_read_i2c_block_data(file_descriptor, DATA_START_ADDR, DATA_LENGTH, data_block);

        // Wake up and take the next reading
        i2c_smbus_write_byte_data(file_descriptor, CTRL_MEASURE, 0x25);

        // Get raw temperature
        temp_raw = (data_block[3] << 16 | data_block[4] << 8 | data_block[5]) >> 4;

        // Get raw pressure
        pressure_raw = (data_block[0] << 16 | data_block[1] << 8 | data_block[2]) >> 4;

        // Get raw humidity
        humidity_raw = data_block[6] << 8 | data_block[7];

        // Calculate and print compensated temperature
        printf("Temperature: %.2f°C  ", compensateTemperature(temp_raw));

        station_pressure = compensatePressure(pressure_raw) / 100.0;

        // Calculate and print compensated pressure
        printf("Station Pressure: %.2f hPa, Sea Level Pressure: %.2f  ", station_pressure,
               calculateSeaLevelPressure(station_pressure));

        // Calculate and print compensated humidity
        printf("Humidity: %.2f %%rH\n", compensateHumidity(humidity_raw));
    }
}


int main(void) {
    int file_descriptor = 0;

    // Open I2C communication
    if ((file_descriptor = open(DEVICE_PATH, O_RDWR)) < 0) {
        perror("Unable to open I2C device");
        return 1;
    }

    // Set up I2C slave
    if (ioctl(file_descriptor, I2C_SLAVE, DEVICE_ADDRESS) < 0) {
        perror("Unable to configure I2C slave device");
        close(file_descriptor);
        return 2;
    }

    // Check device identifier
    if (i2c_smbus_read_byte_data(file_descriptor, CHIP_IDENTIFIER) != 0x60) {
        perror("Device identifier error");
        close(file_descriptor);
        return 3;
    }

    // Soft reset for device initialization
    i2c_smbus_write_byte_data(file_descriptor, SOFT_RESET, 0xB6);
    usleep(50000);

    // Read and set calibration parameters
    initializeCompensation(file_descriptor);

    // Set humidity oversampling x1
    i2c_smbus_write_byte_data(file_descriptor, CTRL_HUMIDITY, 0x1);

    // Turn off the filter
    i2c_smbus_write_byte_data(file_descriptor, CONFIG_REG, 0);

    // Set forced mode for measurement and take the 1st reading
    i2c_smbus_write_byte_data(file_descriptor, CTRL_MEASURE, 0x25);

    // Start reading sensor data
    readSensorData(file_descriptor);

    return 0;
}

