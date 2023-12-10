#include "bme280.h"

#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "calibration.h"


int main(void) {
    int device_file = 0;
    uint8_t dataBlock[8];
    int32_t temperature_integer = 0;
    int32_t pressure_integer = 0;
    int32_t humidity_integer = 0;
    double station_pressure = 0.0;

    /* open i2c comms */
    if ((device_file = open(DEVICE_PATH, O_RDWR)) < 0) {
        perror("Unable to open i2c device");
        return 1;
    }

    /* configure i2c slave */
    if (ioctl(device_file, I2C_SLAVE, DEVICE_ID) < 0) {
        perror("Unable to configure i2c slave device");
        close(device_file);
        return 2;
    }

    /* check our identification */
    if (i2c_smbus_read_byte_data(device_file, DEVICE_IDENT) != 0x60) {
        perror("device ident error");
        close(device_file);
        return 3;
    }


    return 0;
}
