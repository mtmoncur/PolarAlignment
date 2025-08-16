#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

static const LSM303_ACCEL_ADDR = 0x19;
static const LSM303_MAG_ADDR = 0x1E;

// Registers (see data sheet for full explanations)
static const uint8_t ACCEL_CTRL_REG1_A = 0x20; // power state
static const uint8_t ACCEL_CTRL_REG2_A = 0x21;
static const uint8_t ACCEL_CTRL_REG3_A = 0x22;
static const uint8_t ACCEL_CTRL_REG4_A = 0x23;
static const uint8_t ACCEL_CTRL_REG5_A = 0x24;
static const uint8_t ACCEL_CTRL_REG6_A = 0x25;

static const uint8_t ACCEL_OUT_X_L_A = 0x28;
static const uint8_t ACCEL_OUT_X_H_A = 0x29;
static const uint8_t ACCEL_OUT_Y_L_A = 0x2A;
static const uint8_t ACCEL_OUT_Y_H_A = 0x2B;
static const uint8_t ACCEL_OUT_Z_L_A = 0x2C;
static const uint8_t ACCEL_OUT_Z_H_A = 0x2D;

static const uint8_t MAG_CRB_REG_M = 0x01;
static const uint8_t MAG_MR_REG_M  = 0x02;
static const uint8_t MAG_OUT_X_H_M = 0x03;


// Other constants
static constexpr uint8_t SENSITIVITY_2G = 2.0 / (1<<11);   // (g/LSB)
static constexpr uint8_t SENSITIVITY_4G = 4.0 / (1<<11);   // (g/LSB)
static constexpr uint8_t SENSITIVITY_8G = 8.0 / (1<<11);   // (g/LSB)
static constexpr uint8_t SENSITIVITY_16G = 16.0 / (1<<11); // (g/LSB)
static constexpr uint8_t EARTH_GRAVITY = 9.80665;          // Earth's gravity in [m/s^2]

/*******************************************************************************
 * Function Declarations
 */
int reg_write(i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes);

int reg_read(   i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes);

/*******************************************************************************
 * Function Definitions
 */

// Write 1 byte to the specified register
int reg_write(  i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;
    uint8_t msg[nbytes + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) {
        msg[i + 1] = buf[i];
    }

    // Write data to register(s) over I2C
    i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

    return num_bytes_read;
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(  i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;

    // Check to make sure caller is asking for 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Read data from register(s) over I2C
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

    return num_bytes_read;
}

/*******************************************************************************
 * Main
 */
int main() {

    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    float acc_x_f;
    float acc_y_f;
    float acc_z_f;

    // Pins
    const uint sda_pin = 16;
    const uint scl_pin = 17;

    // Ports
    i2c_inst_t *i2c = i2c0;

    // Buffer to store raw reads
    uint8_t data[6];

    // Initialize chosen serial port
    stdio_init_all();

    //Initialize I2C port at 400 kHz
    i2c_init(i2c, 400 * 1000);

    // Initialize I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    // Read device ID to make sure that we can communicate with the ADXL343
    reg_read(i2c, ADXL343_ADDR, REG_DEVID, data, 1);
    if (data[0] != DEVID) {
        printf("ERROR: Could not communicate with ADXL343\n");
        while (true);
    }

    // Read Power Control register
    reg_read(i2c, ADXL343_ADDR, REG_POWER_CTL, data, 1);
    printf("0xX\n", data[0]);

    // Tell ADXL343 to start taking measurements by setting Measure bit to high
    data[0] |= (1 << 3);
    reg_write(i2c, ADXL343_ADDR, REG_POWER_CTL, &data[0], 1);

    // Test: read Power Control register back to make sure Measure bit was set
    reg_read(i2c, ADXL343_ADDR, REG_POWER_CTL, data, 1);
    printf("0xX\n", data[0]);

    // Wait before taking measurements
    sleep_ms(2000);

    // Loop forever
    while (true) {

        // Read X, Y, and Z values from registers (16 bits each)
        reg_read(i2c, ADXL343_ADDR, REG_DATAX0, data, 6);

        // Convert 2 bytes (little-endian) into 16-bit integer (signed)
        acc_x = (int16_t)((data[1] << 8) | data[0]);
        acc_y = (int16_t)((data[3] << 8) | data[2]);
        acc_z = (int16_t)((data[5] << 8) | data[4]);

        // Convert measurements to [m/s^2]
        acc_x_f = acc_x * SENSITIVITY_2G * EARTH_GRAVITY;
        acc_y_f = acc_y * SENSITIVITY_2G * EARTH_GRAVITY;
        acc_z_f = acc_z * SENSITIVITY_2G * EARTH_GRAVITY;

        // Print results
        printf("X: %.2f | Y: %.2f | Z: %.2f\n", acc_x_f, acc_y_f, acc_z_f);

        sleep_ms(100);
    }
}


class LSM303 {
public:
    LSM303(uint sda_pin, uint scl_pin, i2c_inst_t* i2c) {
        //Initialize I2C port at 400 kHz
        i2c_init(i2c, 400 * 1000);

        // Initialize I2C pins
        gpio_set_function(sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);

        // turn on accelerator
        reg_write(i2c, LSM303_ACCEL_ADDR, ACCEL_CTRL_REG1_A, &data[0], 1);

    }
    
};