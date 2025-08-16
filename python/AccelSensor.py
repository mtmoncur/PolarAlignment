import machine
import utime
import ustruct
import math
import utils

LSM303_ACCEL_ADDR = 0x19
LSM303_MAG_ADDR = 0x1E

# Registers (see data sheet for full explanations)
ACCEL_CTRL_REG1_A = 0x20 # power state
ACCEL_CTRL_REG2_A = 0x21
ACCEL_CTRL_REG3_A = 0x22
ACCEL_CTRL_REG4_A = 0x23
ACCEL_CTRL_REG5_A = 0x24
ACCEL_CTRL_REG6_A = 0x25

ACCEL_OUT_X_L_A = 0x28
ACCEL_OUT_X_H_A = 0x29
ACCEL_OUT_Y_L_A = 0x2A
ACCEL_OUT_Y_H_A = 0x2B
ACCEL_OUT_Z_L_A = 0x2C
ACCEL_OUT_Z_H_A = 0x2D

MAG_CRB_REG_M = 0x01
MAG_MR_REG_M  = 0x02
MAG_OUT_X_H_M = 0x03


# Other constants
SENSITIVITY_2G = 2.0 / 2**11  # (g/LSB)
SENSITIVITY_4G = 4.0 / 2**11  # (g/LSB)
SENSITIVITY_8G = 8.0 / 2**11  # (g/LSB)
SENSITIVITY_16G = 16.0 / 2**11  # (g/LSB)
EARTH_GRAVITY = 9.80665     # Earth's gravity in [m/s^2]


MAG_MULT_1_3 = 1.3 / 2**11

X_G_low = -0.9962
X_G_high = 0.9963
Y_G_low = -0.9710
Y_G_high = 1.0231
Z_G_low = -1.2198
Z_G_high = 0.8363

X_G_bias = EARTH_GRAVITY * (X_G_high + X_G_low)/2
Y_G_bias = EARTH_GRAVITY * (Y_G_high + Y_G_low)/2
Z_G_bias = EARTH_GRAVITY * (Z_G_high + Z_G_low)/2

def adjustAccelSingle(inp, val1, val2):
    ran = val2 - val1
    m = 2/ran
    return m*(inp-val1)-1

# def adjustAccel(x, y, z):
#     x = adjustAccelSingle(x, X_G_low, X_G_high)
#     y = adjustAccelSingle(y, Y_G_low, Y_G_high)
#     z = adjustAccelSingle(z, Z_G_low, Z_G_high)
#     return x, y, z

def adjustAccel(x, y, z):
    return x-X_G_bias, y-Y_G_bias, z-Z_G_bias

X_M_low = -0.3878
X_M_high = 0.3771
Y_M_low = -0.3142
Y_M_high = 0.3117
Z_M_low = -0.3612
Z_M_high = 0.3383

X_M_bias = (X_M_high + X_M_low)/2
Y_M_bias = (Y_M_high + Y_M_low)/2
Z_M_bias = (Z_M_high + Z_M_low)/2

def adjustMag(x, y, z):
    return x-X_M_bias, y-Y_M_bias, z-Z_M_bias

class LSM303:
    def __init__(self, sda, scl, channel):
        assert sda in [0,2,8,10,12,14,16,18,20,26], 'Wrong sda'
        assert scl in [1,3,7,11,13,15,17,19,21,27], 'Wrong scl'

        sda = machine.Pin(sda)
        scl = machine.Pin(scl)
        self._i2c = machine.I2C(channel, sda=sda, scl=scl, freq=100000)

        # turn on accelerator
        utils.reg_write(self._i2c, LSM303_ACCEL_ADDR, ACCEL_CTRL_REG1_A, 0x57)
        data = utils.reg_read(self._i2c, LSM303_ACCEL_ADDR, ACCEL_CTRL_REG1_A)
        print(data, type(data))
        print(data[0], type(data[0]))
        assert data[0] == 0x57, f'expected data == 0x57, got {data}'
        print(hex(data[0]))

        # turn on magnetometer
        utils.reg_write(self._i2c, LSM303_MAG_ADDR, MAG_MR_REG_M, 0x00)

        HIRES_ON = 1 << 3
        SCALE_2G = 0 << 4
        # SCALE_4G = 1 << 4
        # SCALE_8G = 2 << 4
        # SCALE_16G = 3 << 4
        flags = HIRES_ON | SCALE_2G
        utils.reg_write(self._i2c, LSM303_ACCEL_ADDR, ACCEL_CTRL_REG4_A, flags)

        utime.sleep(0.5)

    def readAccel(self):
        accel_data = utils.reg_read(self._i2c, LSM303_ACCEL_ADDR, ACCEL_OUT_X_L_A, 6)
        accel = ustruct.unpack('<hhh', accel_data)

        # Convert to 12-bit values by shifting unused bits.
        x, y, z = (accel[0] >> 4, accel[1] >> 4, accel[2] >> 4)

        # Convert measurements to [m/s^2]
        x *= SENSITIVITY_2G * EARTH_GRAVITY
        y *= SENSITIVITY_2G * EARTH_GRAVITY
        z *= SENSITIVITY_2G * EARTH_GRAVITY
        return adjustAccel(x, y, z)
        return x, y, z

    def readMag(self):
        mag_data = utils.reg_read(self._i2c, LSM303_MAG_ADDR, MAG_OUT_X_H_M, 6)
        # per the datasheet, these values are ordered x, z, y and they are big-endian
        x, y, z = ustruct.unpack('>hhh', mag_data)
        x *= MAG_MULT_1_3
        y *= MAG_MULT_1_3
        z *= MAG_MULT_1_3
        return adjustMag(x, y, z)
        return x, y, z