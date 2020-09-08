from machine import I2C

mpu = {
    "scl":5,
    "sda":4,
    "addr":
}

def setup(adress,sda,scl,frequency):
    mpu=