import machine
from addrDict import addrDict

class MPU9250:
    def __init__(self,address,sclPin,sdaPin):
        self.address = address
        self.sclPin = sclPin
        self.sdaPin = sdaPin
        #Create I2C protocol object
        self.i2c = machine.I2C(-1,machine.Pin(sclPin),machine.Pin(sdaPin))
        #Scan for devices
        scan = self.i2c.scan()
        if address in scan:
            print("Device found under the provided address: " + hex(address))
        else:
            print("Couldn't find the device under the provided address! Please check wiring.")

    #returns byte from register
    def byteRead(self,regAddr,size): #register name form datasheet, register size in bytes
        self.regAddr = regAddr
        self.size = size
        return self.i2c.readfrom_mem(self.address,addrDict[regAddr],size)

    #returns int value represented by 16 bit register (two complement signed value)
    def read16bitSigned(self,regAddr): #register name form datasheet
        self.regAddr = regAddr
        buff = self.i2c.readfrom_mem(self.address,addrDict[regAddr],2)
        buff = (buff[0] << 8) | buff[1]
        #detect sign and preform two complement and sign change
        if buff & 0x8000:
            buff = -((buff^0xFFFF)&0x0001)
        return buff

    #returns int value represented by 16 bit register unsigned
    def read16bitUnsigned(self,regAddr): #register name form datasheet
        self.regAddr = regAddr
        buff = self.i2c.readfrom_mem(self.address,addrDict[regAddr],2)
        buff = (buff[0] << 8) | buff[1]
        return buff

    #returns bin representation of a byte from register
    def binRead(self,regAddr): #register name form datasheet, register size in bytes
        self.regAddr = regAddr
        return bin(self.i2c.readfrom_mem(self.address,addrDict[regAddr],1)[0])

    #writes a byte to register, always writes one byte = 8-bit buffer, returns True if write was successful
    def byteWrite(self,regAddr,buff):
        self.regAddr = regAddr
        self.buff = buff
        self.i2c.writeto_mem(self.address,addrDict[regAddr],bytes([buff]))
        if self.byteRead(regAddr,1) == bytes([buff]):
            return True
        else:
            return False

    #takes existing setting on register and changes only pointed bits ex. reg = 0b0100, setting = 0b0011 makes new reg = 0b0111 (works on 8 bit registry only)
    def setRegister(self,regAddr,setting):
        self.regAddr = regAddr
        self.setting = setting
        buff = self.byteRead(regAddr,1)
        setting = buff[0] | setting
        #write new setting
        self.byteWrite(regAddr,setting)

    #set DLPF according to datasheet 
    def setDLPF(self,setting):
        self.setting = setting
        #set fchoice_b to 00 to activate DLFP
        self.setRegister('GYRO_CONFIG',0x00)
        self.setRegister('CONFIG',)

    def readTemp(self):
        temp = 