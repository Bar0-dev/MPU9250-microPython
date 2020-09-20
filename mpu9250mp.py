import machine
from addrDict import addrDict
from time import sleep_ms

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
    
    #returns byte array pulled from register
    def byteRead(self,regAddr,size): #register name form datasheet, register size in bytes
        self.regAddr = regAddr
        self.size = size
        return self.i2c.readfrom_mem(self.address,addrDict[regAddr],size)

    #returns int value represented by 16 bit register (two complement signed value)
    def read16bitSigned(self,regAddr,swap=False): #register name form datasheet, bytes swaped, set True if yes
        self.regAddr = regAddr
        buff = self.i2c.readfrom_mem(self.address,addrDict[regAddr],2)
        if swap == False:
            buff = (buff[0] << 8) | buff[1]
        else:
            buff = (buff[1] << 8) | buff[0]
        #detect sign and preform two complement and sign change
        if buff & 0x8000:
            buff = -((buff^0xFFFF)+1)
        return buff

    #returns string of bits from byte
    def binRead(self,regAddr): #register name form datasheet
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

    #set sample divider
    def setSRD(self,setting):
        self.setting = setting
        #set fchoice_b to 00 to activate DLPF
        self.byteWrite('GYRO_CONFIG',0x00)
        self.byteWrite('SMPLRT_DIV',setting)

    #set Digital Low Pass Filter according to datasheet 
    def setDLPF(self,setting): #set accordingly to datasheet for ex. 0 - 250 Hz, 4 - 20 Hz etc.
        self.setting = setting
        #set fchoice_b to 00 to activate DLPF
        self.byteWrite('GYRO_CONFIG',0x00)
        self.byteWrite('CONFIG',0x00 | setting)

    #set gyro full scale range
    def setGyroRange(self,setting):
        self.setting = setting
        if setting == 250:
            setting = 0b00
        elif setting == 500:
            setting = 0b01
        elif setting == 1000:
            setting = 0b10
        elif setting == 2000:
            setting = 0b11
        else:
            return "Wrong input. Please use: 250, 500, 1000 or 2000"
        self.byteWrite('GYRO_CONFIG',0x00 | setting << 3)

    #set accelerometer full scale range
    def setAccelRange(self,setting):
        self.setting = setting
        if setting == 2:
            setting = 0b00
        elif setting == 4:
            setting = 0b01
        elif setting == 8:
            setting = 0b10
        elif setting == 16:
            setting = 0b11
        else:
            return "Wrong input. Please use: 2, 4, 8, 16"
        self.byteWrite('ACCEL_CONFIG',0x00 | setting << 3)

    #set Accelerometer Digital Low Pass Filter
    def setAccelDLPF(self,setting): #set accordingly to datasheet for ex. 0 - 218.1 Hz, 4 - 21.2 Hz etc.
        self.setting = setting
        #set accel_fchoice_b to 0 to activate accel DLPF
        self.byteWrite('ACCEL_CONFIG_2',0x00 | setting)

    #read temperature in oC
    def readTemp(self):
        temp = self.read16bitSigned('TEMP_OUT_H') #put only higher address, method takes 2 bytes
        #conversion from LSB to oC
        temp = (temp-1/333.87)+21
        return temp

    #returns list of signed 16bit int readings form all XYZ accelerometer data accordingly
    def readAccelRaw(self):
        data = [self.read16bitSigned('ACCEL_XOUT_H'),self.read16bitSigned('ACCEL_YOUT_H'),self.read16bitSigned('ACCEL_ZOUT_H')]
        return data

    #returns list with normalized readings from accelerometer - unit [G]
    def readAccel(self):
        #read accel setting to normalize value
        setting = (self.byteRead('ACCEL_CONFIG',1)[0] >> 3) & 0b00000011
        if setting == 3:
            setting = 16
        elif setting == 2:
            setting = 8
        elif setting == 1:
            setting = 4
        elif setting == 0:
            setting = 2
        data = self.readAccelRaw()
        for index in range(0, len(data)):
            #normalize in range of accelerometer set range
            data[index] = 2*setting*((data[index])-(-32768))/(32767-(-32768))-setting
        return data
    
    #returns list of signed 16bit int readings form all XYZ gyro data accordingly
    def readGyroRaw(self):
        data = [self.read16bitSigned('GYRO_XOUT_H'),self.read16bitSigned('GYRO_YOUT_H'),self.read16bitSigned('GYRO_ZOUT_H')]
        return data

    #returns list with normalized gyro readings - unit [DPS]
    def readGyro(self):
        #read gyro setting to normalize value
        setting = (self.byteRead('GYRO_CONFIG',1)[0] >> 3) & 0b00000011
        if setting == 3:
            sensitivity = 16.4
            setting = 2000
        elif setting == 2:
            sensitivity = 32.8
            setting = 1000
        elif setting == 1:
            sensitivity = 65.5
            setting = 500
        elif setting == 0:
            sensitivity = 131
            setting = 250
        data = self.readGyroRaw()
        for index in range(0, len(data)):
            #normalize in range of gyro set range
            data[index] = 2*setting*((data[index]/sensitivity)-(-32768))/(32767-(-32768))-setting
        return data

    #set magnetometer with bypass mode
    def setMag(self):
        self.byteWrite('USER_CTRL',0) #disable i2c master mode
        self.byteWrite('INT_PIN_CFG',0b00000010) #enable bypass mode
        sleep_ms(100)
        self.i2c.writeto_mem(addrDict['MAG_ADDRESS'],addrDict['CTRL'],bytes([0b00010010])) #set magnetometer to send 16 bit values in 8Hz period(continous mode 1)
        #read sensitivity adjustment coefficients
        self.ASAX = self.i2c.readfrom_mem(addrDict['MAG_ADDRESS'],addrDict['ASAX'],1)[0]
        self.ASAY = self.i2c.readfrom_mem(addrDict['MAG_ADDRESS'],addrDict['ASAY'],1)[0]
        self.ASAZ = self.i2c.readfrom_mem(addrDict['MAG_ADDRESS'],addrDict['ASAZ'],1)[0]
        self.byteWrite('INT_PIN_CFG',0) #disable bypass mode
        self.byteWrite('I2C_MST_CTRL',0b01000000) #enable i2c master mode
        self.byteWrite('USER_CTRL',0b00100000) #enable wait for data to write to external sensor data register
        self.byteWrite('I2C_SLV0_ADDR',0b10000000 | addrDict['MAG_ADDRESS']) #set slave 0 in read mode with magnetometer address
        self.byteWrite('I2C_SLV0_REG',0x03) #set starting address to read from magnetometer
        self.byteWrite('I2C_SLV0_CTRL',0b10000111) #enable i2c slave 0, pull 7 bytes

    #read raw data from magnetometer
    def readMagRaw(self):
        data = [self.read16bitSigned('EXT_SENS_DATA_00',True)*((self.ASAX-128)*0.5/128+1),self.read16bitSigned('EXT_SENS_DATA_02',True)*((self.ASAY-128)*0.5/128+1),self.read16bitSigned('EXT_SENS_DATA_04',True)*((self.ASAZ-128)*0.5/128+1)]
        return data

    #returns list with normalized magnetometer readings - unit [uT]
    def readMag(self):
        setting = 4900
        data = self.readMagRaw()
        for index in range(0, len(data)):
            #normalize in range of magnetometer maximum range
            data[index] = 2*setting*((data[index])-(-32768))/(32767-(-32768))-setting
        return data
