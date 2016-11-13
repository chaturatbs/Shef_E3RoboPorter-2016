import smbus
import time

SELF_TEST_X_GYRO = 0x00
SELF_TEST_Y_GYRO = 0x01
SELF_TEST_Z_GYRO = 0x02

SELF_TEST_X_ACCEL = 0x0D
SELF_TEST_Y_ACCEL = 0x0E
SELF_TEST_Z_ACCEL = 0x0F


bus = smbus.SMBus(1)
#I2C address for MMA7660
addr = 0x68

#write(0x68, 0x6B, 0);
#    write(0x68, 0x6A, 0);  /
#    write(0x68, 0x37, 0x02);
#    writeMag(0x0A, 0x12); //

try:
    bus.write_byte_data(addr, 0x6B, 0x00) #Initialize Power Manager Register to 0
    bus.write_byte_data(addr, 0x6A, 0x00) #Disable master I2C, enable value is 0x20, so disable it should be just bit 5 = 0, since I don't use other bits and they are all 0.
    bus.write_byte_data(addr, 0x37, 0x02) #Enable bypass mode
    #bus.write_byte_data(addr, 0x07, 0x01) #Setup magnetic sensor to measure contiuously.
except IOError, err:
    print err

while True:
    try:
      x = bus.read_byte_data(addr,0x00)
      y = bus.read_byte_data(addr,0x01)
      z = bus.read_byte_data(addr,0x02)
      tr = bus.read_byte_data(addr,0x03)
      print x, y, z, tr
      time.sleep(0.25)
    except:
      print 'exiting...'
      break