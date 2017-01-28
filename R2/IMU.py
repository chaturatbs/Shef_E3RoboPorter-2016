#!/usr/bin/python

'''
  Here is some sample data from the MPU-9255 IMU.
  {'pressureValid': False, 
    'accelValid': True, 
    'temperature': 0.0, 
    'pressure': 0.0, 
    'fusionQPoseValid': True, 
    'timestamp': 1471314050076560L, 
    'compassValid': True, 
    'compass': (26.2247257232666, 36.678741455078125, -17.60536003112793), 
    'accel': (0.0107421875, 1.013427734375, -0.03369140625), 
    'humidity': 0.0, 'gyroValid': True, 
    'gyro': (0.00553120207041502, -0.0031295656226575375, 0.0031766612082719803), 
    'temperatureValid': False, 
    'humidityValid': False, 
    'fusionQPose': (0.6710100769996643, 0.6879799365997314, -0.20192061364650726, -0.18883132934570312), 
    'fusionPoseValid': True, 
    'fusionPose': (1.5989785194396973, -0.011157416738569736, -0.5601144433021545)}
'''

import sys, getopt
sys.path.append('.')
import RTIMU
import os.path
import time
import math  

SETTINGS_FILE = "RTIMULib.ini"  
   
s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)  

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):  
  print("IMU Init Failed")
  sys.exit(1)  
else:
  print("IMU Init Succeeded")

imu.setSlerpPower(0.02)  
imu.setGyroEnable(True)  
imu.setAccelEnable(True)  
imu.setCompassEnable(True)  
   
poll_interval = imu.IMUGetPollInterval()  
print("Recommended Poll Interval: %dmS\n" % poll_interval)

while True:
  if imu.IMURead():
    # x, y, z = imu.getFusionData()
    # print("%f %f %f" % (x,y,z))
    data = imu.getIMUData()
    fusionPose = data["fusionPose"]
    print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), 
        math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
    time.sleep(poll_interval*1.0/1000.0)