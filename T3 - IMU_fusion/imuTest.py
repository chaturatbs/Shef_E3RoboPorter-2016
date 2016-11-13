from mpu9250 import MPU9250

imu = MPU9250('X', 0)

print(imu.accel.xyz)
print(imu.gyro.xyz)
print(imu.mag.xyz)
print(imu.temperature)
print(imu.accel.z)
