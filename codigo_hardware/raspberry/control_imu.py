import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

# Configuración del sensor
mpu = MPU9250(
    address_ak=AK8963_ADDRESS, 
    address_mpu_master=MPU9250_ADDRESS_68, # Dirección I2C por defecto
    bus=1, 
    gfs=GFS_250, 
    afs=AFS_2G, 
    mfs=AK8963_BIT_16, 
    mode=AK8963_MODE_C100HZ
)

mpu.configure()

print("Lectura de IMU iniciada. Presiona Ctrl+C para detener.")

try:
    while True:
        # Leer todas las variables
        accel = mpu.read_accelerometer_data() # [x, y, z] en G
        gyro = mpu.read_gyroscope_data()      # [x, y, z] en grados/s
        
        print(f"Accl X: {accel[0]:.2f} | Gyro Z: {gyro[2]:.2f}")
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nLectura finalizada.")