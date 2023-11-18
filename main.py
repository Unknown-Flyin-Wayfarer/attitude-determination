'''from machine import Pin
from bangbang.bangbang import BangBang 
import time
from codes import *
from command import OBC

led = Pin(25, Pin.OUT) 
detumble = BangBang() 

#Start detumbling
detumble.start()
time.sleep_ms(100)
del detumble 
obc = OBC()

#Set the reference frame for finding orientaion

#Estimate the orientaion and adjust for correction

#Set mag field as ref vector 

#Begin transmission 
obc.ExecuteCommand(str(TRIAD_ORIENT))
while True:
    led.off()
    try:
        obc.receiveCommand()
    except Exception as e:
        print(e)
    time.sleep_ms(100)
'''

########### code for attitude determination and and visualizing it in openGL by sending quarternion through serial port. Testing!!

from imu import CalibrateIMU
from triad.triad import TRIAD
import npy as np
import math

def quat_to_euler(q):
    yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    pitch = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    yaw   -= -0.13  # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
    roll  *= 180.0 / math.pi
    return roll, pitch, yaw

imuData = CalibrateIMU()
orient = TRIAD()

while True:
    mag, gyro, accel,timestamp = imuData.getCalibDataWithoutLowPass() 
    #print("Raw Magnetometer Data:", mag)
    #print("Raw Gyroscope Data:", gyro)
    #print("Raw Accelerometer Data:", accel)
    #print("Timestamp:", timestamp)

    DCM = orient.estimate(w1=accel, w2=mag)
    q = np.chiaverini(DCM)
    print("w",q[0],"w","a",q[1],"a","b",q[2],"b","c",q[3],"c")    #use "\n" at the end to break the line inorder to feed it to the openGL simulation code
    euler = quat_to_euler(q)
    print("Euler :",euler,"\n")