'''
        Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
    http://www.electronicwings.com
'''
# added offset handling

import smbus            #import SMBus module of I2C
from time import sleep          #import
import math
import paho.mqtt.publish as publish
import paho.mqtt.subscribe as subscribe
import paho.mqtt.client as mqtt
import pickle
host = "localhost"




#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("$SYS/#")


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
  #  print(msg.topic+" "+str(msg.payload))
    if(msg.topic == "IMU/offset"): # store offsets which are send from phone
        print("IMU/offset message: ", msg.topic)
        print(" IMU/offset value: ", msg.payload, "degree")
        with open('maueOffsetValues.pkl', 'wb') as f:  # Python 3: open(..., 'wb')
            pickle.dump([msg.topic,msg.payload], f)
        print("retrieved", retrievedMsg, " :", retrievedValue)  
        print("class value", type(retrievedMsg))  
    if(msg.topic == "Phone/conected"): # sent msg to phone with offset values when phone is connected as indicated in topic Phone/connected
        print("phone connected")
        with open('maueOffsetValues.pkl') as f:  # Python 3: open(..., 'rb')
             retrievedMsg, retrievedValue,  = pickle.load(f)
        publish.single("Raspi/Offsets", retrievedValue, hostname=host)

# read offset file and send offsets

with open('maueOffsetValues.pkl') as f:  # Python 3: open(..., 'rb')
             retrievedMsg, retrievedValue,  = pickle.load(f)

print("Offset ready to send offset", retrievedValue)           
publish.single("Raspi/Offsets", retrievedValue, hostname=host)

client = mqtt.Client()


client.on_connect = on_connect
client.on_message = on_message

client.connect(host, 1883, 60)


# Phone/conected: phone has connected and needs stored offsets
# IMU/offset: offset button at phone was pressed=> save new values in file maueOffsetValues.pkl and send offset values to phone
#
#

client.subscribe([("IMU/pitch/offset", 2), ("IMU/roll/offset", 2),
("IMU/pitch",2),("IMU/roll",2),("IMU/offset",2), ("Phone/conected", 2)])

# The loop_start() starts a new thread, that calls the loop method at regular intervals for you. It also handles re-connects automatically.
# details at http://www.steves-internet-guide.com/loop-python-mqtt-client/
client.loop_start()

print (" Reading Data of Gyroscope and Accelerometer")

while True:

    #Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    #Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    #Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0

    Gx = gyro_x/131.0*180/math.pi
    Gy = gyro_y/131.0*180/math.pi
    Gz = gyro_z/131.0*180/math.pi

    #Roll and Pitch (self made)
    roll = math.atan(acc_y /  math.sqrt(acc_x*acc_x + acc_z*acc_z))
    # pitch: (nose up/down, about Y axis)
    pitch = math.atan(acc_x / math.sqrt(acc_y*acc_y + acc_z*acc_z))

    roll_deg = roll * (180.0/math.pi)
    pitch_deg = pitch * (180.0/math.pi)

    msgs = [{'topic': "IMU/pitch", 'payload': str(pitch_deg)},
        {'topic': "IMU/roll", 'payload': str(roll_deg)}]

#    print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)     
    #print ("roll=%.2f" %roll_deg, "tpitch=%.2f" %pitch_deg)
    # publish.single(topic="IMU/pitch", payload=str(pitch_deg), hostname=host)
    #mosquitto_pub -h localhost -t "test/message" -m str(pitch_deg)
    publish.multiple(msgs, hostname=host)
    
    sleep(0.5)
   
