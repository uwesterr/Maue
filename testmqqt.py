'''
        Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
    http://www.electronicwings.com
'''
from time import sleep          #import
import math
import paho.mqtt.publish as publish
import paho.mqtt.subscribe as subscribe
import paho.mqtt.client as mqtt
import pickle


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("$SYS/#")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    if(msg.topic == "IMU/offset"):
        print("IMU/offset message: ", msg.topic)
        print(" IMU/offset value: ", msg.payload, "degree")
        with open('objs.pkl', 'wb') as f:  # Python 3: open(..., 'wb')
            pickle.dump([msg.topic,msg.payload], f)
        with open('objs.pkl') as f:  # Python 3: open(..., 'rb')
             retrievedMsg, retrievedValue,  = pickle.load(f)
        print("retrieved", retrievedMsg, " :", retrievedValue)     


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("192.168.4.1", 1883, 60)

client.publish("test/message","OFF paho ")#publish

client.subscribe([("IMU/pitch/offset", 2), ("IMU/roll/offset", 2),
("IMU/pitch",2),("IMU/roll",2),("IMU/offset",2)])



client.loop_forever()
