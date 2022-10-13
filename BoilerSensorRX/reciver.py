## Fore home assistant MQTT, Make sure u toggle on force_update 

SERIAL_PORT = "/dev/ttyS0"
SERIAL_BAUDRATE = 9600
MAX_SENSORS = 6
TRANSMITTER_ID = 0xCAFE
TRANSMITTER_ID_H = (TRANSMITTER_ID & 0xff00) >> 8
TRANSMITTER_ID_L = (TRANSMITTER_ID & 0xff)

# from https://github.com/leech001/hass-mqtt-discovery
from ha_mqtt_device import *

from pickletools import uint8
import signal
import time   # For the demo only
import serial

def signal_handler(signal, frame):
    global interrupted
    interrupted = True

signal.signal(signal.SIGINT, signal_handler)

interrupted = False
# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port=SERIAL_PORT,
    baudrate=SERIAL_BAUDRATE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.3
)

"""byte mesurement_to_byte(float &mesurement){
  // from 10dC (-10)
  // Accuracy 0.5 deg (*2)
  if(mesurement<=10 || mesurement>=100){
    return 0;
  }
  // cap 0xff 
  return (byte)( (mesurement-10)*2 );
}"""
def byte_to_mesurment(b: uint8):
    return (b/2.0)+10

# Try use https://github.com/leech001/hass-mqtt-discovery for forwording requests to the MQTT Server

mqtt_client = mqtt.Client("BoilserSensorRX")

boiler_tx = Device(identifiers=TRANSMITTER_ID,
    name="BoilerSensor", # Can't have spaces
    sw_version=0,
    model="BoilerSensorTX0",
    manufacturer="HLL")

temp_sensors = []

def verify_sensor_initialized(sensors_count):
    global temp_sensors, boiler_tx, mqtt_client
    for i in range(len(temp_sensors), sensors_count):
        temp_sensors.append(
            Sensor(
                mqtt_client,
                ("BoilerTemp %d" % i),
                parent_device=boiler_tx,
                unit_of_measurement="°C",
                topic_parent_level=("temp%d" % i),
                force_update = True
            )
        )

def do_loop():
    global mqtt_client, temp_sensors, ser
    while True:
        ser.read_all()
        mqtt_client.loop()
        out = ''
        id_and_sensor_count = ser.read_until(size=3)  # Read identifier + active sensors
        if len(id_and_sensor_count)>0:
            print("%x " % id_and_sensor_count[0])
        #id_and_sensor_count = ser.read_until(size=3)  # Read identifier + active sensors
        if len(id_and_sensor_count)!=3:
            print("bad len or no RX, expected 3, got=%d" % len(id_and_sensor_count))
            continue

        if(id_and_sensor_count[0] != TRANSMITTER_ID_H or \
           id_and_sensor_count[1] != TRANSMITTER_ID_L ):
           print("bad id")
           continue
        
        # ID Match
        
        curpos = 2
        sensors_count = id_and_sensor_count[curpos]
        curpos+=1
        if(sensors_count>MAX_SENSORS):
            print("Bad sensor count")
            continue

        #Sensor count within range
        verify_sensor_initialized(sensors_count)

        read_payload_size = sensors_count + 1 # +1 CRC8
        sensors_data = ser.read_until(size = read_payload_size)
        if(len(sensors_data)!=read_payload_size):
            print("Bad expected size")
            continue

        #TODO: Check CRC8

        for i in range(sensors_count):
            temp_sensors[i].send(byte_to_mesurment(sensors_data[i]))
        print("OK")

        if interrupted:
            print("Gotta go")
            ser.close()
            break

def do_setup():
    global mqtt_client
    mqtt_client.connect("127.0.0.1", 1883, 10)
    print("Connected to MQTT")

if __name__=="__main__":
    do_setup()
    do_loop()
