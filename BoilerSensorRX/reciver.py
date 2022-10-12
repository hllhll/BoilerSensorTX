SERIAL_PORT = "/dev/ttyS0"
SERIAL_BAUDRATE = 9600

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

mqtt_client = mqtt.Client("user")

example_device = Device(identifiers="000102aabbcc",
    name="device1",
    sw_version=0,
    model="MODEL1",
    manufacturer="HLL")

temp_sensors = []

def do_loop():
    global mqtt_client, sensor_temp1, sensor_temp2, temp_sensors
    while True:
        mqtt_client.loop()
        out = ''
        s_payload_size = ser.read_until(size=1)
        if len(s_payload_size)==0:
            continue
        active_sensors = s_payload_size[0]
        # print("Reading %d bytes..." % active_sensors)
        #out = ser.read_until(size = payload_size)
        out = ser.read_until(size = 7)
        # while ser.inWaiting() > 0:
        #     out += ser.read(1)

        #TODO: Use sensor array, initialize based on the length/count value etc.
        if out != '':
            # print(out)
            # print("Sensor 0 %f" % byte_to_mesurment(out[0]))
            # print("Sensor 1 %f" % byte_to_mesurment(out[1]))
            sensor_temp1.send(byte_to_mesurment(out[0]))
            sensor_temp2.send(byte_to_mesurment(out[1]))

        if interrupted:
            print("Gotta go")
            ser.close()
            break

def do_setup():
    global mqtt_client,sensor_temp1,sensor_temp2
    #mqtt_client.on_connect = on_connect
    #mqtt_client.on_message = on_message
    #mqtt_client.username_pw_set("user", "pass")
    print("b4 connect")
    mqtt_client.connect("127.0.0.1", 1883, 10)

    sensor_temp1 = Sensor(
        mqtt_client,
        "Temperature 1",
        parent_device=example_device,
        unit_of_measurement="°C",
        topic_parent_level="temp1",
    )

    sensor_temp2 = Sensor(
        mqtt_client,
        "Temperature 2",
        parent_device=example_device,
        unit_of_measurement="°C",
        topic_parent_level="temp2",
    )
    #mqtt_client.loop_forever()
    print("after connect")

if __name__=="__main__":
    do_setup()
    do_loop()
