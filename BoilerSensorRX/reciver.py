SERIAL_PORT = "/dev/ttyS0"
SERIAL_BAUDRATE = 9600
MAX_SENSORS = 6
TRANSMITTER_ID = 0xCAFE
TRANSMITTER_ID_H = (TRANSMITTER_ID & 0xff00) >> 8
TRANSMITTER_ID_L = (TRANSMITTER_ID & 0xf8)

# https://dev.to/tardisgallifrey/raspberry-pi-gpio-4-ways-45do
"""
Set up GPIO pin for use. Write "4" into the export file at /sys/class/gpio/export.
Establish pin as input or output. Write "out" or "in" at /sys/class/gpio/gpio4/direction.
Finally, turn pin on or off. Write "1" or "0" at /sys/class/gpio/gpio4/value.

sudo echo 2 > /sys/class/gpio/export
sudo echo out > /sys/class/gpio/gpio2/direction
sudo echo 0 > /sys/class/gpio/gpio2/value  #Into AT Mode

cu -l /dev/ttyS0 -s 9600

AT

sudo echo 1 > /sys/class/gpio/gpio2/value  # Out of AT Mode

or use a library,
or follow this https://dev.to/tardisgallifrey/raspberry-pi-gpio-4-ways-45do

"""

# from https://github.com/leech001/hass-mqtt-discovery
from ha_mqtt_device import *
import hc12
import RPi.GPIO as GPIO
# HC12_SET_PIN for me is GPIO2 on BCM, Pin #3 on the board pinout


from pickletools import uint8
import signal
import crc8
import time   # For the demo only
import serial
import argparse

def signal_handler(signal, frame):
    global interrupted
    interrupted = True

signal.signal(signal.SIGINT, signal_handler)

interrupted = False
# configure the serial connections (the parameters differs on the device you are connecting to)

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

boiler_tx = Device(
    name="BoilerSensor", # Can't have spaces
    identifiers=("%x" % TRANSMITTER_ID),
    sw_version=0,
    model="BoilerSensorTX0",
    manufacturer="HLL")

temp_sensors = []

# https://stackoverflow.com/questions/51731313/cross-platform-crc8-function-c-and-python-parity-check
def crc8(data):
    crc = 0
    for i in range(len(data)):
        byte = data[i]
        for b in range(8):
            fb_bit = (crc ^ byte) & 0x01
            if fb_bit == 0x01:
                crc = crc ^ 0x18
            crc = (crc >> 1) & 0x7f
            if fb_bit == 0x01:
                crc = crc | 0x80
            byte = byte >> 1
    return crc

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
                unique_id = ("%x-%i" % (TRANSMITTER_ID, i) ),
                force_update = True  ## Fore home assistant MQTT, Make sure u toggle on force_update
                                     ## so last_update would update on every msg queued
            )
        )

def clear_rx():
    global ser
    ser.read_all()

def do_loop():
    global mqtt_client, temp_sensors, ser
    while True:
        if interrupted:
            print("Gotta go")
            ser.close()
            break

        mqtt_client.loop()

        id_and_sensor_count = ser.read_until(size=2)  # Read identifier + active sensors
        #id_and_sensor_count = ser.read_until(size=3)  # Read identifier + active sensors
        if len(id_and_sensor_count)!=2:
            #print("bad len or no RX, expected 3, got=%d" % len(id_and_sensor_count))
            clear_rx()
            continue

        if(id_and_sensor_count[0] != TRANSMITTER_ID_H or \
           id_and_sensor_count[1]&0xf8 != TRANSMITTER_ID_L ):
           print("bad id")
           clear_rx()
           continue
        
        # ID Match

        sensors_count = id_and_sensor_count[1]&0x7
        
        """
        curpos = 2
        sensors_count = id_and_sensor_count[curpos]
        curpos+=1
        if(sensors_count>MAX_SENSORS):
            print("Bad sensor count")
            clear_rx()
            continue
        """
        #Sensor count within range
        verify_sensor_initialized(sensors_count)

        read_payload_size = sensors_count + 1 # +1 CRC8
        sensors_data = ser.read_until(size = read_payload_size)
        if(len(sensors_data)!=read_payload_size):
            print("Bad expected size")
            clear_rx()
            continue

        #TODO: Fix CRC check instead of sum
        #hash = crc8.crc8()
        full_buf = id_and_sensor_count + sensors_data
        #hash = sum(full_buf[:-1]) & 0xff
        hash = crc8(full_buf[:-1]) & 0xff
        #hash.update(full_buf[:-1])
        if int(hash)!=int.from_bytes(full_buf[-1:], "big"):
            print("crc8 error")
            #print("crc result: %s" % hash.hexdigest())
            print("crc result: %d" % hash)
            print("Sum: %d"% ((full_buf[0]+full_buf[1]+full_buf[2]+full_buf[3]+full_buf[4])&0xff))
            clear_rx()
        for i in range(sensors_count):
            temp_sensors[i].send(byte_to_mesurment(sensors_data[i]))
        print("ok")

def do_on_disconnect():
    global mqtt_client
    mqtt_client.connect("127.0.0.1", 1883, 10)

def do_setup():
    global mqtt_client, ser, SERIAL_PORT, SERIAL_BAUDRATE
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=SERIAL_BAUDRATE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.8
    )

    mqtt_client.on_disconnect = do_on_disconnect
    mqtt_client.connect("127.0.0.1", 1883, 10)

    print("Connected to MQTT")

def parse_arguments():
    parser = argparse.ArgumentParser(description='Recive invormation from BoilerSensorTX and transmit over MQTT in HomeAssistant-compatible manner')
    parser.add_argument("-mqtt", required=False, default="127.0.0.1:1883", help="MQTT host:port")
    parser.add_argument("-s", "--survey", action="store_true", help="Perform a site-survey when BoilerSensorTX is flashed with site-survey firmware")
    return parser.parse_args()

import datetime
stats_map = {}
def get_pings(rf: hc12):
    # Example output:
    # {8: 19, 7: 17, 6: 12, 5: 20, 4: 13, 3: 1}
    """
    #define TOTAL_ITERATION_TIME 800
    #define PING_COUNT 20
    #define POWER_START 8
    #define POWER_END 6
    #define NUM_POWER_STEPS (POWER_START-POWER_END+1)
    #define TIME_PER_BAUDRATE ((TOTAL_ITERATION_TIME*PING_COUNT*1.2*NUM_POWER_STEPS)+2000) // ~20*800*1.1 17600
    """
    TOTAL_ITERATION_TIME = 800
    PING_COUNT = 20
    POWER_START = 8
    POWER_END = 3
    BAUD_START = 0
    BAUD_END = 5
    NUM_POWER_STEPS = (POWER_START-POWER_END+1)
    TIME_PER_BAUDRATE = ((TOTAL_ITERATION_TIME*PING_COUNT*1.1*NUM_POWER_STEPS)+2000)
    TIME_PER_BAUDRATE -= 1000

    global stats_map
    budrate_iteration_start_time = datetime.datetime.now()
    cur_baudrate_index = 0
    while True:
        count=0
        if interrupted:
            print("Gotta go")
            rf.close()
            break
        
        rx = rf.read_until(size=5)
        if len(rx) == 5 and rx[0]==0xaa and rx[1] ==0x55:
            power = rx[3]
            tx_count = rx[2]
            count+=1
            if power not in stats_map.keys():
                stats_map[power] = 0
            stats_map[power]+=1    
            print( "Got ping #%d power %d tx_count #%x" % (count, power, tx_count) )
        #millis_delta = ((datetime.datetime.now() - budrate_iteration_start_time).microseconds) / 1000
        millis_delta = ((datetime.datetime.now() - budrate_iteration_start_time).total_seconds()) * 1000 
        if ( millis_delta >TIME_PER_BAUDRATE):
            print("Statistics for baudratre: %d" % rf._baudrate)
            print(stats_map)
            stats_map = {}
            budrate_iteration_start_time = datetime.datetime.now()
            # if this was the last index
            cur_baudrate_index+=1
            if cur_baudrate_index>BAUD_END:
                print("Scan finished")
                return
            rf.clear_rx()
            print("Switching to baudrate...")
            rf.set_baudrate(cur_baudrate_index)

def site_survey():
    global temp_sensors, ser

    rf = hc12.hc12(9600, "/dev/ttyS0", 3)  # HC12_SET_PIN for me is GPIO2 on BCM, Pin #3 on the board pinout
    rf.open()
    #rf.check_baudrate(9600)
    rf.find_baudrate()
    # Wait on the found baudrate for SYNC command from transmitter
    rf.mode_txrx()

    rf.set_baudrate(0) # Start with the slowest
    foundSync = False
    while not foundSync:
        rf.clear_rx()
        # rx = rf.read_until([0,0], size=2)
        rx = rf.read_until(size=5)
        # print(rx)
        if len(rx)==5 and rx==bytes([0]*5):
            print("Survey Scan SYNC")
            time.sleep(0.300)
            rf.clear_rx()
            get_pings(rf)
            rf.set_baudrate(0) # revert back to initial budrate for next sync

    while True:
        if interrupted or True:
            print("Gotta go")
            rf.close()
            break

if __name__=="__main__":
    args = parse_arguments()
    print(args)
    if args.survey or True:
        print("Survey")
        site_survey()
    do_setup()
    do_loop()
