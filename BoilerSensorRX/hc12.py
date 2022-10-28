import RPi.GPIO as GPIO
import serial
import time

def tohex(s):
    return (":".join("{:02x}".format(ord(c)) for c in s))

class hc12(object):
    BAUDRATES = [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]
    def __init__(self, baudrate: int, serial_port: str, set_pin: int=None):
        self._baudrate = baudrate
        self._set_pin = set_pin
        self._serial_port = serial_port
        self._ser = None
        self._is_at = False
        #self._hw_set_pin = None

    def set(self, value):
        if value == GPIO.HIGH or \
           value == GPIO.LOW:
           pass
        elif value == False or value == 0:
            value = GPIO.LOW
        else:
            value = GPIO.HIGH
        GPIO.output(self._set_pin, value)

    def mode_at(self):
        if self._is_at:
            return
        self.set(0)
        time.sleep(0.060)
        self._is_at = True

    def mode_rxtx(self):
        if not self._is_at:
            return
        self.set(1)
        time.sleep(0.040)
        self._is_at = False

    def mode_txrx(self):
        self.mode_rxtx()

    def at_command(self, cmd):
        self._ser.flush()
        self._ser.write(cmd.encode('ascii'))
        self._ser.flush()
        time.sleep(0.40)
    
    def at_response_raw(self):
        self._ser.flush()
        return self._ser.read_all()

    def at_response(self):
        self._ser.flush()
        return self._ser.read_all().decode('ascii')


    def ser_open_baudrate(self, baudrate):
        self._ser = serial.Serial(
            port=self._serial_port,
            baudrate=baudrate,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.3
        )

    def open(self):
        self.ser_open_baudrate(self._baudrate)
        if self._set_pin is not None:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup( self._set_pin, GPIO.OUT)
    
    def close(self):
        self._ser.close()

    def check_baudrate(self, baud):
        self.mode_at()  # Verify, should already be in it
        if self._ser.isOpen():
            self._ser.close()
        self.ser_open_baudrate(baud)
        self.at_command("AT")
        resp = self.at_response_raw()
        if resp=="OK\r\n".encode('ascii'):
            return True
        else:
            return False
    
    def find_baudrate(self):
        for baudrate in hc12.BAUDRATES:
            if self.check_baudrate(baudrate):
                print(baudrate)
                return

    def read_until(self, terminator= serial.LF, size=None):
        # self._ser.write([0,0,0,0xff,0xff,0xff])
        self._ser.read_until(terminator=terminator, size=size)