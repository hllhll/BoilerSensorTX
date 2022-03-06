#include <Arduino.h>
#include <EEPROM.h>
// Current sketch static configuration
#define HC12_DEFAULT_BAUDRATE 9600
#define HC12_SET_PIN 3
#define MAX_SENSORS 6
#define RESOLUTION 12 // 9 bit-0.5°C
#define RELEARN_SENSORS_THRESHOLD 2 // If x new sensors are found that are not in ROM, re-learn all of them
#define SENSOR_1WIRE_PIN 14
// OneWireNoResistor-1.0 ; On some devices it can possibly make the PULLUP resistor redunded.
// NOTE: Does not work on all devices, and not in all cases, see https://wp.josh.com/2014/06/23/no-external-pull-up-needed-for-ds18b20-temp-sensor/
// Arduino pro mini 8MHZ 3.3 - Doesn't work, still need pull-up.
#include <OneWire.h>

//Dallas Temperature slim code configuration
#define REQUIRESALARMS false
#define REQUIRESNEW false
#include <DallasTemperature.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

bool hc12_is_atmode = false;
bool avr_sleeping = false;


int counter = 0;
char buf[100];


// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library

OneWire  oneWire(SENSOR_1WIRE_PIN);  // (a 4.7K resistor is necessary)
DallasTemperature sensors(&oneWire);

unsigned int tx_counter;
byte sensor_addresses[MAX_SENSORS][8];
byte sensor_types[MAX_SENSORS];
byte known_sensors;

char binstr_buf[17];

byte nv_sensors;
#include "Sensor.hpp"

void start_at(){
  if(!hc12_is_atmode){
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(10);
    hc12_is_atmode = true;
  }
}

void stop_at(){
  if(hc12_is_atmode){
    digitalWrite(HC12_SET_PIN, HIGH);
    hc12_is_atmode = false;
  }
}

void locahEcho(){
  //Local echo!
  // Arduino TX Connected to HC12 rx overrides TTL device TX!, Arduino driver is stronger then TTL.
  // Echoing rx data on tx might resolve this, thus HC12 will see the data on it's rx (+pc echo)
  int ch;
  stop_at();
  ch = Serial.read();
  while(ch!=-1){
    Serial.write(ch);
    ch = Serial.read();
  }
  start_at();
}

void hc12_sleep(){
  start_at();
  Serial.println("AT+SLEEP");
  Serial.flush();
  // "AT+SLEEP CR,CL" - 10 chars, +start stop bits, 10*10 - 100 bits @ 9600 => 10.4ms
  delay(11);
  stop_at(); // Sleep will be in effect after leaving AT mode.
}

void hc12_sleep_exit(){
  start_at();
  stop_at();
  delay(5);
}

void clear(){
  tx_counter = 0;
  known_sensors = 0;
}

// Wakeup interrupt, TODO: needed?
ISR(WDT_vect) {
  cli();
  wdt_disable();
  // Serial.println("wakeup!");
  sei();
}




void myWatchdogEnable() {  // turn on watchdog timer; interrupt mode every 2.0s
  cli();
  MCUSR = 0;
  WDTCSR |= B00011000;
  WDTCSR = B01000111;
  sei();
}

void avr_sleep(){
  avr_sleeping = true;
  wdt_reset();
  myWatchdogEnable();
  Serial.println("Slp");
  sleep_mode();
}

int stam = 0;
void mesure_and_send(){
  Serial.println("Req temp...");
  sensors.requestTemperatures();
  Serial.println("Temp reqd.");
  Serial.println(stam++);
  for(int i=0; i<sensors.getDS18Count(); i++)
  {
    Serial.print("Sensor #");
    Serial.print(i);
    Serial.print(" ");
    Serial.print(sensors.getTempCByIndex(i));
    Serial.print("c ");

    /*Serial.print("Res: ");
    Serial.print(sensors.getResolution());*/

    Serial.print("Raw: ");
    DeviceAddress deviceAddress;
	  if (!sensors.getAddress(deviceAddress, i)) {
      Serial.print(i);
      Serial.println(" NF");
  		return ;
  	}
	  sensors.getTempC((uint8_t*) deviceAddress);
    int16_t raw_temp = sensors.getTemp(deviceAddress);
    Serial.print(raw_temp);
    Serial.print(" ");
    itoa(raw_temp, binstr_buf, 2);
    Serial.print( binstr_buf );

    Serial.println();
  }

}

void setup_hc12(){
  pinMode(HC12_SET_PIN, OUTPUT);
  digitalWrite(HC12_SET_PIN, HIGH);
  Serial.begin(HC12_DEFAULT_BAUDRATE);
}



void eeprom_save(){
  Serial.println("EEPROM: Saving");
  /*EEPROM.put(EEPROM_SENSORS_NUM_ADDR, nv_sensors);
  EEPROM.put(EEPROM_SENSORS_LIST_ADDR,  nv_sensors_list);*/
}

byte find_sensors() {
  // This will reset dallas library
  sensors.setOneWire(&oneWire);
  sensors.begin();
  return sensors.getDS18Count();
  /*
  byte sensors = 0;
  byte addr[8];

  ds.reset_search();

  while ( ds.search(addr)) {
          Serial.print("ROM =");
      for( i = 0; i < 8; i++) {
        Serial.write(' ');
        Serial.print(addr[i], HEX);
      }
    

    memset(sensor_addresses[sensors], 0, sizeof(sensor_addresses[sensors]));
    sensor_types[sensors] = 0xff;

    if(check_sensor_crc(addr)){
      memcpy(sensor_addresses[sensors], addr, sizeof(sensor_addresses[sensors]));
      sensor_types[sensors] = get_sensor_type(addr);
    }
    
    sensors++;
  }
  ds.reset_search();
  return;
*/
}




void configure_sensors(){
  sensors.setResolution(RESOLUTION);
  // TRUE : function requestTemperature() etc will 'listen' to an IC to determine whether a conversion is complete
  //sensors.setCheckForConversion(true);
  sensors.setCheckForConversion(false);

  // sets the value of the waitForConversion flag
  // TRUE : function requestTemperature() etc returns when conversion is ready
  // FALSE: function requestTemperature() etc returns immediately (USE WITH CARE!!)
  //        (1) programmer has to check if the needed delay has passed
  //        (2) but the application can do meaningful things in that time
  //sensors.setWaitForConversion(false); // use with isConversionComplete ?
  sensors.setWaitForConversion(true); // use with isConversionComplete ?
}

SensorSnapshot snapshot_nvram;
SensorSnapshot snapshot_detected;
SensorSnapshot snapshot_current;

void setup(void) {
  setup_hc12();
  hc12_sleep();

  set_sleep_mode(SLEEP_MODE_PWR_SAVE);

  EEPROM.begin();
  clear();

  // Load known sensors address from NVRAM
  FromEEPROM(snapshot_nvram);

  // Find active sensors on bus - sensors object is now populated
  FromDallas(sensors, snapshot_detected);

  byte available_from_nvram = MarkActive(snapshot_nvram, snapshot_detected);

  // Find how many new sensors, if > RELEARN_SENSORS_THRESHOLD, reset them.
  byte new_sensors_count = snapshot_detected.count - available_from_nvram;
  Serial.print("Found ");
  Serial.print(new_sensors_count);
  Serial.print(" new sensors");

  if(new_sensors_count> RELEARN_SENSORS_THRESHOLD){
    nv_sensors = s_count;

  }else{
    // Either 0 or .. RELEARN_SENSORS_THRESHOLD)
    // Copy found sensors to NVRAM


    // Don't waist EEPROM writes
    if(new_sensors_count>0){
      eeprom_save();
    }
  }

  //sensors.getAddress();
  //byte diff_count
  
  // #define RELEARN_SENSORS_THRESHOLD 2 // If x new sensors are found that are not in ROM, re-learn all of them
  //known_sensors = rewrite_sensors_to_nvram();
  //eeprom_save()''

  configure_sensors();
}


void loop() {
  if(!avr_sleeping){
    // Goto sleep
    hc12_sleep();
    //stop_at(); // Leave AT so SLEEP takes effect- NO! DO NOT LEAVE AT, Current consumption is few uA lower
    avr_sleep();
  }else{
    avr_sleeping = !avr_sleeping;
    hc12_sleep_exit();
    Serial.print("ON/Data\n");
    Serial.flush();
    //exit HC12 sleep
    mesure_and_send();
    delay(1000); // Stay on 1 S
    // Next loop - device goes into sleep
  }
}



/*
ROM = 28 D8 39 96 F0 1 3C B0
  Chip = DS18B20
  Data = 1 38 1 55 5 7F A5 A5 66 A4  CRC=A4
  Temperature = 19.50 Celsius, 67.10 Fahrenheit
ROM = 28 FF 64 1E F 84 81 F2
  Chip = DS18B20
  Data = 1 58 1 55 0 7F FF C 10 FF  CRC=FF
  Temperature = 21.50 Celsius, 70.70 Fahrenheit*/


void check_availability(){
  //bool DallasTemperature::isConnected(const uint8_t* deviceAddress) {
}



/*
ROM = 28 D8 39 96 F0 1 3C B0
  Chip = DS18B20
  Data = 1 30 1 55 5 7F A5 A5 66 8E  CRC=8E
  Temperature = 19.00 Celsius, 66.20 Fahrenheit
ROM = 28 25 91 96 F0 1 3C 85
  Chip = DS18B20
  Data = 1 30 1 55 5 7F A5 A5 66 8E  CRC=8E
  Temperature = 19.00 Celsius, 66.20 Fahrenheit
  */
 
 /*
void setup(void) {
  Serial.begin(9600);
}

void loop(void) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");
}

/*
void setup() {
  // put your setup code here, to run once:
  Serial.begin(HC12_DEFAULT_BAUDRATE);
}

void loop() {
  sprintf(buf, "%s%d\n", str_fixed, counter++);
  Serial.print(buf);
  delay(1000);
  // put your main code here, to run repeatedly:
}
*/