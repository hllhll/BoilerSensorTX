#include <Arduino.h>
#include <EEPROM.h>
// Current sketch static configuration


//#define ACTIVATE_TEST_MODULE
//#define RF_DONT_LEAVE_SLEEP // For testing locally only
#define JUST_TEST_RF_ON_BOOT
#define DEBUG_PRINTS

#define HC12_DEFAULT_BAUDRATE 9600
#define HC12_SET_PIN 3
#define MAX_SENSORS 6
byte txframe[MAX_SENSORS+1+1];
#define RESOLUTION 12 // 9 bit-0.5°C
#define RELEARN_SENSORS_THRESHOLD 2 // If x new sensors are found that are not in ROM, re-learn all of them
#define SENSOR_1WIRE_PIN 14

//Amount of bits sent in UART TX of bytes bytes
#define UART_BITS_IN_BYTES(bytes) ((8+1+1)*(bytes))
// How meny MS does it take to TX bytes amount of bytes
#define DELAY_TX_DATA_MS(bytes) ((UART_BITS_IN_BYTES(bytes)/HC12_DEFAULT_BAUDRATE)*1000*1.25)

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

#include "Sensor.hpp"

SensorSnapshot snapshot_nvram;
SensorSnapshot snapshot_detected;
SensorSnapshot snapshot_current;

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

char CRC8(const byte *data,int length) 
{
   byte crc = 0x00;
   byte extract;
   byte sum;
   for(int i=0;i<length;i++)
   {
      extract = *data;
      for (char tempI = 8; tempI; tempI--) 
      {
         sum = (crc ^ extract) & 0x01;
         crc >>= 1;
         if (sum)
            crc ^= 0x8C;
         extract >>= 1;
      }
      data++;
   }
   return crc;
}

void start_at(){
  if(!hc12_is_atmode){
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(20);
    hc12_is_atmode = true;
  }
}

void stop_at(){
  if(hc12_is_atmode){
    digitalWrite(HC12_SET_PIN, HIGH);
    hc12_is_atmode = false;
    delay(20);
  }
}

void locahEcho(){
  //Local echo!
  // Arduino TX Connected to HC12 rx overrides TTL device TX!, Arduino driver is stronger then TTL.
  // Echoing rx data on tx might resolve this, thus HC12 will see the data on it's rx (+pc echo)
  int ch;
  //stop_at();
  ch = Serial.read();
  while(ch!=-1){
    Serial.write(ch);
    ch = Serial.read();
  }
  //start_at();
}

/// ********For ENTERING sleep mode, 45ms after setting low, 35 after command, 35 after setting high sufficient to exit sleep
void hc12_sleep(){
  start_at();
  delay(25); // another 25 to complete delay to 45
  Serial.println("AT+SLEEP");
  Serial.flush();
  delay(35);
  stop_at(); // Sleep will be in effect after leaving AT mode.
  delay(15); // another 15 to complete delay to 35
}

/// ********For leaving sleep mode, 20ms after setting low, 20 after setting high sufficient to exit sleep
void hc12_sleep_exit(){
#ifndef RF_DONT_LEAVE_SLEEP
  start_at();
  stop_at();
#endif
}

void transmit(const byte *buf, size_t size){
  Serial.flush();
#ifndef RF_DONT_LEAVE_SLEEP
  hc12_sleep_exit();
#endif
  Serial.write(buf, size);
  Serial.flush();
  // If not enugth delay hc12 will:
  //   1. enter AT mode and will dismiss tx data
  //   2. enter sleep mode and will kill tx data.
  delay( DELAY_TX_DATA_MS(size) );
  delay(50); //Extra delay for... what? This is needed...
  hc12_sleep();
}

void clear_stats(){
  tx_counter = 0;
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
#ifdef DEBUG_PRINTS
  Serial.println("Slp");
#endif
  sleep_mode();
  
}

byte mesurement_to_byte(float &mesurement){
  // from 10dC (-10)
  // Accuracy 0.5 deg (*2)
  if(mesurement<=10 || mesurement>=100){
    return 0;
  }
  // cap 0xff 
  return (byte)( (mesurement-10)*2 );
}

int stam = 0;
void mesure_and_send(){
  memset(txframe, 0, sizeof(txframe) );
  txframe[0] = snapshot_current.count;
#ifdef DEBUG_PRINTS
  Serial.println("Req temp...");
#endif
  sensors.requestTemperatures();
#ifdef DEBUG_PRINTS
  Serial.println("Temp reqd.");
  Serial.println(stam++);
#endif
  byte *cur_addr;

  // Iterate all sensors that are `considered installed`
  // Only checkout one that are available
  // but transmit all of them (dead ones with the value 0).
  for(int i=0; i<snapshot_current.count; i++)
  {
    if(snapshot_current.sensors[i].available){
      cur_addr = snapshot_current.sensors[i].address;
      //float reading = sensors.getTempCByIndex(i);
      float reading = sensors.getTempC(cur_addr);
      //txframe[1+GetIndexByAddr(cur_addr, snapshot_current) ] = mesurement_to_byte(reading);
      txframe[1+i ] = mesurement_to_byte(reading);
#ifdef DEBUG_PRINTS
      Serial.print("Sensor #");
      Serial.print(i);
      Serial.print(" ");
      Serial.print( reading );
      Serial.print("c ");
#endif
      /*
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

      Serial.println();*/
    }
  }
  // Checksum
  txframe[1+txframe[0]] = CRC8(txframe, 1+txframe[0]);
  //transmit("01234567890ABCDEFGHIJKLMNOPQRSTUVWXYZ", 37);
  transmit(txframe, 1+txframe[0]+1);
}

void setup_hc12(){
  //TODO: DEBUG
#ifdef JUST_TEST_RF_ON_BOOT
  Serial.begin(HC12_DEFAULT_BAUDRATE);
  Serial.println("HELLO WORLD!!!");
  Serial.flush();
  delay(20);
#endif  
  pinMode(HC12_SET_PIN, OUTPUT);
  digitalWrite(HC12_SET_PIN, HIGH);
  Serial.begin(HC12_DEFAULT_BAUDRATE);
}



void eeprom_save(){
#ifdef DEBUG_PRINTS
  Serial.println("EEPROM: Saving");
#endif
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

void setup(void) {
  setup_hc12();
#ifdef ACTIVATE_TEST_MODULE
  return;
#endif
  hc12_sleep();

  set_sleep_mode(SLEEP_MODE_PWR_SAVE);

  EEPROM.begin();
  clear_stats();

  // Load known sensors address from NVRAM
  FromEEPROM(snapshot_nvram);

  // Find active sensors on bus - sensors object is now populated
  sensors.begin();
  FromDallas(sensors, snapshot_detected);
#ifdef DEBUG_PRINTS
  Serial.print("Dallas lib: ");
  Serial.println(snapshot_detected.count);
#endif
  // Take the remembered sensors, and mark which of them is currently available
  // Returns the amount of available sensors from memory
  byte available_from_nvram = MarkActive(snapshot_nvram, snapshot_detected);

  // New sensors = detected - (remembered&detected)
  // Find how many new sensors, if > RELEARN_SENSORS_THRESHOLD, reset them.
  byte new_sensors_count = snapshot_detected.count - available_from_nvram;
#ifdef DEBUG_PRINTS
  Serial.print("Found ");
  Serial.print(new_sensors_count);
  Serial.print(" new sensors");
#endif
  if(new_sensors_count>=RELEARN_SENSORS_THRESHOLD){
#ifdef DEBUG_PRINTS
    Serial.println(">=THRESHOLD");
#endif
    // Delete all sensors, learn the new ones
    snapshot_current = snapshot_detected;
    if(snapshot_current.count>0){
      eeprom_save();
    }
  }else{
#ifdef DEBUG_PRINTS
    Serial.println("<THRESHOLD");
#endif
    // Either 0 or .. RELEARN_SENSORS_THRESHOLD)
    // Learn the new_sensors_count
    snapshot_current = snapshot_nvram;
    Merge(snapshot_current, snapshot_detected);

    // Don't waste EEPROM writes
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

#ifdef ACTIVATE_TEST_MODULE
void test_hc12_safe_sleep(){
    delay(400);
    start_at();
    delay(100);
    Serial.println("AT+SLEEP");
    Serial.flush();
    delay(100);
    delay(500 );
    stop_at();
    delay(400 );
}

void test_hc12_safe_wakeup(){
    Serial.flush();
    delay(600);
    start_at();
    delay(300);
    stop_at();
    delay(600 );
}

void test_hc12(){
    hc12_is_atmode = false;
    Serial.begin(HC12_DEFAULT_BAUDRATE);

    test_hc12_safe_wakeup();

    //Seen on RX side
    Serial.println("TEST MODULE");
    Serial.flush();
    delay(500);
    
    /*
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(300);
    Serial.println("1)AT-Shoudn't be seen on RX");
    Serial.flush();
    // Should show an error
    locahEcho();
    delay( 1000 );
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    */

    test_hc12_safe_sleep();
    Serial.println("2)Slp-houldn't be seen on RX");
    Serial.flush();
    delay(500 );

    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(100);
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    Serial.println("3)Should show on RX(100,0)");
    Serial.flush();
    delay(500);

    test_hc12_safe_sleep();
    Serial.println("4)Slp-houldn't be seen on RX");
    Serial.flush();
    delay(500 );

    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(10);
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    delay(10);
    Serial.println("5)Should show on RX(10,10)");
    Serial.flush();
    delay(500 );

    test_hc12_safe_sleep();
    Serial.println("6)Slp-houldn't be seen on RX");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(50 );
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    delay(20 );
    Serial.println("7)Should show on RX(50,20)");
    Serial.flush();
    delay(500 );

    test_hc12_safe_sleep();
    Serial.println("7.1)Slp-houldn't be seen on RX");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(50 );
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    delay(0 );
    Serial.println("7.2)Should show on RX(50,0)");
    Serial.flush();
    delay(500 );


    /// ********For leaving sleep mode, 20ms after setting low, 20 after setting high sufficient to exit sleep
    /// According to here https://github.com/RobertRol/SimpleHC12/blob/master/simpleHC12.h
    ///    Datasheet has specified rise and low times for SET pin...
    ///    https://statics3.seeedstudio.com/assets/file/bazaar/product/HC-12_english_datasheets.pdf
    ///    setLowTime{50},setHighTime{90},cmdTime{100},
    test_hc12_safe_sleep();
    Serial.println("7.3)Slp-houldn't be seen on RX");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(20 );
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    delay(20 );
    Serial.println("7.4)Should show on RX(20,20)");
    Serial.flush();
    delay(500 );

    test_hc12_safe_sleep();
    Serial.println("8)Slp-houldn't be seen on RX");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(100 );
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    delay(100 );
    Serial.println("9)Should show on RX(100,100)");
    Serial.flush();
    delay(500 );


    test_hc12_safe_wakeup();
    Serial.println("10)Baseline for go-to-sleep Seen on RX(0,0,0)");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    Serial.println("AT+SLEEP");  
    Serial.flush();
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    Serial.println("11)Slp-houldn't be seen on RX(0,0,0)");
    Serial.flush();

    delay(500);


    test_hc12_safe_wakeup();
    Serial.println("11.1)Baseline for go-to-sleep Seen on RX(0,0,20)");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    Serial.println("AT+SLEEP");  //Accepted by device
    Serial.flush();
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    delay(20); //Note that the device takes a while to leave AT mode...
    //NOTE: I Think it first leaves 
    Serial.println("11.2)Slp-houldn't be seen on RX(0,0,20)");
    Serial.flush();
    delay(500);

    Serial.print("11.2-11.3)Between tests-Should not show");
    Serial.flush();


    test_hc12_safe_wakeup();
    Serial.println("11.4)Baseline for go-to-sleep Seen on RX(20,0,20)");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(20); //Note that the device takes a while to leave AT mode...
    Serial.println("AT+SLEEP");  //Accepted by device
    Serial.flush();
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    delay(35); //Note that the device takes a while to leave AT mode...
    Serial.println("11.5)Slp-houldn't be seen on RX(20,0,20)");
    Serial.flush();
    delay(500);
    Serial.print("11.5-11.6)Between tests-Should not show");
    Serial.flush();

    test_hc12_safe_wakeup();
    Serial.println("11.6)Baseline for go-to-sleep Seen on RX(20,20,35)");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(20); //Note that the device takes a while to leave AT mode...
    while(Serial.available()) Serial.read();
    Serial.println("AT+SLEEP");  //Accepted by device
    Serial.flush();
    delay(20); 
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    locahEcho(); //Should show OK+SLEEP
    delay(35); //Note that the device takes a while to leave AT mode...
    Serial.println("11.7)Slp-houldn't be seen on RX(20,20,20)");
    Serial.flush();
    delay(500);
    Serial.print("11.7-12)Between tests-Should not show");
    Serial.flush();



    test_hc12_safe_wakeup();
    Serial.println("11.8)Baseline for go-to-sleep Seen on RX(20,35,35)");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(20); //Note that the device takes a while to leave AT mode...
    while(Serial.available()) Serial.read();
    Serial.println("AT+SLEEP");  //Accepted by device
    Serial.flush();
    delay(35); 
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    locahEcho(); //Should show OK+SLEEP
    delay(35); //Note that the device takes a while to leave AT mode...
    Serial.println("11.9)Slp-houldn't be seen on RX(20,35,20)");
    Serial.flush();
    delay(500);
    Serial.print("11.9-12)Between tests-Should not show");
    Serial.flush();


    /////////// This is okay., Maybe we could go lower
    /// ********For ENTERING sleep mode, 45ms after setting low, 35 after command, 35 after setting high sufficient to exit sleep
    /// According to here https://github.com/RobertRol/SimpleHC12/blob/master/simpleHC12.h
    ///    Datasheet has specified rise and low times for SET pin...
    ///    https://statics3.seeedstudio.com/assets/file/bazaar/product/HC-12_english_datasheets.pdf
    ///    setLowTime{50},setHighTime{90},cmdTime{100},
    test_hc12_safe_wakeup();
    Serial.println("11.10)Baseline for go-to-sleep Seen on RX(45,35,35)");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(45); //Note that the device takes a while to leave AT mode...
    while(Serial.available()) Serial.read();
    Serial.println("AT+SLEEP");  //Accepted by device
    Serial.flush();
    delay(35); 
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    locahEcho(); //Should show OK+SLEEP
    delay(35); //Note that the device takes a while to leave AT mode...
    Serial.println("11.11)Slp-houldn't be seen on RX(45,35,20)");
    Serial.flush();
    delay(500);
    

    test_hc12_safe_wakeup();
    Serial.println("12)Baseline for go-to-sleep Seen on RX(20,100,20)");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(20);
    Serial.println("AT+SLEEP");
    Serial.flush();   
    delay(100);
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    delay(20);
    Serial.println("13)Slp-houldn't be seen on RX(20,100,20)");
    Serial.flush();
    delay(500);


    test_hc12_safe_wakeup();
    Serial.println("14)Baseline for go-to-sleep Seen on RX(40,100,40)");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(40);
    Serial.println("AT+SLEEP");
    Serial.flush();   
    delay(100);
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    delay(40);
    Serial.println("15)Slp-houldn't be seen on RX(40,100,40)");
    Serial.flush();
    delay(500);

    // Looks like it goes to sleep even with 0,0,0; Do sanity without flush:
    // Okay - No flush doesn't put device into sleep
    test_hc12_safe_wakeup();
    Serial.println("16)Baseline for go-to-sleep Seen on RX(0,0,0 noflush)");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    Serial.println("AT+SLEEP");
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    Serial.println("17)Slp-houldn't be seen on RX(0,0,0 noflush)");
    Serial.flush();
    delay(500);


    // Testing lower delayt attempts for going to sleep
    // NOPE
    test_hc12_safe_wakeup();
    Serial.println("18)Baseline for go-to-sleep Seen on RX(30,15,15)");
    Serial.flush();
    delay(100);
    digitalWrite(HC12_SET_PIN, LOW);//Moves device into at mode
    delay(30); //Note that the device takes a while to leave AT mode...
    while(Serial.available()) Serial.read();
    Serial.println("AT+SLEEP");  //Accepted by device
    Serial.flush();
    delay(15); 
    digitalWrite(HC12_SET_PIN, HIGH);//Stops AT mode;
    locahEcho(); //Should show OK+SLEEP
    delay(15); //Note that the device takes a while to leave AT mode...
    Serial.println("19)Slp-houldn't be seen on RX(30,15,15)");
    Serial.flush();
    delay(500);
}
#endif

void loop() {
#ifdef ACTIVATE_TEST_MODULE
  test_hc12();
  return;
#endif
  //if(false){
  if(!avr_sleeping){
    // Goto sleep
    hc12_sleep();
    //stop_at(); // Leave AT so SLEEP takes effect- NO! DO NOT LEAVE AT, Current consumption is few uA lower
    avr_sleep();
  }else{
    avr_sleeping = !avr_sleeping;
#ifdef DEBUG_PRINTS    
    Serial.print("ON/Data\n");
#endif
    Serial.flush();
    //Serial.println("ASDASDASDASDSADSASDASDASDASDASDSSDSaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaADAS");
    //exit HC12 sleep
    mesure_and_send();

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