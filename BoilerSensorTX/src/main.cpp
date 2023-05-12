#include "LowPower.h"
#include <Arduino.h>
#include <EEPROM.h>

#ifndef LowPower_h // Still in expirimintation, if lowpower is included don't use thi
#include <avr/power.h>
#endif 

// Current sketch static configuration

// New: lower 3 bits used for count#
#define DEVICE_SIG ((unsigned short)0xCAFE)
//#define SURVEY_SCAN

//TODO: Don't forget to un-comment
//#define READ_TEMPS_WHILE_SLEEPING // This will send the temperatures as they were, up to AVR_SLEEP_TIME ago

//#define BENCHMARK_POWER
//#define ACTIVATE_TEST_MODULE
//#define RF_DONT_LEAVE_SLEEP // For testing locally only
//#define JUST_TEST_RF_ON_BOOT
//#define DEBUG_PRINTS
// Options 2, 4, 8 in seconds
//#define AVR_SLEEP_TIME 8000
#define AVR_SLEEP_TIME 8000

// in millis
//#define SAMPLE_INTERVAL 15000 // ==> real time ~22 @8s sleep
//#define SAMPLE_INTERVAL 120000L // ==> real time 4.5m?
//#define SAMPLE_INTERVAL 60000L // ==> real time ~70s
//#define SAMPLE_INTERVAL 75000L // ==> real time 90s?
#define SAMPLE_INTERVAL 95000L // ==> real time ?

#if SAMPLE_INTERVAL<AVR_SLEEP_TIME
  #error "Sample interval should be gt avr sleep time"
#endif

#ifdef READ_TEMPS_WHILE_SLEEPING
#if AVR_SLEEP_TIME < 1000
  #error "AVR_SLEEP_TIME Must be >= 1sec when READ_TEMPS_WHILE_SLEEPING"
#endif
#endif

// HC12 Official english guide
// http://www.hc01.com/downloads/HC-12%20english%20datasheets.pdf
// https://www.elecrow.com/download/HC-12.pdf
// https://www.datsi.fi.upm.es/docencia/Informatica_Industrial/DMC/HC-12_v2.3A.pdf

#if AVR_SLEEP_TIME!=2000 && \
    AVR_SLEEP_TIME!=1000 && \
    AVR_SLEEP_TIME!=4000 && \
    AVR_SLEEP_TIME!=8000 && \
    AVR_SLEEP_TIME!=500
#error "Bad AVR Sleep time"  
#endif


const unsigned long baudArray[] = {1200,2400,4800,9600,19200,38400,57600,115200};
const unsigned int baudArrayLen = 8;


#define HC12_ASSUME_CURRENT_BAUD_EQ_TARGET true
#define HC12_DEFAULT_BAUDRATE 9600
#define HC12_TARGET_BAUDRATE_IDX 4 //19200
#define HC12_TARGET_BAUDRATE (baudArray[HC12_TARGET_BAUDRATE_IDX])

#define HC12_TARGET_POWER 3

#define HC12_SET_PIN 3
#define MAX_SENSORS 6

#define RESOLUTION 12 // 9 bit-0.5°C
#define RELEARN_SENSORS_THRESHOLD 2 // If x new sensors are found that are not in ROM, re-learn all of them
#define SENSOR_1WIRE_PIN 14
#define SENSOR_POWER_PIN 5

//Amount of bits sent in UART TX of bytes bytes
#define UART_BITS_IN_BYTES(bytes) ((8+1+1)*(bytes))
// How meny MS does it take to TX bytes amount of bytes
#define DELAY_TX_DATA_MS(bytes) ((UART_BITS_IN_BYTES(bytes)/HC12_TARGET_BAUDRATE)*1000*1.25)

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
bool avr_sleeping = true; // Start with sending data

#include "Sensor.hpp"

SensorSnapshot snapshot_nvram;
SensorSnapshot snapshot_detected;
SensorSnapshot snapshot_current;

int counter = 0;
char buf[100];


const int cmdResBuffLen=60;
char cmdResBuff[cmdResBuffLen];



// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library

OneWire  oneWire(SENSOR_1WIRE_PIN);  // (a 4.7K resistor is necessary)
DallasTemperature sensors(&oneWire);

unsigned int tx_counter;

#include <util/atomic.h>
// send cmd to HC12 module
boolean hc12_cmd(const char cmd[]) {
    memset(cmdResBuff, 0, sizeof(cmdResBuff));
    Serial.print(cmd);
    Serial.flush();
    delay(300);
    
    // write response into buffer
    size_t i = 0;
    char input;
    
    while (Serial.available() && i<cmdResBuffLen-1) {
        input = Serial.read();
        // no interrupts allowed while writing to buffer
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            cmdResBuff[i] = input;
            i++;
        }
    }
    //Serial.println(cmdResBuff); // This causes bogus HC commands errors
    Serial.flush();
    return true;
}

bool led_next = HIGH;
void toggle_led(){
//#ifdef SURVEY_SCAN
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, led_next);
  led_next = led_next == HIGH ? LOW : HIGH;
//#endif
}

void blink(unsigned long d, int times){
  times*=2;
  for( ; times; times--){
    toggle_led();
    delay(d);
  }
}

// https://stackoverflow.com/questions/51731313/cross-platform-crc8-function-c-and-python-parity-check
uint8_t CRC8( uint8_t *addr, uint8_t len) {
      uint8_t crc=0;
      for (uint8_t i=0; i<len;i++) {
         uint8_t inbyte = addr[i];
         for (uint8_t j=0;j<8;j++) {
             uint8_t mix = (crc ^ inbyte) & 0x01;
             crc >>= 1;
             if (mix) 
                crc ^= 0x8C;
         inbyte >>= 1;
      }
    }
   return crc;
}

void start_at(){
  if(!hc12_is_atmode){
    pinMode(HC12_SET_PIN, OUTPUT);
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
    pinMode(HC12_SET_PIN, INPUT);
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
  delay(60); // another 25 to complete delay to 45 //Proved working :300,25
  Serial.print("AT+SLEEP");
  Serial.flush();
  delay(110);  // //Proved working :400,35
  stop_at(); // Sleep will be in effect after leaving AT mode. 
  delay(15); // another 15 to complete delay to 35
  Serial.end();
  pinMode(0, INPUT);
  pinMode(1, INPUT);
}

/// ********For leaving sleep mode, 20ms after setting low, 20 after setting high sufficient to exit sleep
void hc12_sleep_exit(){
#ifndef RF_DONT_LEAVE_SLEEP
  Serial.begin(baudArray[HC12_TARGET_BAUDRATE_IDX]);
  start_at();
  stop_at();
#endif
}

unsigned long sleeping_millis = 0;

void avr_sleep(){
  
  avr_sleeping = true;
#ifndef LowPower_h
  wdt_reset();
  myWatchdogEnable();
#endif
/*#ifdef DEBUG_PRINTS
  Serial.println("Slp");
  Serial.flush();
#endif*/
  sleeping_millis+=AVR_SLEEP_TIME;
#ifndef LowPower_h
  sleep_mode();  // POWER: ~0.8-0.7 ~ 0.005
#else
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
#endif
  
}

void transmit(const byte *buf, size_t size){
#ifdef DEBUG_PRINTS
  Serial.println("Sending...");
#endif
  /// Serial.flush();  //// THIS CAUSES EXCESS CURRENT - FROM NOW AND NEVER??
#ifndef RF_DONT_LEAVE_SLEEP
  hc12_sleep_exit();
#endif
  Serial.write(buf, size);
#ifdef BENCHMARK_POWER // ~120mA consumption while TX
  unsigned long tx_start = millis();
  while( (millis()-tx_start) < 1500 )
  {
   Serial.write(buf, size);
  }
#endif
  Serial.flush();
#ifdef DEBUG_PRINTS
  Serial.println("...Queued");
#endif
  // If not enugth delay hc12 will:
  //   1. enter AT mode and will dismiss tx data
  //   2. enter sleep mode and will kill tx data.
  //delay( DELAY_TX_DATA_MS(size) );
  delay(50); //Extra delay for... what? This is needed...
#ifdef BENCHMARK_POWER  // ~22.6 mA while idling RX
  delay(3000);
#endif
  hc12_sleep();
#ifdef BENCHMARK_POWER  // ~9.2 While hc12 sleeping?
  delay(5000);
#endif
}

void clear_stats(){
  tx_counter = 0;
}

#ifndef LowPower_h // Still in expirimintation, if lowpower is included don't use this
ISR(WDT_vect) {
  cli();
  wdt_disable();
  sei();
}
#endif

void myWatchdogEnable() {  // turn on watchdog timer; interrupt mode every 2.0s
  cli();
  MCUSR = 0;
  // 7    6    5    4    3  2     1    0
  //WDIF WDIE WDP3 WDCE WDE WDP2 WDP1 WDP0
  WDTCSR |= B00011000;
#if AVR_SLEEP_TIME==1000
  WDTCSR =  B01000110; // 1 Seconds
#elif AVR_SLEEP_TIME==2000
  WDTCSR =  B01000111; // 2 Seconds
  // DO NOT USE  wdt_enable!! It bricks!
  //wdt_enable(WDTO_4S);
#elif AVR_SLEEP_TIME==4000
  WDTCSR =  B01100000; // 4 Seconds
  //         --^--^^^
#elif AVR_SLEEP_TIME==8000
  WDTCSR =  B01100001; // 8 Seconds
#elif AVR_SLEEP_TIME==500
  WDTCSR =  B01000101; // 0.5 Seconds
#endif
  sei();
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

//ID + Sensors_count + <readings> + checksum
uint8_t txframe[1+MAX_SENSORS+1+1] = {
  (DEVICE_SIG & 0xFF00) >> 8,
  (DEVICE_SIG & 0xFF) & 0xF8,
  0
};
#define TXFRAME_SENSORCOUNT_POS 1
int txframe_pos=TXFRAME_SENSORCOUNT_POS;


/* TODO: Reduce number of bytes in TX,
1. Find use 4 signature bits to be sensor count
2. Use less bits for checksum or data (maybe parity?)*/
void mesure_and_send(){
  txframe_pos=TXFRAME_SENSORCOUNT_POS;
  txframe[txframe_pos] &= 0xF8;
  txframe[txframe_pos] |= snapshot_current.count & 0x7;
  txframe_pos++;

#ifndef READ_TEMPS_WHILE_SLEEPING
#ifdef BENCHMARK_POWER
  unsigned long start = millis();
  while( (millis()-start) < 3000  ) // Spend 3 seconds in re-gathering sensor data to check current
  {
    Serial.println("Rerequesting temps");
    sensors.requestTemperatures();
  }
#endif

  // Since we were in sleep, make sure dallas library was initiated
  // This should also re-activate idle SENSOR_1WIRE_PIN
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  //sensors.begin(); //NO! I expect to know the existing sensors within each cycle.
  //sensors.requestTemperatures(); Done by loop() function asyncronusly.

#endif
/*#ifdef DEBUG_PRINTS
  Serial.println("Temp reqd.");
  Serial.println(stam++);
#endif*/
  byte *cur_addr;

  // Iterate all sensors that are `considered installed`
  // Only checkout one that are available
  // but transmit all of them (dead ones with the value 0).
  for(int i=0; i<snapshot_current.count; i++)
  {
    if(snapshot_current.sensors[i].available){
      cur_addr = snapshot_current.sensors[i].address;
      //float reading = sensors.getTempCByIndex(i);
      sensors_wait_conversions();
      float reading = sensors.getTempC(cur_addr);
      txframe[txframe_pos] = mesurement_to_byte(reading);
/*#ifdef DEBUG_PRINTS
      Serial.print("Sensor #");
      Serial.print(i);
      Serial.print(" ");
      Serial.print( reading );
      Serial.print("c ");
      Serial.print(" Mesurment: ");
      Serial.println( txframe[txframe_pos ] );
      Serial.flush();
#endif*/
      txframe_pos++;
    }
  }
  // "Trun off" 1-wire
  pinMode(SENSOR_1WIRE_PIN, INPUT);
  // Turn off sensor power
  pinMode(SENSOR_POWER_PIN, INPUT);
  // Checksum
  // I Don't know why I cant get CRC8 to work, just used ADD()
  txframe[txframe_pos] = CRC8(txframe, txframe_pos);
  txframe_pos++;
  /*
  Serial.print( txframe[0], 16 ); Serial.print( ' ' );
  Serial.print( txframe[1], 16 ); Serial.print( ' ' );
  Serial.print( txframe[2], 16 ); Serial.print( ' ' );
  Serial.print( txframe[3], 16 ); Serial.print( ' ' );
  Serial.print( txframe[4], 16 ); Serial.print( ' ' );
  Serial.print( txframe[5], 16 ); Serial.println( ' ' );*/
  transmit(txframe, txframe_pos);
  //transmit(txframe, txframe_pos);  // Optional: Redundency 
}

// from https://github.com/RobertRol/SimpleHC12/
unsigned long findBaudrateIdx() {
    memset(cmdResBuff, 0, sizeof(cmdResBuff));
    void(* resetFunc) (void) = 0;
    const char *cmdChar = "AT\r\n";
    bool foundBaud=false;
    size_t i=0;
    boolean bufferOK;
    start_at();
    while (i<baudArrayLen && !foundBaud) {         
        toggle_led();      
        Serial.end();
        delay(500);
        Serial.begin(baudArray[i]);
        delay(500);
        
        bufferOK=hc12_cmd(cmdChar);
        if (!bufferOK) break;
        foundBaud=(cmdResBuff[0]=='O'&&cmdResBuff[1]=='K');
        delay(500);
        if (!foundBaud) i++;
    }
    stop_at();
//#ifdef SURVEY_SCAN
    if(foundBaud){
      toggle_led();
      delay(400);
      toggle_led();
      delay(400);
      toggle_led();
      delay(400);
      toggle_led();
      delay(400);
      toggle_led();
      delay(400);    
    }
    if(!foundBaud){
      toggle_led();
      delay(600);
      toggle_led();
    }
//#endif
    return (i);
    
}

// Send sync packet in highest power possible on all speeds (reciver speed is unknown)
void set_hc_baudrate(int baudIndex)
{
  char buf[12];
  snprintf(buf, 12, "AT+B%d\r\n", baudArray[baudIndex]);
  start_at();
  delay(40);
  hc12_cmd(buf);
  stop_at();
  //Serial.print("resp ");
  //Serial.println(cmdResBuff);
  while(Serial.available()) Serial.read();
  Serial.end();
  Serial.begin(baudArray[baudIndex]);
}
void set_power(byte power)
{
  char cmd[10];
  snprintf(cmd, 7, "AT+P%d\r\n", power);
  start_at();
  delay(100);
  hc12_cmd(cmd);
  stop_at();
}

void setup_hc12(){
  //TODO: DEBUG
#ifdef JUST_TEST_RF_ON_BOOT
  Serial.begin(HC12_TARGET_BAUDRATE);
  Serial.println("HELLO WORLD!!!");
  Serial.flush();
#endif  
  // Congiure HC12 Set pin as output
  pinMode(HC12_SET_PIN, OUTPUT);

  if(!HC12_ASSUME_CURRENT_BAUD_EQ_TARGET){
    // Find currently configured baudrate and start serial on it
    unsigned long baud = findBaudrateIdx();
    Serial.begin( baudArray[baud]);  
  }else{
    Serial.begin( HC12_TARGET_BAUDRATE );  
  }
  start_at();

  set_power(HC12_TARGET_POWER);
  set_hc_baudrate(HC12_TARGET_BAUDRATE_IDX);

  digitalWrite(HC12_SET_PIN, HIGH); //stop_at

  hc12_is_atmode = false;

#ifdef READ_TEMPS_WHILE_SLEEPING
  sensors.setWaitForConversion(false);
#endif
}



void eeprom_save(){
#ifdef DEBUG_PRINTS
  Serial.println("EEPROM: Saving");
#endif
  /*EEPROM.put(EEPROM_SENSORS_NUM_ADDR, nv_sensors);
  EEPROM.put(EEPROM_SENSORS_LIST_ADDR,  nv_sensors_list);*/
}

unsigned long sensor_conversion_requested_millis;
void sensors_off(){
  digitalWrite(SENSOR_POWER_PIN, LOW);
}

void sensors_on(){
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  sensors.setResolution(RESOLUTION);
}

void sensors_triggerMesurment(){
  sensors_on();
  sensors.requestTemperatures();
  sensor_conversion_requested_millis = millis();
}

void sensors_wait_conversions(){
  while((millis() - sensor_conversion_requested_millis) < sensors.millisToWaitForConversion(RESOLUTION)){
  }
}

void configure_sensors(){
  sensors_on();
  // TRUE : function requestTemperature() etc will 'listen' to an IC to determine whether a conversion is complete
  //sensors.setCheckForConversion(true);
  sensors.setCheckForConversion(false); // Doesn't changes something in the device, issue only once

  // sets the value of the waitForConversion flag
  // TRUE : function requestTemperature() etc returns when conversion is ready
  // FALSE: function requestTemperature() etc returns immediately (USE WITH CARE!!)
  //        (1) programmer has to check if the needed delay has passed
  //        (2) but the application can do meaningful things in that time
  sensors.setWaitForConversion(false); // use with isConversionComplete ?
  //sensors.setWaitForConversion(true); // use with isConversionComplete ? // Doesn't changes something in the device, issue only once
}

void setup(void) {
  
  // Doesnt doo much?
  //for(int a=0;a<13;) pinMode(a++,INPUT_PULLUP);

  //power_all_disable(); //This reduces ~10uA // But doesnt wake up :/
  
  // Next 3 commands reduces to 0.13 uA
#ifndef LowPower_h // Still in expirimintation, if lowpower is included don't use these
  power_adc_disable();
  power_twi_disable();
  power_spi_disable();
#endif

  // These causes problems, dont run them, device doesn't wake up
  /* power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable(); */
  
  // power_usart0_disable(); // This actually takes more power, I guess since the input pins are floating
  // 1.45uA when I do this, 0.14 when not executed.

  setup_hc12();
#ifdef SURVEY_SCAN
  return;
#endif
#ifdef ACTIVATE_TEST_MODULE
  return;
#endif
  hc12_sleep();

#ifndef LowPower_h
  // TODO: Improve power consumption
  // Maybe power down?
  // Further sleep modes reading: https://forum.arduino.cc/t/power-consumption-of-pins-in-different-pin-modes/567117/15
  //
  //set_sleep_mode(SLEEP_MODE_PWR_SAVE); //1.8 mA?
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //10uA
#endif
  EEPROM.begin();
  clear_stats();

  // Load known sensors address from NVRAM
  FromEEPROM(snapshot_nvram);

  // Find active sensors on bus - sensors object is now populated
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  sensors_on(); //Should be before sensors.begin
  sensors.begin();
  configure_sensors();
  FromDallas(sensors, snapshot_detected);
  sensors_off();
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
    Serial.begin(HC12_TARGET_BAUDRATE);

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

unsigned long last_sample_millis = 0;
bool first_loop = true;

bool isLastSleepCycleBeforeTrnamission(unsigned long last_sample_millis,unsigned long  cur_millis){
  /* when 
      (cur_millis-last_sample_millis)<SAMPLE_INTERVAL )
     Evaluate to true, time has passed, try to step this expression forword in time */
  cur_millis += + AVR_SLEEP_TIME;
  return !((cur_millis-last_sample_millis)<SAMPLE_INTERVAL );
}

void send_sync()
{
  Serial.write("\x00\x00\x00\x00\x00\x00\x00", 7);
  Serial.flush();
}

void send_ping(unsigned int count, byte power){
  byte buf[] = {0xaa, 0x55, count & 0xff, power, 0xff};
  Serial.write(buf, sizeof(buf));
  Serial.flush();
}


#define UART_BITS_IN_BYTES(bytes) ((8+1+1)*(bytes))
// How meny MS does it take to TX bytes amount of bytes\
// Frame time is amount of time per bit *Nbits
#define MINIMUM_MS_PER_BIT(baudrate) ((1/(baudrate/2))*1000)
// if 600 bit per seconds,
// each bit is 1/600 s per bit, or 1000/600 ms per bit
#define MINIMUM_AIR_TIME(bytes, baudrate) ( UART_BITS_IN_BYTES(bytes)*MINIMUM_MS_PER_BIT(baudrate) )
#define TOTAL_ITERATION_TIME 800 // at 1200 bitrate, this is around 833 W/O further dalay
#define PING_COUNT 20
#define POWER_START 8
#define POWER_END 3
#define BAUD_START 0
#define BAUD_END 5

//const unsigned long baudArray[] = {1200,2400,4800,9600,19200,38400,57600,115200};

#define NUM_POWER_STEPS (POWER_START-POWER_END+1)
#define TIME_PER_BAUDRATE ((TOTAL_ITERATION_TIME*PING_COUNT*1.1*NUM_POWER_STEPS)+2000) // ~20*800*1.1 17600

//#define TIME_PER_BAUDRATE (PING_COUNT*MINIMUM_AIR_TIME(5,1200)) + FURTHER_ITERATION_DELAY*PING_COUNT
// 5 bytes 1200 speed should be:
// 20 * (10*5) * ( (1000/600) ) ~ 1666
// with delay: ~7666 ms
void survey_scan_loop(){
  unsigned int count = 0;
  byte power = 8;
  byte nextBaudIndex;
  //This seem to work
  unsigned long baud = findBaudrateIdx();
  Serial.begin(1200);
  //Serial.print("Found ");
  Serial.println(baudArray[baud]);
  Serial.flush();
  while(Serial.available()) Serial.read();
  Serial.end();
  Serial.begin( baudArray[baud]);  

  //for(int i; i<baudArrayLen; i++)
  //{
  //blink(200,5);
  set_power(power);
  set_hc_baudrate(0);
  nextBaudIndex = 1;
  send_sync();
  delay(400);

  unsigned long start_baud_millis = millis();
  while(nextBaudIndex<=BAUD_END){
    // TODO: fix for power>0
    for(power=POWER_START; power>=POWER_END; power--)
    {
      set_power(power);
      for(int i=0; i<PING_COUNT; i++){
        unsigned long iteration_start_millis = millis();
        toggle_led();
        send_ping(count, power);
        toggle_led();
        count++;
        while( ( millis()-iteration_start_millis ) < TOTAL_ITERATION_TIME ) ;
      }
    }
    
    if( 1 ){
      //Prepare for next iteration
      if (nextBaudIndex > (baudArrayLen-1) )
      {
        return;
      }
      //nextBaudIndex = 0;//TODO: TEST REMOVE THIS
      set_hc_baudrate(nextBaudIndex);
      nextBaudIndex++;
      set_power(power);
    }

    // Wait for time sync - iteration end
    while( (millis()-start_baud_millis) < TIME_PER_BAUDRATE ){
      //delay(100);
      //Serial.print('.');
    }
    start_baud_millis = millis();

  }//nextBaudIndex
}


void loop() {
  //if we sleep here, current cons is super low.
#ifdef DEBUG_PRINTS 
  Serial.print('.');
#endif
#ifdef SURVEY_SCAN
  survey_scan_loop();
  return;
#endif
#ifdef ACTIVATE_TEST_MODULE
  test_hc12();
  return;
#endif
  unsigned long cur_millis = millis() + (sleeping_millis);
  //if(false){
  //if(!avr_sleeping){
  if( (!first_loop) && (cur_millis-last_sample_millis)<SAMPLE_INTERVAL ) {
#ifdef READ_TEMPS_WHILE_SLEEPING
    if (isLastSleepCycleBeforeTrnamission(last_sample_millis, cur_millis)){
      sensors.requestTemperatures();
#ifdef DEBUG_PRINTS    
      Serial.println("next loop()-mesure_and_send");
#endif 
    }
#endif
    // Goto sleep
    //stop_at(); // Leave AT so SLEEP takes effect- NO! DO NOT LEAVE AT, Current consumption is few uA lower
    avr_sleep();
  }else{
#ifdef DEBUG_PRINTS    
    unsigned long loop_start = millis();
#endif
#ifdef READ_TEMPS_WHILE_SLEEPING
    if(first_loop){
      sensors.requestTemperatures();
      delay(1000);
    }
#endif
    sensors_triggerMesurment();
    first_loop = false;
    avr_sleeping = !avr_sleeping;
    last_sample_millis = millis() + sleeping_millis;
    //exit HC12 sleep
    mesure_and_send();
    // Next loop - device goes into sleep
#ifdef DEBUG_PRINTS    
    unsigned long loop_time = millis() - loop_start;
    Serial.print("Loop took ");
    Serial.println(loop_time);
#endif
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

