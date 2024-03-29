#include <Arduino.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

#define EEPROM_SENSORS_NUM_ADDR 0
#define EEPROM_SENSORS_LIST_ADDR 4
#define EEPROM_SENSORS_LIST_SIZE (8*MAX_SENSORS)

//#define MAX_SENSORS 4

typedef  byte SensorAddress[8];
typedef struct {
    SensorAddress address;
    bool available;
} SensorRuntime;

typedef struct{
    byte count = 0;
    SensorRuntime sensors[MAX_SENSORS];
}SensorSnapshot;

bool find_addr_in_arr(const SensorAddress arr[], byte count, const byte *addr);
SensorRuntime *GetByAddr( SensorSnapshot &snapshot, const DeviceAddress &addr);
bool FromDallas(DallasTemperature &sensors, SensorSnapshot &snapshot);
bool FromEEPROM(SensorSnapshot &snapshot);
byte MarkActive(SensorSnapshot &repo, SensorSnapshot &active);
int GetIndexByAddr( SensorAddress &addr, SensorSnapshot &repo);
SensorSnapshot *Merge( SensorSnapshot &target, SensorSnapshot &source);

bool find_addr_in_arr(const SensorAddress arr[], byte count, const byte *addr){
  for( ; count; ){
    count--;
    if(memcmp(arr[count], addr, 8) == 0)
      return true;
  }
  return false;
}

SensorRuntime *GetByAddr( SensorSnapshot &snapshot, const DeviceAddress &addr){
    for(byte i; i<snapshot.count; i++){
        if(memcmp( snapshot.sensors[i].address, &addr, sizeof(DeviceAddress)) == 0){
            return &snapshot.sensors[i];
        }
    }
    return NULL;
}

bool FromDallas(DallasTemperature &sensors, SensorSnapshot &snapshot){
    static SensorAddress tmp_addr;
    snapshot.count = sensors.getDS18Count();
    Serial.println(snapshot.count);
    for(byte i=0; i<snapshot.count; i++){
        snapshot.sensors[i].available = true;
        if(!sensors.getAddress(tmp_addr, i)){
            Serial.println("**Unexpected error?");
            return false;
        }
        memcpy(snapshot.sensors[i].address, tmp_addr, sizeof(SensorAddress));
    }
    return true;
}

bool FromEEPROM(SensorSnapshot &snapshot){
    static SensorAddress nv_sensors_list_tmp[MAX_SENSORS];
    EEPROM.get(EEPROM_SENSORS_NUM_ADDR, snapshot.count);
    if (snapshot.count<=0 || snapshot.count>MAX_SENSORS)
    {
        snapshot.count = 0;
        //memset(nv_sensors_list, 0, sizeof(nv_sensors_list));
#ifdef DEBUG_PRINTS
        Serial.println("EEPROM: Sensors reset - invalid count");
#endif    
        return false;
    }

    EEPROM.get( EEPROM_SENSORS_LIST_ADDR,  nv_sensors_list_tmp);
    for(byte i=0; i<snapshot.count; i++){
        snapshot.sensors[i].available = false; //Assume
        memcpy(snapshot.sensors[i].address, nv_sensors_list_tmp[i], sizeof (SensorAddress));
    }
    return true;
#ifdef DEBUG_PRINTS
  Serial.println("EEPROM: Sensors:");
  for(int sensor_idx=0; sensor_idx<snapshot.count; sensor_idx++){
    for( int j = 0; j < 8; j++) {
      Serial.write(' ');
      Serial.print(nv_sensors_list_tmp[sensor_idx][j], HEX);
    }
    Serial.println();
  }
#endif
}


byte MarkActive(SensorSnapshot &repo, SensorSnapshot &active){
    byte ret = 0;
    for(byte i=0; i<active.count; i++)
    {
        auto entry = GetByAddr(repo, active.sensors[i].address);
        if(entry){
            // Found active[i] in entry(in repo)
            entry->available = true;
            ret ++;
        }
    }
    return ret;
}

int GetIndexByAddr( SensorAddress &addr, SensorSnapshot &repo){
    for(int i=0; i<repo.count; i++)
    {
        if(memcmp(addr, repo.sensors[i].address, sizeof(addr))==0){
            return i;
        }
    }
    return -1;
}

SensorSnapshot *Merge( SensorSnapshot &target, SensorSnapshot &source){
    byte new_count = source.count;
    for(byte i=0; i<source.count; i++){
        if(GetIndexByAddr( source.sensors[i].address, target ) ==-1 ){
            target.sensors[new_count] = source.sensors[i];
            new_count++;
        }
    }
    return &target;
}