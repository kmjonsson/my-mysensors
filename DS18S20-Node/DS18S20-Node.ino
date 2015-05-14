// Example sketch showing how to send in OneWire temperature readings
#include <MySensor.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <string.h>

#define ONE_WIRE_BUS 4 // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 4

char lookup[16]={'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};

unsigned long SLEEP_TIME = 30*1000; // Sleep time between reads (in milliseconds)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
MySensor gw;

typedef struct _sensor {
  byte id;
  char sensor[3*8+1];
  float lastTemperature;
  bool sentPresent;
} dsSensor;

dsSensor ds_sensors[MAX_ATTACHED_DS18B20];

int numSensors=0;
boolean receivedConfig = false;
boolean metric = true; 
// Initialize temperature message
MyMessage msg(0,V_TEMP);
// Info V_ID message
MyMessage msginfo(0, V_VAR4);

#define SENDCOUNT 2
int send=SENDCOUNT-1;

byte asked    = 0;
byte askCount = 0;

int reqCount=0;
void incomingMessage(const MyMessage &message) {
  Serial.println("Incomming :-)");
  char str[3*8+1];
  message.getString(str);
  Serial.println(str);
  for(int i=0;i<numSensors;i++) {
    if(strcmp(str,ds_sensors[i].sensor) == 0) {
      ds_sensors[i].id = message.sensor;
      Serial.print("found sensor id=");
      Serial.println(message.sensor);
    }
  }
  if(askCount < 5) {
    askCount++;
  }
  Serial.print("reqCount=");
  Serial.println(reqCount++);
}


void setup()  
{ 
  // Startup OneWire 
  sensors.begin();

  // Startup and initialize MySensors library. Set callback for incoming messages. 
  gw.begin(incomingMessage,7); 

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Temperature Sensor", "1.0");

  // Fetch the number of attached temperature sensors  
  numSensors = sensors.getDeviceCount();
  
  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {  
    char msgbuf[8*3+1];
    uint8_t deviceAddress[8];
    //gw.present(i, S_TEMP);
    if (sensors.getAddress(deviceAddress, i)) {
      uint8_t j = 0;
      for (uint8_t x = 0; x < 8; x++) {
        msgbuf[j++] = lookup[deviceAddress[x]/16];
        msgbuf[j++] = lookup[deviceAddress[x]%16];
        msgbuf[j++] = ':';
      }
      msgbuf[j-1] = '\0';
      strcpy(ds_sensors[i].sensor,msgbuf);
      ds_sensors[i].id = 255;
      ds_sensors[i].lastTemperature = -999.9;
      ds_sensors[i].sentPresent = 0;
      Serial.print("Sensor: ");
      Serial.print(i);
      Serial.print(" : ");
      Serial.println(msgbuf);      
    }
  }

  // Ask controller får ID of active sensors
  // Start with 0
  gw.request(0,V_VAR4);
  
  Serial.println("End of Setup");
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void loop()     
{     
  // Process incoming messages (like config from server)
  gw.process(); 

  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   
    if(!ds_sensors[i].sentPresent && ds_sensors[i].id != 255) {
      Serial.print("Sending SensorId: ");
      Serial.println(ds_sensors[i].id);
      gw.present(ds_sensors[i].id, S_TEMP);
      ds_sensors[i].sentPresent = 1;
      delay(1000);
    }
  }
  
  if(asked < askCount) {
    gw.request(++asked,V_VAR4);
  }
  
  if(reqCount <= 5) {
    delay(1000);
    return;
  }

  // Fetch temperatures from Dallas sensors
  sensors.requestTemperatures(); 
  
//  gw.send(msg.setSensor(0).set(13.4,1));

  send = (send+1) % SENDCOUNT;

  // Read temperatures and send them to controller 
  for (int n=0; n<numSensors && n<MAX_ATTACHED_DS18B20; n++) {    
    if(ds_sensors[n].id == 255) {
      continue;
    }
      
    int i = ds_sensors[n].id;
 
    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;
 
    // Only send data if temperature has changed and no error
    if ((ds_sensors[n].lastTemperature != temperature || send == 0) && temperature != -127.00) {
      // Send in the new temperature
      gw.send(msg.setSensor(i).set(temperature,1));
      ds_sensors[n].lastTemperature=temperature;     
    }
  }
  
  send = 0;
  if(send == 0) { 
    gw.sendBatteryLevel(readVcc()/100);
  }
  
  gw.sleep(SLEEP_TIME);
}



