// Example sketch showing how to send in OneWire temperature readings
#include <MySensor.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#include <DHT.h>  


#define BUSSES 4
#define ONE_WIRE_BUS 5 // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 5

unsigned long SLEEP_TIME = 300*1000; // (30s) Sleep time between reads (in milliseconds)
OneWire *oneWire[BUSSES];
DallasTemperature *sensors[BUSSES];
MySensor gw;
float lastTemperature[BUSSES];
int numSensors[BUSSES];
boolean receivedConfig = false;
boolean metric = true; 
// Initialize temperature message
MyMessage msg(0,V_TEMP);

#define CHILD_ID_HUM (BUSSES+0)
#define CHILD_ID_TEMP (BUSSES+1)
#define HUMIDITY_SENSOR_DIGITAL_PIN 3

#define SOIL_HUM1_ID (CHILD_ID_TEMP+1)
#define SOIL_HUM1_AIN A0
#define SOIL_HUM1_POWER A1
#define SOIL_HUM2_ID (CHILD_ID_TEMP+2)
#define SOIL_HUM2_AIN A2
#define SOIL_HUM2_POWER A3

DHT dht;

float lastTemp;
float lastHum;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

MyMessage msgSoilHum1(SOIL_HUM1_ID, V_HUM);
MyMessage msgSoilHum2(SOIL_HUM2_ID, V_HUM);

void setup()  
{ 
  for(int i=0;i<BUSSES;i++) {    
    // Create OneWire Bus object
    oneWire[i] = new OneWire(ONE_WIRE_BUS + i);
    // Create Dallas object
    sensors[i] = new DallasTemperature(oneWire[i]);
    // Startup OneWire 
    sensors[i]->begin();
    // Fetch the number of attached temperature sensors  

    numSensors[i] = sensors[i]->getDeviceCount();
    Serial.print("numSensors[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(numSensors[i]);
  }
  
  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN);

  // Startup and initialize MySensors library. Set callback for incoming messages. 
  gw.begin(NULL); 

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Vaxthus", "1.0");
    
  // Present all sensors to controller
  for (int i=0; i<BUSSES; i++) {   
    if(numSensors[i] > 0) {
      gw.present(i, S_TEMP);
    }
  }
  
  gw.present(CHILD_ID_HUM, S_HUM);
  gw.present(CHILD_ID_TEMP, S_TEMP);

//  gw.present(SOIL_HUM1_ID, S_HUM);
//  gw.present(SOIL_HUM2_ID, S_HUM);
  
  metric = gw.getConfig().isMetric;
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

  // Fetch temperatures from Dallas sensors
  for(int i=0; i<BUSSES; i++) {   
    sensors[i]->requestTemperatures(); 
  }
  
  // Read temperatures and send them to controller 
  for (int i=0; i<BUSSES; i++) {
    
    if(numSensors[i] <= 0) {
      continue;
    }
 
    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors[i]->getTempCByIndex(0):sensors[i]->getTempFByIndex(0)) * 10.)) / 10.;
 
    // Only send data if temperature has changed and no error
    if (temperature != -127.00) {
      // Send in the new temperature
      gw.send(msg.setSensor(i).set(temperature,1));
      lastTemperature[i]=temperature;     
    }
  }
  
  delay(dht.getMinimumSamplingPeriod());

  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
      Serial.println("Failed reading temperature from DHT");
  } else if (temperature != lastTemp) {
    lastTemp = temperature;
    if (!metric) {
      temperature = dht.toFahrenheit(temperature);
    }
    gw.send(msgTemp.set(temperature, 1));
    Serial.print("T: ");
    Serial.println(temperature);
  }
  
  float humidity = dht.getHumidity();
  if (isnan(humidity)) {
      Serial.println("Failed reading humidity from DHT");
  } else if (humidity != lastHum) {
      lastHum = humidity;
      gw.send(msgHum.set(humidity, 1));
      Serial.print("H: ");
      Serial.println(humidity);
  }

/*
  uint32_t hum=0;
  digitalWrite(SOIL_HUM1_POWER,HIGH);
  delay(250);
  for(int i=0;i<10250;i++) {
    hum += analogRead(SOIL_HUM1_AIN);
  }
  hum /= 104960;
  hum = analogRead(SOIL_HUM2_AIN);
  Serial.print("H1: ");
  Serial.println(hum);

  gw.send(msgSoilHum1.set(hum, 1));
  digitalWrite(SOIL_HUM1_POWER,LOW);

  hum=0;
  digitalWrite(SOIL_HUM2_POWER,HIGH);
  delay(250);
  for(int i=0;i<10250;i++) {
    hum += analogRead(SOIL_HUM2_AIN);
  }
  hum /= 104960;
  hum = analogRead(SOIL_HUM2_AIN);
  Serial.print("H2: ");
  Serial.println(hum);
  gw.send(msgSoilHum2.set(hum, 1));
  digitalWrite(SOIL_HUM2_POWER,LOW);
*/
  
  gw.sendBatteryLevel(readVcc()/100);  
  gw.sleep(SLEEP_TIME);
}



