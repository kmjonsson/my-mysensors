// Example sketch showing how to send in OneWire temperature readings
#include <MySensor.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#define ONE_WIRE_BUS 4 // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 16

unsigned long SLEEP_TIME = 30*1000; // (300s) Sleep time between reads (in milliseconds)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
MySensor gw;
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors=0;
boolean receivedConfig = false;
boolean metric = true; 
// Initialize temperature message
MyMessage msg(0,V_TEMP);

#define SENDCOUNT 12
int send=SENDCOUNT-1;

void setup()  
{ 
  // Startup OneWire 
  sensors.begin();

  // Startup and initialize MySensors library. Set callback for incoming messages. 
  gw.begin(NULL); 

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Temperature Sensor", "1.0");

  // Fetch the number of attached temperature sensors  
  numSensors = sensors.getDeviceCount();
  
  Serial.print("numSensors = ");
  Serial.println(numSensors);
  
  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   
     gw.present(i, S_TEMP);
  }
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
  sensors.requestTemperatures(); 
  
  // Read temperatures and send them to controller 
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
 
    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;
 
    // Only send data if temperature has changed and no error
    if (temperature != -127.00) {
      // Send in the new temperature
      gw.send(msg.setSensor(i).set(temperature,1));
      lastTemperature[i]=temperature;     
    }
  }
  gw.sendBatteryLevel(readVcc()/100);  
  gw.sleep(5000);
}



