// Example sketch showing how to create a node thay repeates messages
// from nodes far from gateway back to gateway. 
// It is important that nodes that has enabled repeater mode calls  
// gw.preocess() frequently. This node should never sleep. 

#include <MySensor.h>
#include <SPI.h>

MySensor gw;

void setup()  
{  
  // The third argument enables repeater mode.
  gw.begin(NULL, 250, true);

  //Send the sensor node sketch version information to the gateway
  gw.sendSketchInfo("Repeater Node", "1.0");
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
  gw.sendBatteryLevel(readVcc()/100);  
  // By calling wait() what calles process() you route messages in the background
  gw.wait(300000); // Wait 5 min
}

