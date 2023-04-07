//code sourced from keyestudio wiki page
//https://wiki.keyestudio.com/index.php/Ks0022_keyestudio_LM35_Linear_Temperature_Sensor
//https://wiki.keyestudio.com/Ks0023_keyestudio_18B20_Temperature_Sensor
//for testing purposes only

//one wire library: http://www.pjrc.com/teensy/arduino_libraries/OneWire.zip

#include <OneWire.h>
 int DS18S20_Pin = 2; //DS18S20 Signal pin on digital pin 2
 //Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2

void setup()
{
    Serial.begin(9600);//Set Baud Rate to 9600 bps
}
 void loop()
{   double val;
    double dat;
    val=analogRead(0);//Connect LM35 on Analog 0
    dat=(500 * val) /1024;
    Serial.print("LM35:"); //Display the temperature on Serial monitor
    Serial.println(dat);
    delay(100);
    float temperature = getTemp();
    Serial.print("DS18S20:");
    Serial.println(temperature);
    delay(100);
}

float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius
 
  byte data[12];
  byte addr[8];
 
  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }
 
  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }
 
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }
 
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
 
  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad
 
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  ds.reset_search();
   
  byte MSB = data[1];
  byte LSB = data[0];
 
  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;  
  return TemperatureSum;
}
