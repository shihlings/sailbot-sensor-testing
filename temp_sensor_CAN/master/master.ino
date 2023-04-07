// demo: CAN-BUS Shield, send data
// loovee@seeed.cc


#include <SPI.h>
#include <OneWire.h>
 int DS18S20_Pin = 2; //DS18S20 Signal pin on digital pin 2
 //Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2
#define CAN_2515
// #define CAN_2518FD

// Set SPI CS Pin according to your hardware

#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#endif


#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin
#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif

void setup() {
    SERIAL_PORT_MONITOR.begin(115200);
    while(!Serial){};

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
}

unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
void loop() {
  double val;
  double dat;
  val=analogRead(0);//Connect LM35 on Analog 0
  dat=(500 * val) /1024;
  Serial.print("LM35:"); //Display the temperature on Serial monitor
  Serial.println(dat);
  float temperature = getTemp();
  Serial.print("DS18S20:");
  Serial.println(temperature);

  //change float into hex chars
  stmp[4] = (int) dat/10*16 + (int) dat % 10;
  int dec = (int)(dat*100.00) - ((int)dat * 100);
  stmp[5] = (int) dec/10*16 + (int) dec % 10;

  stmp[6] = (int) temperature/10*16 + (int) temperature % 10;
  int deci = (int)(temperature*100.00) - ((int)temperature * 100);
  stmp[7] = (int) deci/10*16 + (int) deci % 10;


  CAN.sendMsgBuf(0x0001, 0, 8, stmp);
  delay(100);                       // send data per 100ms
  SERIAL_PORT_MONITOR.println("CAN BUS sendMsgBuf ok!");
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