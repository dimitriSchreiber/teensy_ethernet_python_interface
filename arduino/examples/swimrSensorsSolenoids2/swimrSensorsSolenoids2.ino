//--------------------------------------------------------------------------
//Imports
//--------------------------------------------------------------------------
#include "QuadEncoder.h"
#include <ADC.h>
#include <ADC_util.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <ArduinoJson.h>


//--------------------------------------------------------------------------
//General Defines
//--------------------------------------------------------------------------

#define  PRINT_SERIAL 0
#define PRINT_SERIAL_RECV 1
#define UPDATE_TIME_US 1000


//--------------------------------------------------------------------------
//Ethernet UDP and JSON setup
//--------------------------------------------------------------------------

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 127);
unsigned int localPort = 8888;      // local port to listen on
// buffers for receiving and sending data
char packetBuffer[1024];  // buffer to hold incoming packet,
char ReplyBuffer[1024];        // a string to send back
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

StaticJsonDocument<2000> doc;
StaticJsonDocument<2000> doc2;

//--------------------------------------------------------------------------
//ADC and digital IO setup
//--------------------------------------------------------------------------
ADC *adc = new ADC(); // adc object
#define PINS 18
#define DIG_PINS 11
uint8_t adc_pins[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17};
uint8_t digital_pins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  
//--------------------------------------------------------------------------
//Other global variables
//--------------------------------------------------------------------------
elapsedMicros time_full_loop;
elapsedMicros time_convert_json;
elapsedMicros time_read_ADC;
elapsedMicros time_since_start;
uint32_t loop_sleep_time = 0;


void toggle_digital_pin(int value, int index){
    if(value == 0)
    {
      digitalWrite(digital_pins[index], LOW);
    }
    else
    {
      digitalWrite(digital_pins[index], HIGH);
    }
}
                   
void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting up");
  
  //--------------------------------------------------------------------------
  //ADC Setup and digital IO
  //--------------------------------------------------------------------------
  pinMode(LED_BUILTIN, OUTPUT);
  
  for (int i = 0; i < DIG_PINS; i++)
  {
    pinMode(digital_pins[i], OUTPUT);
  }
  
  for (int i = 0; i < PINS; i++)
  {
      pinMode(adc_pins[i], INPUT);
  }
  ///// ADC0 ////
  adc->adc0->setAveraging(8);                                    // set number of averages
  adc->adc0->setResolution(16);                                   // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);     // change the sampling speed

  ////// ADC1 /////
  adc->adc1->setAveraging(8);                                    // set number of averages
  adc->adc1->setResolution(16);                                   // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);     // change the sampling speed

  
  //--------------------------------------------------------------------------
  //Ethernet Setup
  //--------------------------------------------------------------------------
  // start the Ethernet
  // Open serial communications and wait for port to open:
  Serial.println("Opening Ethernet Port");
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start UDP
  Udp.begin(localPort);
  Serial.println("Opened Ethernet Port and started UDP");

  //--------------------------------------------------------------------------
  //JSON Setup
  //--------------------------------------------------------------------------
  JsonArray data = doc.createNestedArray("data");
  data.add(0.00);

  //--------------------------------------------------------------------------
  //Misc. Setup
  //--------------------------------------------------------------------------
  time_since_start = 0;
  time_full_loop = 0;

  Serial.println("Entering main loop");
  delay(500);
  
}

void loop(){
  time_full_loop = 0; // zero loop timer

  //--------------------------------------------------------------------------
  //Fill JSON, read analog inputs, output data from Remote to Digital IO
  //--------------------------------------------------------------------------

  doc["time"] = uint32_t(time_since_start);
  doc["analog0_voltage"] = adc->analogRead(adc_pins[0])* 3.3 / adc->adc0->getMaxValue();
  doc["analog1_voltage"] = adc->analogRead(adc_pins[1])* 3.3 / adc->adc0->getMaxValue();
  doc["analog2_voltage"] = adc->analogRead(adc_pins[2])* 3.3 / adc->adc0->getMaxValue();
  doc["analog3_voltage"] = adc->analogRead(adc_pins[3])* 3.3 / adc->adc0->getMaxValue();
  doc["analog4_voltage"] = adc->analogRead(adc_pins[4])* 3.3 / adc->adc0->getMaxValue();
  doc["analog5_voltage"] = adc->analogRead(adc_pins[5])* 3.3 / adc->adc0->getMaxValue();
  doc["analog6_voltage"] = adc->analogRead(adc_pins[6])* 3.3 / adc->adc0->getMaxValue();
  doc["analog7_voltage"] = adc->analogRead(adc_pins[7])* 3.3 / adc->adc0->getMaxValue();
  doc["analog8_voltage"] = adc->analogRead(adc_pins[8])* 3.3 / adc->adc0->getMaxValue();
  doc["analog9_voltage"] = adc->analogRead(adc_pins[9])* 3.3 / adc->adc0->getMaxValue();
  doc["analog10_voltage"] = adc->analogRead(adc_pins[10])* 3.3 / adc->adc0->getMaxValue();
  doc["analog11_voltage"] = adc->analogRead(adc_pins[11])* 3.3 / adc->adc0->getMaxValue();
  doc["analog12_voltage"] = adc->analogRead(adc_pins[12])* 3.3 / adc->adc0->getMaxValue();
  doc["loop_sleep_time"] = loop_sleep_time;

  if(PRINT_SERIAL){
    Serial.print("Analog Line 0: ");
    Serial.println((double)doc["analog0_voltage"], 2);  
  }

 
  //--------------------------------------------------------------------------
  //Convert JSON, send UDP packet
  //--------------------------------------------------------------------------
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {

    String output = "";
    serializeJson(doc, output);
    output.toCharArray(ReplyBuffer, 1024);
    
    if(PRINT_SERIAL_RECV){
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
    }
    IPAddress remote = Udp.remoteIP();

        // read the packet into packetBufffer
    Udp.read(packetBuffer, packetSize);
    deserializeJson(doc2, packetBuffer);

    if(PRINT_SERIAL_RECV){
      Serial.print("Contents:");
      Serial.println(packetBuffer);
    }

    if(PRINT_SERIAL_RECV){
      Serial.print("Unpacked JSON Solenoid 0:");
      Serial.println((int)(doc2["solenoid0"]));
      Serial.print("Unpacked JSON Solenoid 1:");
      Serial.println((int)(doc2["solenoid1"]));
      Serial.print("Unpacked JSON Solenoid 2:");
      Serial.println((int)(doc2["solenoid2"]));
    }
    if(PRINT_SERIAL_RECV){
      Serial.print("Received JSON Solenoid 0: ");
      Serial.println((int)doc2["solenoid0"]);
      Serial.print("Received JSON Solenoid 1: ");
      Serial.println((int)doc2["solenoid1"]);
      Serial.print("Received JSON Solenoid 2: ");
      Serial.println((int)doc2["solenoid2"]);
      Serial.print("Received JSON Solenoid 9: ");
      Serial.println((int)doc2["solenoid2"]);
      Serial.print("Received JSON Solenoid 10: ");
      Serial.println((int)doc2["solenoid2"]);
    } 
    
    toggle_digital_pin((int)(doc2["solenoid0"]),0);
    toggle_digital_pin((int)(doc2["solenoid1"]),1);
    toggle_digital_pin((int)(doc2["solenoid2"]),2);
    toggle_digital_pin((int)(doc2["solenoid3"]),3);
    toggle_digital_pin((int)(doc2["solenoid4"]),4);
    toggle_digital_pin((int)(doc2["solenoid5"]),5);
    toggle_digital_pin((int)(doc2["solenoid6"]),6);
    toggle_digital_pin((int)(doc2["solenoid7"]),7);
    toggle_digital_pin((int)(doc2["solenoid8"]),8);
    toggle_digital_pin((int)(doc2["solenoid9"]),9);
    toggle_digital_pin((int)(doc2["solenoid10"]),10); 

    //toggle_digital_pin(int(1),1);

    
    // send a reply to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }
  
  doc["loop_time"] = (uint32_t)(time_full_loop);

  if(PRINT_SERIAL){
    Serial.print("Time full loop code minus sleep: ");
    Serial.println(time_full_loop);
  }
 
  //--------------------------------------------------------------------------
  //Control loop rate to ~UPDATE_TIME_US
  //--------------------------------------------------------------------------
  if(uint32_t(time_full_loop) < UPDATE_TIME_US)
  {
    loop_sleep_time = UPDATE_TIME_US - (uint32_t)(time_full_loop);
  }
  else
  {
    loop_sleep_time = 1;
  }
  delayMicroseconds(loop_sleep_time);

  if(PRINT_SERIAL){
    Serial.print("Time full loop code with sleep: ");
    Serial.println(time_full_loop);
  }
 

}
