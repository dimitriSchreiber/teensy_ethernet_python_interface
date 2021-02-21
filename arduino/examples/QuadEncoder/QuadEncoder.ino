#include "QuadEncoder.h"

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#include <ArduinoJson.h>

#define  PRINT_SERIAL 1
#define UPDATE_TIME_US 1000
//--------------------------------------------------------------------------
//Ethernet UDP setup
//--------------------------------------------------------------------------

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 126);
unsigned int localPort = 8888;      // local port to listen on
// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer[1000];        // a string to send back
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
StaticJsonDocument<2000> doc;


//--------------------------------------------------------------------------
//Hardware encoder setup
//--------------------------------------------------------------------------
uint32_t mCurPosValue;
uint32_t old_position = 0;
uint32_t mCurPosValue1;
uint32_t old_position1 = 0;
QuadEncoder myEnc1(1, 3, 4, 0);  // Encoder on channel 1 of 4 available
                                 // Phase A (pin0), PhaseB(pin1), Pullups Req(0)
//QuadEncoder myEnc2(2, 2, 3, 0);  // Encoder on channel 2 of 4 available
                                 //Phase A (pin2), PhaseB(pin3), Pullups Req(0)

 uint32_t loop_time = 0;
 uint32_t loop_sleep_time = 0;   

elapsedMicros time_full_loop;
elapsedMicros time_convert_json;
elapsedMicros time_read_encoder;
elapsedMicros time_read_ADC;
                        
void setup()
{
  while(!Serial && millis() < 4000);

  analogReadResolution(13);
  //--------------------------------------------------------------------------
  //Hardware Encoder Setup
  //--------------------------------------------------------------------------

  /* Initialize the ENC module. */
  myEnc1.setInitConfig();  //
  myEnc1.EncConfig.revolutionCountCondition = ENABLE;
  myEnc1.EncConfig.enableModuloCountMode = ENABLE;
  myEnc1.EncConfig.positionModulusValue = 20000; 
  // with above settings count rev every 20 ticks
  // if myEnc1.EncConfig.revolutionCountCondition = ENABLE;
  // is not defined or set to DISABLE, the position is zeroed every
  // 20 counts, if enabled revolution counter is incremented when 
  // phaseA ahead of phaseB, and decrements from 65535 when reversed.
  myEnc1.init();

  /*
  myEnc2.setInitConfig();  //
  myEnc2.EncConfig.positionInitialValue = 160;
  myEnc2.EncConfig.positionMatchMode = ENABLE;
  myEnc2.EncConfig.positionCompareValue = 200;
  myEnc2.EncConfig.filterCount = 5;
  myEnc2.EncConfig.filterSamplePeriod = 255;
  myEnc2.init();
  */

  //--------------------------------------------------------------------------
  //Ethernet Setup
  //--------------------------------------------------------------------------
// start the Ethernet
  Ethernet.begin(mac, ip);

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}

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

  JsonArray data = doc.createNestedArray("data");
  data.add(48.756080);
  data.add(2.302038);


  /*
  for(int i=0;i<10;i++){
    sendNTPpacket(timeServer); // send an NTP packet to a time server
  
    // wait to see if a reply is available
    delay(1000);
    if (Udp.parsePacket()) {
      // We've received a packet, read the data from it
      Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
  
      // the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, extract the two words:
  
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      Serial.print("Seconds since Jan 1 1900 = ");
      Serial.println(secsSince1900);
  
      // now convert NTP time into everyday time:
      Serial.print("Unix time = ");
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      // subtract seventy years:
      unsigned long epoch = secsSince1900 - seventyYears;
      // print Unix time:
      Serial.println(epoch);
  
  
      // print the hour, minute and second:
      Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
      Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
      Serial.print(':');
      if (((epoch % 3600) / 60) < 10) {
        // In the first 10 minutes of each hour, we'll want a leading '0'
        Serial.print('0');
      }
      Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
      Serial.print(':');
      if ((epoch % 60) < 10) {
        // In the first 10 seconds of each minute, we'll want a leading '0'
        Serial.print('0');
      }
      Serial.println(epoch % 60); // print the second
    }
    // wait ten seconds before asking for the time again
    delay(10000);
    Ethernet.maintain();
  }
  */
}

void loop(){
  time_full_loop = 0;
  /* This read operation would capture all the position counter to responding hold registers. */

  uint32_t loop_start_time = micros();
  mCurPosValue = myEnc1.read();

  if(mCurPosValue != old_position && PRINT_SERIAL){
    /* Read the position values. */
    Serial.printf("Current position value1: %ld\r\n", mCurPosValue);
    Serial.printf("Position differential value1: %d\r\n", (int16_t)myEnc1.getHoldDifference());
    Serial.printf("Position HOLD revolution value1: %d\r\n", myEnc1.getHoldRevolution());
    Serial.println();
  }

  old_position = mCurPosValue;
  if(mCurPosValue1 != old_position1  && PRINT_SERIAL){
    /* Read the position values. */
    Serial.printf("Current position value2: %ld\r\n", mCurPosValue1);
    Serial.println();
  }

  old_position1 = mCurPosValue1;


   doc["time"] = micros();
   doc["encoder"] = int(mCurPosValue);
   doc["analog0"] = analogRead(0);
   doc["analog1"] = analogRead(1);
   doc["analog2"] = analogRead(2);
   doc["analog3"] = analogRead(3);
   doc["analog4"] = analogRead(4);
   doc["analog5"] = analogRead(5);
   doc["analog6"] = analogRead(6);
   doc["analog7"] = analogRead(7);
   doc["analog8"] = analogRead(8);
   doc["analog9"] = analogRead(9);
   doc["analog10"] = analogRead(12);
   doc["analog11"] = analogRead(13);
   doc["analog12"] = analogRead(14);

     Serial.print("time full loop");
    Serial.println(time_full_loop);

    
  doc["loop_sleep_time"] = loop_sleep_time;
  doc["loop_time"] = loop_time;
  

   

   
  String output = "";
  serializeJson(doc, output);
  output.toCharArray(ReplyBuffer, 1000);

  
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    //Serial.print("Received packet of size ");
    //Serial.println(packetSize);
    //Serial.print("From ");
    IPAddress remote = Udp.remoteIP();

        // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    
    //Serial.println("Contents:");
    //Serial.println(packetBuffer);

    // send a reply to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }

  loop_time = micros() - loop_start_time;
  if(loop_time < UPDATE_TIME_US)
  {
    loop_sleep_time = UPDATE_TIME_US - loop_time;
  }
  else
  {
    loop_sleep_time = 1;
  }
  delayMicroseconds(loop_sleep_time);
  //delayMicroseconds(10000);

}
