//--------------------------------------------------------------------------
//Imports
//--------------------------------------------------------------------------
#include <ADC.h>
#include <ADC_util.h>

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#include <ArduinoJson.h>


//--------------------------------------------------------------------------
//Defines
//--------------------------------------------------------------------------

#define  PRINT_SERIAL 0
#define UPDATE_TIME_US 1000

//--------------------------------------------------------------------------
//Ethernet UDP and JSON setup
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
//Other global variables
//--------------------------------------------------------------------------
elapsedMicros time_full_loop;
elapsedMicros time_convert_json;
elapsedMicros time_read_ADC;
elapsedMicros time_since_start;
uint32_t loop_sleep_time = 0;
uint32_t loop_time = 0;
                        
void setup()
{
  while(!Serial && millis() < 1000); // give system startup time

  //--------------------------------------------------------------------------
  //ADC Setup
  //--------------------------------------------------------------------------

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

  //--------------------------------------------------------------------------
  //JSON Setup
  //--------------------------------------------------------------------------
  JsonArray data = doc.createNestedArray("data");
  data.add(0.00);

  //--------------------------------------------------------------------------
  //Misc. Setup
  //--------------------------------------------------------------------------
  time_since_start = 0;
}

void loop(){
  time_full_loop = 0;
  
  doc["time"] = time_since_start;
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
  doc["loop_sleep_time"] = loop_sleep_time;
  doc["loop_time"] = time_full_loop;

  //--------------------------------------------------------------------------
  //Convert JSON, send UDP packet
  //--------------------------------------------------------------------------

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


  //--------------------------------------------------------------------------
  //Control loop rate to ~UPDATE_TIME_US
  //--------------------------------------------------------------------------
  Serial.print("time for a full loop");
  Serial.println(time_full_loop);

  if(time_full_loop < UPDATE_TIME_US)
  {
    loop_sleep_time = UPDATE_TIME_US - time_full_loop;
  }
  else
  {
    loop_sleep_time = 1;
  }
  delayMicroseconds(loop_sleep_time);


  
}
