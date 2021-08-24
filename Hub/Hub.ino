// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem
// configuration

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 905.0

// who am i? (server address)
#define MY_ADDRESS     1


#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);


/************ Ethernet Setup ***************/
#include <Ethernet.h>
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEC };

IPAddress fallback_ip(192, 168, 1, 8);
IPAddress fallback_gateway(192, 168, 1, 1);


/************ MQTT Setup ***************/
#include <MQTT.h>
#define MQTT_HOST "192.168.1.178"
EthernetClient MQTTEthClient;
MQTTClient mqtt_client(512);
/************ Watchdg Setup ***************/
#include <Adafruit_SleepyDog.h>

/************ JSON Setup ***************/
#include <ArduinoJson.h>

/************ Display Setup ***************/
#include "Adafruit_ThinkInk.h"
#define EPD_DC      10
#define EPD_CS      9
#define SRAM_CS     6
#define EPD_RESET   -1 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY    -1 // can set to -1 to not use a pin (will wait a fixed delay)
ThinkInk_213_Mono_B72 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);


void setup()
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);

  // Set a watchdog in case we get stuck somewhere
  Watchdog.enable(10000);

  
  // Init and reset RF module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  pinMode(RFM69_CS, OUTPUT);

  // manual reset RF
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  // Disable RF spi until we need to actually use RF
  disable_radio_spi();


  // Initialize display
  display.begin(THINKINK_MONO);
  Serial.println("Banner demo");
  display.clearBuffer();
  display.setTextSize(1);
  display.setCursor((display.width() - 180)/2, (display.height() - 24)/2);
  display.setTextColor(EPD_BLACK);
  display.print("HVAC-IoT Ethernet Hub");
  display.display();

  // Init ethernet wing
  Ethernet.init(10);

  Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
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
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, fallback_ip, fallback_gateway);
  } else {
    Serial.print("  DHCP assigned IP ");
    Serial.println(Ethernet.localIP());
  }
  
  // MQTT init
  mqtt_client.begin(MQTT_HOST, MQTTEthClient);
  mqtt_client.onMessage(messageReceived);
  mqtt_connect();

  enable_radio_spi();
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  disable_radio_spi();

  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}


// Dont put this on the stack:
uint8_t data[] = "And hello back to you";
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop() {
  enable_radio_spi();
  bool received_msg = false;
  uint8_t from;
  int received_rssi = 0;
  StaticJsonDocument<512> doc;

  // Reset watchdog on each loop
  Watchdog.reset();
  uint8_t len = sizeof(buf);

  if (rf69_manager.available())
  {
    digitalWrite(LED, HIGH);

    Serial.print("Free mem:");
    Serial.println(freeMemory());
    // Wait for a message addressed to us from the client
    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      buf[len] = 0; // zero out remaining string
      received_msg = true;

      received_rssi = rf69.lastRssi();
      Serial.print("Got packet from #"); Serial.print(from, HEX);
      Serial.print(" [RSSI :");
      Serial.print(received_rssi);
      Serial.print("] : ");
      Serial.println((char*)buf);        
    }
  }
  disable_radio_spi();
  if (received_msg) {
    char mqtt_msg[1024];   
    bin_rf_msg_to_data(doc, from, received_rssi, (char*) &buf, len);   
    serializeJson(doc, mqtt_msg);
    // Reconnect if not connected
    if( !mqtt_client.connected() ){
       Serial.println("MQTT Not connected");
       mqtt_connect();
    }
    mqtt_client.publish("/metrics_json", mqtt_msg);

    Serial.println("MQTT Publish: ");
    Serial.println(mqtt_client.lastError());
    print_eth_status();
    const char* hex_id = doc["meta"]["id_hex"];
    const char* msg_type = doc["type"];
    if (hex_id && msg_type) {
      char topic[255];
      sprintf(topic, "/hvac-iot/type/%s/hex_id/%s", msg_type, hex_id);

      mqtt_publish_config(topic, hex_id);
      mqtt_client.publish(topic, mqtt_msg);
      Serial.println("MQTT Publish: ");
      Serial.println(topic);

      Serial.println(mqtt_client.lastError());
      print_eth_status();
    }
  }
  mqtt_client.loop();
  digitalWrite(LED, LOW);

}

void print_eth_status() {
  Serial.print("Hardware Status:");
  Serial.println(Ethernet.hardwareStatus());
  Serial.print("Link Status:");
  Serial.println(Ethernet.linkStatus());
  
}

void mqtt_publish_config(char* data_topic, const char* hex_id) {
      char topic[255];
      sprintf(topic, "homeassistant/sensor/%s_tempc/config", hex_id);
      char mqtt_msg[1024];    
      StaticJsonDocument<512> doc;
      char name_val[128];
      sprintf(name_val, "hvac-iot temperature %s", hex_id);
      doc["device_class"] = "temperature";
      doc["name"] = name_val;
      doc["state_topic"] = data_topic;
      doc["unit_of_measurement"] = "°C";
      doc["value_template"] = "{{value_json.data.temp_c}}";

      serializeJson(doc, mqtt_msg);
      
      mqtt_client.publish(topic, mqtt_msg);
      Serial.println("MQTT Publish: ");
      Serial.println(topic);
      Serial.println(mqtt_msg);

      Serial.println(mqtt_client.lastError()); 
      print_eth_status();
}

bool mqtt_publish(String topic, String msg) {
    if(mqtt_client.publish(topic, msg) ) {
       Serial.println("MQTT Published");
    } else {
       Serial.println("MQTT NOT Published");
       Serial.println(mqtt_client.lastError());
       print_eth_status();
    }    
}

void bin_rf_msg_to_data(JsonDocument &doc, uint8_t sid, int rssi, char *rf_msg, int LenRemaining) {
   char FirstByte = rf_msg[0];
   Serial.print("First Byte ");
   Serial.println((int) FirstByte);
   LenRemaining = LenRemaining - 1;
   int pos = 1;
   int type = 0;
   uint8_t sid_int8 = 0;
   doc["data"]["rssi"] = rssi;
   if (FirstByte == 128) {
    doc["type"] = "sensor_reading";
   } else {
    doc["type"] = "unknown";
   }
   
   while( LenRemaining > 0 ) {
    type = rf_msg[pos];
    Serial.print("Remaining: ");
    Serial.println(LenRemaining);
    Serial.print("Type ");
    Serial.println((int) type);
    if (type == 130) {
      Serial.print("Getting Sensor uint32_t:");
      int32_t id = read_int32(&rf_msg[pos+1]);
      sid_int8 = (uint8_t) id;
      doc["meta"]["id" ] = id;
      doc["meta"]["id_hex"] = String(id, HEX);
      doc["meta"]["sid"] = String(sid_int8, HEX);
      doc["meta"]["name"] = id_to_name(id);
      LenRemaining = LenRemaining - 5;  
      pos = pos + 5;    
    }
    else if (type == 131) {
      doc["data"]["temp_c"] = read_float(&rf_msg[pos+1]);
      LenRemaining = LenRemaining - 5;
      pos = pos + 5;      
    }
    else if (type == 132) {
      doc["data"]["rh"] = read_float(&rf_msg[pos+1]);
      LenRemaining = LenRemaining - 5;
      pos = pos + 5;      
    }
    else if (type == 133) {
      doc["data"]["vbat"] = read_float(&rf_msg[pos+1]);
      LenRemaining = LenRemaining - 5;
      pos = pos + 5;      
    }
    else if (type == 134) {
      doc["data"]["tvoc"] = read_int16(&rf_msg[pos+1]);
      LenRemaining = LenRemaining - 3;
      pos = pos + 3;      
    }
    else if (type == 135) {
      doc["data"]["co2"] = read_int16(&rf_msg[pos+1]);
      LenRemaining = LenRemaining - 3;
      pos = pos + 3;      
    }
    else if (type == 136) {
      doc["data"]["mBar"] = read_float(&rf_msg[pos+1]);
      LenRemaining = LenRemaining - 5;
      pos = pos + 5;      
    }  
    else if (type == 141) {
      doc["data"]["pm10"] = read_int16(&rf_msg[pos+1]);
      LenRemaining = LenRemaining - 3;
      pos = pos + 3;      
    }
    else if (type == 142) {
      doc["data"]["pm25"] = read_int16(&rf_msg[pos+1]);
      LenRemaining = LenRemaining - 3;
      pos = pos + 3;      
    }
    else if (type == 143) {
      doc["data"]["pm100"] = read_int16(&rf_msg[pos+1]);
      LenRemaining = LenRemaining - 3;
      pos = pos + 3;      
    }
    else {
      Serial.println("Unknown type, skipping rest of message");
      serializeJsonPretty(doc, Serial);
      return;
    }
    
   }
   serializeJsonPretty(doc, Serial);
   return;
  
}

String id_to_name(int32_t id) {
  switch (id) {
    case 263925577:
      return String("Office");
    case 263575446:
      return String("Basement");
    case 273272222:
      return String("HallwayBathroom");
    case 273272244:
      return String("MasterBathroom");
    case 263575467:
      return String("FrontBedroom");
    case 263925386:
      return String("LivingRoom");
    case 272957121:
      return String("MasterBedroom");
    default:
      return String("Unknown");
  }  
  
}

int32_t read_int32(char* pChar4)
{
    return (pChar4[3] << 24) | (pChar4[2] << 16) | (pChar4[1] << 8) | (pChar4[0]);
}

int16_t read_int16(char* pChar4)
{
    return (pChar4[1] << 8) | (pChar4[0]);
}

float read_float(char* pChar4)
{
    float f;
    memcpy(&f, pChar4, sizeof(float));
    return f;
}

void disable_radio_spi() {
  digitalWrite(RFM69_CS, HIGH);
}
void enable_radio_spi() {
  digitalWrite(RFM69_CS, LOW);
}

void mqtt_connect() {
  mqtt_client.setKeepAlive(15);
  Serial.print("\nmqtt connecting...");

  // Loop forever trying to connect. Watchdog timer should reset if this takes too long
  while (!mqtt_client.connect("rf_eth_bridge_json", "rf_eth_bridge", "rf_eth_bridge")) {
    Serial.println("MQTT connection attempted!");
    if (!mqtt_client.connected()) {
      delay(100);
    }
  }
  mqtt_client.subscribe("/hello");
}



void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
}
// https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
    #ifdef __arm__
    // should use uinstd.h to define sbrk but Due causes a conflict
    extern "C" char* sbrk(int incr);
    #else  // __ARM__
    extern char *__brkval;
    #endif  // __arm__

    int freeMemory() {
      char top;
    #ifdef __arm__
      return &top - reinterpret_cast<char*>(sbrk(0));
    #elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
      return &top - __brkval;
    #else  // __arm__
      return __brkval ? &top - __brkval : &top - __malloc_heap_start;
    #endif  // __arm__
    }
