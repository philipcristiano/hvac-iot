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
#define RF69_FREQ 915.0

// Where to send packets to!
#define DEST_ADDRESS   1
// change addresses for each client board, any number :)
#define MY_ADDRESS     2


#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radioƒ
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13
#endif


/* WICED Feather w/wing
  #define RFM69_RST     PA4     // "A"
  #define RFM69_CS      PB4     // "B"
  #define RFM69_IRQ     PA15    // "C"
  #define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);


int16_t packetnum = 0;  // packet counter, we increment per xmission

/************ SHT40 Setup ***************/
#include "Adafruit_SHT4x.h"
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
uint32_t SHT40_ID;
uint8_t SHT40_ID_short;

/************ SGP30 Setup ***************/
#include "Adafruit_SGP30.h"
Adafruit_SGP30 sgp;
bool has_sgp30 = false;
/* return absolute humidity [mg/m^3] with approximation formula
* @param temperature [°C]
* @param humidity [%RH]
*/
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}


/************ Battery Setup ***************/
#define VBATPIN A4

#include "ArduinoLowPower.h"
/************ Display Setup ***************/

#include "Adafruit_ThinkInk.h"

#define EPD_CS      9
#define EPD_DC      10
#define SRAM_CS     6
#define EPD_RESET   -1 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY    -1 // can set to -1 to not use a pin (will wait a fixed delay)
uint32_t MILLIS_BETWEEN_DISPLAY_UPDATES = 180 * 1000;
uint32_t SLEEP_PER_LOOP = 60 * 1000;
uint8_t LOOPS_BEFORE_DISPLAY_UPDATE = MILLIS_BETWEEN_DISPLAY_UPDATES / SLEEP_PER_LOOP;
uint8_t LOOPS_SINCE_DISPLAY_UPDATE = LOOPS_BEFORE_DISPLAY_UPDATE + 1; // +1 to trigger an update on the first loop through


// 2.13" Monochrome displays with 250x122 pixels and SSD1675 chipset
//ThinkInk_213_Mono_B72 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

// 2.9" Grayscale Featherwing or Breakout:
ThinkInk_290_Grayscale4_T5 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

// Cache sensor vars
float CACHE_TEMP = 0;
float CACHE_HUMIDITY = 0;
float CACHE_VBAT = 0;
char MSG[64];

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("Booting");

  // Disable RF module until we need it
  pinMode(RFM69_CS, OUTPUT);
  disable_radio_spi();

  // Start display
  display.begin(THINKINK_MONO);
  Serial.println("Start display write");
  display.clearBuffer();
  display.setCursor((display.width() - 180) / 2, (display.height() - 24) / 2);
  display.setTextColor(EPD_BLACK);
  display.print("Booting");

  display.display();
  Serial.println("End display write");

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(RFM69_RST, OUTPUT);

  // RF manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  // Start sht4
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }


  SHT40_ID = sht4.readSerial();
  SHT40_ID_short = SHT40_ID;
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);

  rf69_manager.setThisAddress(SHT40_ID_short);

  // Start SGP30
  if (! sgp.begin()){
    Serial.println("SGP30 init failed");
    has_sgp30 = false;
  } else {
    has_sgp30 = true;
  }
  // Start RF init
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
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);
  disable_radio_spi();

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";


// RF Message Table
//  130 -> uint32_t - Sensor ID
//  131 -> uint32_t - temp

void write_radio_packet(uint8_t *pkt, uint32_t ID, sensors_event_t *temp){
    pkt[0] = (uint8_t) 128;

    pkt[1] = (uint8_t) 130;
    write_int32(&pkt[2], ID);

    pkt[6] = (uint8_t) 131;
    write_int32(&pkt[7], temp->temperature);
}


void write_int32(uint8_t *pkt, uint32_t val) {
    buf[0] = val;
    buf[1] = val >> 8;
    buf[2] = val >> 16;
    buf[3] = val >> 24;
}


void loop() {
  // Sleep mode was entered at the end of the loop, make sure things are started up again
  Serial.begin(115200);
  bool display_needs_update = false;
  // 3.2 reads as 4.2 4.2 reads as 4.99
  float measured_vbat = analogRead(VBATPIN);
  measured_vbat *= 2;    // we divided by 2, so multiply back
  measured_vbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measured_vbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measured_vbat);

  // Read sht40 sensor
  sensors_event_t humidity, temp;
  uint16_t tvoc =0;
  uint16_t ecO2 = 0;

  uint32_t timestamp = millis();
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  timestamp = millis() - timestamp;
  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

  Serial.print("Read duration (ms): ");
  Serial.println(timestamp);
  // Start radio sending

  char radiopacket[64];
  char msg[64] = "All OK!";
  write_radio_packet((uint8_t*)radiopacket, SHT40_ID, &temp);
  //write_radio_packet((uint8_t *)&radiopacket, SHT40_ID, &temp, &humidity, measured_vbat);
  // snprintf(radiopacket, sizeof(radiopacket), "sensor_reading tc=%f,batv=%f,rh=%f", temp.temperature, measured_vbat, humidity.relative_humidity);
  Serial.print("Sending "); Serial.println(radiopacket);
  Serial.print("Size "); Serial.println(strlen(radiopacket));
  enable_radio_spi();
  // Send a message to the DESTINATION!
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    char msg[64];
    if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      buf[len] = 0; // zero out remaining string

      Serial.print("Got reply from #"); Serial.print(from);
      Serial.print(" [RSSI :");
      Serial.print(rf69.lastRssi());
      Serial.print("] : ");
      Serial.println((char*)buf);
      strcpy(msg, "All OK");
    } else {
      Serial.println("No reply, is anyone listening?");
      strcpy(msg, "RF-No reply");
    }
  } else {
    Serial.println("Sending failed (no ack)");
    strcpy(msg, "RF-No Ack");
  }
  rf69.sleep();
  disable_radio_spi();

  // If changes are significant then update the display
  if (info_sig_change(temp, humidity, measured_vbat, msg)) {
    display_needs_update = true;
    Serial.println("Changes ARE significant, display should be updated");
  } else {
    Serial.println("Changes are NOT significant, don't bother updating display");
  }

  // If we need to update, and haven't updated recently / at first, then update!
  if ( LOOPS_SINCE_DISPLAY_UPDATE >= LOOPS_BEFORE_DISPLAY_UPDATE) {
    write_info(temp, humidity, measured_vbat, SHT40_ID, msg);
  } else {
   Serial.println("Display not updating");
  }


  Serial.end();
  digitalWrite(LED, LOW);
  //delay(SLEEP_PER_LOOP);
  LowPower.deepSleep(SLEEP_PER_LOOP);
  digitalWrite(LED, HIGH);
  LOOPS_SINCE_DISPLAY_UPDATE += 1;
  Serial.begin(115200);

}


bool info_sig_change(sensors_event_t temp, sensors_event_t humidity, float v_bat, char *msg) {
  if (abs(temp.temperature - CACHE_TEMP) > .25) {
    return true;
  }
  if (abs(humidity.relative_humidity - CACHE_HUMIDITY) > 1) {
    return true;
  }
  if (abs(v_bat - CACHE_VBAT) > 0.05) {
    return true;
  }
  if (strcmp(msg, MSG) != 0) {
    return true;
  }

  return false;
}

void write_info(sensors_event_t temp, sensors_event_t humidity, float v_bat, uint32_t id, char *msg) {
  display.begin(THINKINK_MONO);
  display.setTextColor(EPD_BLACK);
  display.setRotation(3);

  Serial.println("Start display write");
  display.clearBuffer();
  int temp_height = 10;
  int left_justify = 10;
  int round_temp = (int) (temp.temperature + 0.5);
  int round_humidity = (int) (humidity.relative_humidity + 0.5);
  display.setCursor(left_justify, temp_height);
  display.setTextSize(9);
  display.print(round_temp);

  display.setTextSize(2);
  display.setCursor(left_justify, temp_height + 80);
  display.setTextColor(EPD_BLACK);
  display.print(temp.temperature);
  display.print(" C");
  CACHE_TEMP = temp.temperature;

  display.setTextSize(2);
  display.setCursor(left_justify, temp_height + 100);
  display.print(round_humidity);
  display.print("% rH");
  CACHE_HUMIDITY = humidity.relative_humidity;

  display.setCursor(left_justify, temp_height + 120);
  display.print(v_bat);
  display.print("v");
  CACHE_VBAT = v_bat;

  display.setCursor(left_justify, temp_height + 140);
  display.print(msg);
  strncpy(MSG, msg, strlen(msg));

  display.setCursor(left_justify, temp_height + 230);
  display.setTextSize(1);
  display.print("ID: ");
  display.print(id, HEX);
  display.print(" ");
  display.print(SHT40_ID_short);
  display.print(" ");
  display.print(SHT40_ID_short, HEX);

  LOOPS_SINCE_DISPLAY_UPDATE = 0;
  display.display();
  Serial.println("End display write");
}

void disable_radio_spi() {
  digitalWrite(RFM69_CS, HIGH);
}
void enable_radio_spi() {
  digitalWrite(RFM69_CS, LOW);
}
