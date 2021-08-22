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
bool HAS_SHT40 = false;

/************ SGP30 Setup ***************/
#include "Adafruit_SGP30.h"
Adafruit_SGP30 sgp;
bool HAS_SGP30 = false;
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

/************ SCD30 Setup ***************/
#include <Adafruit_SCD30.h>

Adafruit_SCD30  scd30;
bool HAS_SCD30;

/************ BMP280 Setup ***************/
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C
bool HAS_BMP280;

/************ PM25AQI Setup ***************/
#include "Adafruit_PM25AQI.h"
Adafruit_PM25AQI pm25aqi = Adafruit_PM25AQI();
bool HAS_PM25AQI;

/************ Battery Setup ***************/
#define SENSOR_VBATPIN A4
#define FEATHER_VBATPIN A7

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
  delay(500);
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
  if (sht4.begin()) {
    HAS_SHT40 = true;
    SHT40_ID = sht4.readSerial();
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
  } else {
    HAS_SHT40 = false;
    SHT40_ID = 2;
  }
  SHT40_ID_short = SHT40_ID;
  rf69_manager.setThisAddress(SHT40_ID_short);

  // Start SGP30
  if (sgp.begin()){
    Serial.println("SGP30 init");
    HAS_SGP30 = true;
  } else {
    HAS_SGP30 = false;
    Serial.println("SGP30 init");
  }
  if (scd30.begin()) {
    Serial.println("Found SCD30 sensor");
    scd30.selfCalibrationEnabled(true);
    scd30.setMeasurementInterval(20);
    HAS_SCD30 = true;
  } else {
    Serial.println("Did not find SCD30 sensor");
    HAS_SCD30 = false;
  }

  if (pm25aqi.begin_I2C()) {
     HAS_PM25AQI = true;
     Serial.println("Found PM25AQI sensor");

  } else {
     HAS_PM25AQI = false;
     Serial.println("Did not find PM25AQI sensor");
  }

  if (bmp.begin()) {
    Serial.println("Found BMP280 sensor");
    HAS_BMP280 = true;
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  } else {
    HAS_BMP280 = false;
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
//  132 -> float - relative humidity
//  133 -> float - battery voltage
//  134 -> uint16_t - tvoc
//  135 -> uint16_t - co2 concentration
//  136 -> float - pressure mbar

int start_radio_packet(uint8_t *pkt, uint32_t ID) {
    pkt[0] = (uint8_t) 128;

    int pos = 1;
    pkt[pos] = (uint8_t) 130;
    write_int32(&pkt[pos+1], ID);
    pos = pos + 1 + 4;
    return pos;

}

int write_radio_uint16_t_msg(uint8_t *pkt, int pos, uint8_t type, uint16_t msg) {
  pkt[pos] = type;
  write_int16(&pkt[pos+1], msg);
  return pos + 1 + 2;
}

int write_radio_float_msg(uint8_t *pkt, int pos, uint8_t type, float msg) {
  pkt[pos] = type;
  write_float(&pkt[pos+1], msg);
  return pos + 1 + 4;
}
void write_int32(uint8_t *pkt, uint32_t val) {
    pkt[0] = val;
    pkt[1] = val >> 8;
    pkt[2] = val >> 16;
    pkt[3] = val >> 24;
}

void write_int16(uint8_t *pkt, uint16_t val) {
    pkt[0] = val;
    pkt[1] = val >> 8;
}

void write_float(uint8_t *pkt, float val) {
    memcpy(pkt, &val, sizeof(float));
}


void loop() {
  // Sleep mode was entered at the end of the loop, make sure things are started up again
  Serial.begin(115200);
  bool display_needs_update = false;
  float measured_vbat;
  // 3.2 reads as 4.2 4.2 reads as 4.99
  if (HAS_BMP280 || HAS_SGP30 || HAS_SCD30) {
    measured_vbat = analogRead(FEATHER_VBATPIN);
  } else {
    measured_vbat = analogRead(SENSOR_VBATPIN);
  }
  measured_vbat *= 2;    // we divided by 2, so multiply back
  measured_vbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measured_vbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measured_vbat);

  // Read sht40 sensor
  sensors_event_t humidity, temp;
  uint16_t tvoc =0;
  uint16_t eco2 = 0;

  uint32_t timestamp = millis();
  char radiopacket[64];
  int radiopacketlen = 0;
  char msg[64] = "All OK!";

  radiopacketlen = start_radio_packet((uint8_t*)radiopacket, SHT40_ID);
  radiopacketlen = write_radio_float_msg((uint8_t*)radiopacket, radiopacketlen, (uint8_t) 133, measured_vbat);

  if (HAS_SHT40) {
    sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    timestamp = millis() - timestamp;
    Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
    Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

    Serial.print("Read duration (ms): ");
    Serial.println(timestamp);
    radiopacketlen = write_radio_float_msg((uint8_t*)radiopacket, radiopacketlen, (uint8_t) 131, temp.temperature);
    radiopacketlen = write_radio_float_msg((uint8_t*)radiopacket, radiopacketlen, (uint8_t) 132, humidity.relative_humidity);
  }
  if (HAS_SGP30 && sgp.IAQmeasure()) {
     tvoc = sgp.TVOC;
     eco2 = sgp.eCO2;
     Serial.print("TVOC "); Serial.print(tvoc); Serial.print(" ppb\t");
     Serial.print("eCO2 "); Serial.print(eco2); Serial.println(" ppm");
     radiopacketlen = write_radio_uint16_t_msg((uint8_t*)radiopacket, radiopacketlen, (uint8_t) 134, tvoc);
     radiopacketlen = write_radio_uint16_t_msg((uint8_t*)radiopacket, radiopacketlen, (uint8_t) 135, eco2);
  }

  float pressuremBar;
  if (HAS_BMP280) {
    pressuremBar = bmp.readPressure() / 100;
    Serial.print(F("Pressure = "));
    Serial.print(pressuremBar);
    Serial.println(" mBar");
    radiopacketlen = write_radio_float_msg((uint8_t*)radiopacket, radiopacketlen, (uint8_t) 136, pressuremBar);
  }
  if (HAS_SCD30) {
    if (HAS_BMP280) {
      Serial.println("Sending pressure to SCD30");
      scd30.startContinuousMeasurement(pressuremBar);
    }
    if (scd30.dataReady()){
        Serial.println("SCD Data available!");

        if (scd30.read()){

        Serial.print("SCD30 Temperature: ");
        Serial.print(scd30.temperature);
        Serial.println(" degrees C");

        Serial.print("Relative Humidity: ");
        Serial.print(scd30.relative_humidity);
        Serial.println(" %");

        Serial.print("CO2: ");
        Serial.print(scd30.CO2, 3);
        Serial.println(" ppm");
        Serial.println("");
        Serial.println((uint16_t)scd30.CO2);

        radiopacketlen = write_radio_uint16_t_msg((uint8_t*)radiopacket, radiopacketlen, (uint8_t) 135, scd30.CO2);


      } else {
        Serial.println("No SCD30 data");
      }
    }
  }

  if (HAS_PM25AQI) {
      PM25_AQI_Data pm_data;
      if (pm25aqi.read(&pm_data)) {
          Serial.println("AQI reading success");

          Serial.println();
          Serial.println(F("---------------------------------------"));
          Serial.println(F("Concentration Units (standard)"));
          Serial.println(F("---------------------------------------"));
          Serial.print(F("PM 1.0: ")); Serial.print(pm_data.pm10_standard);
          Serial.print(F("\t\tPM 2.5: ")); Serial.print(pm_data.pm25_standard);
          Serial.print(F("\t\tPM 10: ")); Serial.println(pm_data.pm100_standard);
          Serial.println(F("Concentration Units (environmental)"));
          Serial.println(F("---------------------------------------"));
          Serial.print(F("PM 1.0: ")); Serial.print(pm_data.pm10_env);
          Serial.print(F("\t\tPM 2.5: ")); Serial.print(pm_data.pm25_env);
          Serial.print(F("\t\tPM 10: ")); Serial.println(pm_data.pm100_env);
          Serial.println(F("---------------------------------------"));
          Serial.print(F("Particles > 0.3um / 0.1L air:")); Serial.println(pm_data.particles_03um);
          Serial.print(F("Particles > 0.5um / 0.1L air:")); Serial.println(pm_data.particles_05um);
          Serial.print(F("Particles > 1.0um / 0.1L air:")); Serial.println(pm_data.particles_10um);
          Serial.print(F("Particles > 2.5um / 0.1L air:")); Serial.println(pm_data.particles_25um);
          Serial.print(F("Particles > 5.0um / 0.1L air:")); Serial.println(pm_data.particles_50um);
          Serial.print(F("Particles > 10 um / 0.1L air:")); Serial.println(pm_data.particles_100um);
          Serial.println(F("---------------------------------------"));

      } else {
        Serial.println("Could not read PM25AQI");
      }
  }
  Serial.print("Sending "); Serial.println(radiopacket);
  Serial.print("Size "); Serial.println(radiopacketlen);
  enable_radio_spi();
  // Send a message to the DESTINATION!
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, radiopacketlen, DEST_ADDRESS)) {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    char msg[64];

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

  digitalWrite(LED, LOW);
  //delay(SLEEP_PER_LOOP);
  LowPower.deepSleep(SLEEP_PER_LOOP);
  digitalWrite(LED, HIGH);
  LOOPS_SINCE_DISPLAY_UPDATE += 1;


}


bool info_sig_change(sensors_event_t temp, sensors_event_t humidity, float v_bat, char *msg) {
  if (abs(temp.temperature - CACHE_TEMP) > .25) {
    return true;
  }
  if (abs(humidity.relative_humidity - CACHE_HUMIDITY) > 5) {
    return true;
  }
  if (abs(v_bat - CACHE_VBAT) > 0.10) {
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
