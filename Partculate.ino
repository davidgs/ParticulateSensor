
#include <Wire.h>
// We will be using the I2C hardware interface on the Arduino in
// combination with the built-in Wire library to interface.
// Arduino analog input 5 - I2C SCL
// Arduino analog input 4 - I2C SDA
#include "esp_system.h"
#include <InfluxDb.h>
#include <WiFi.h>
//needed for library
#include <DNSServer.h>

#include "HardwareSerial.h"


//#define DEBUG //enable/disable serial debug output

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTHEX(x) Serial.print(x, HEX)
#define DEBUG_PRINTLNHEX(x) Serial.println(x, HEX)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTHEX(x)
#define DEBUG_PRINTLNHEX(x)
#define DEBUG_PRINTLN(x)
#endif


HardwareSerial pmsSerial = Serial1;
//#include <ESP8266WebServer.h>
#define POWER_PIN A13
#define BATCH_SIZE 5
#define INFLUXDB_HOST "YOURHOSTNAME"
Influxdb influx(INFLUXDB_HOST);
WiFiClient client;
const char* wifi_ssid = "YOURSSID"; // "Murray Home";
const char* wifi_password = "YOURPASSWORD"; // "Toby64.Mime!";
int failure = 0;
int status = WL_IDLE_STATUS;     // the Wifi radio's status
WiFiClient espClient;
byte mac[6];
const int loopTimeCtl = 0;
hw_timer_t *timer = NULL;
int batchCount = 0;

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define MINUTE =60
#define TIME_TO_SLEEP  1        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;


struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;

void(* resetFunc) (void) = 0; //declare reset function @ address 0
void IRAM_ATTR resetModule() {
  ets_printf("reboot\n");
  resetFunc();
}

void setup() {
  Serial.begin(115200);

  delay(1000);
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  // The PM SEnsor runs at 9600 baud
  pmsSerial.begin(9600);
  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  pinMode(POWER_PIN, INPUT);

  setup_wifi();
  influx.setBucket("YOURBUCKET");
  // We're using InfluxDB 2.0
  influx.setVersion(2);
  influx.setOrg("YOURORG");
  influx.setPort(9999);
  influx.setToken("YOURTOKEN");
  influx2.setBucket("telegraf");
  //pinMode(loopTimeCtl, INPUT_PULLUP);
  timer = timerBegin(0, 80, true); //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);
  timerAlarmWrite(timer, 5000000, false); //set time in us
  timerAlarmEnable(timer); //enable interrupt
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR * MINUTE);
  DEBUG_PRINTLN("Setup ESP32 to sleep for " + String(TIME_TO_SLEEP) +
                " Seconds");
}
void setup_wifi() {
  char mySSID[14];
  findSSID();
  // Start by connecting to the WiFi network
  DEBUG_PRINTLN();
  DEBUG_PRINT("Connecting to ");
  DEBUG_PRINTLN(mySSID);
  WiFi.macAddress(mac);
  DEBUG_PRINT("MAC: ");
  DEBUG_PRINTHEX(mac[5]);
  DEBUG_PRINT(":");
  DEBUG_PRINTHEX(mac[4]);
  DEBUG_PRINT(":");
  DEBUG_PRINTHEX(mac[3]);
  DEBUG_PRINT(":");
  DEBUG_PRINTHEX(mac[2]);
  DEBUG_PRINT(":");
  DEBUG_PRINTHEX(mac[1]);
  DEBUG_PRINT(":");
  DEBUG_PRINTLNHEX(mac[0]);
  WiFi.mode(WIFI_STA);
  Serial.printf("Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode(WIFI_STA) ? "" : "Failed!");

  while (int v = wait_for_wifi() > 0) {
    findSSID();
  }

  DEBUG_PRINT("IP address: ");
  DEBUG_PRINTLN(WiFi.localIP());

}

void findSSID() {
  boolean foundSSID = false;
  int searches = 0;
  while (!foundSSID) {
    if (searches > 25) {
      resetFunc();
    }
    DEBUG_PRINT("Searching for WiFi Network: "); DEBUG_PRINTLN(wifi_ssid);
    int numberOfNetworks = WiFi.scanNetworks();
    for (int i = 0; i < numberOfNetworks; i++) {
      DEBUG_PRINT("Network name: ");
      DEBUG_PRINTLN(WiFi.SSID(i));
      DEBUG_PRINT("Signal strength: ");
      DEBUG_PRINTLN(WiFi.RSSI(i));
      if (WiFi.SSID(i) == wifi_ssid) {
        foundSSID = true;
        DEBUG_PRINT("Found ");
        DEBUG_PRINTLN(WiFi.SSID(i));
        DEBUG_PRINTLN(WiFi.psk().c_str());
        WiFi.mode(WIFI_STA);
        Serial.printf("Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode(WIFI_STA) ? "" : "Failed!");
        WiFi.begin(wifi_ssid, wifi_password);
        return;
      }
      DEBUG_PRINTLN("-----------------------");
    }
    searches++;
  }
}

int wait_for_wifi()
{
  int tries = 30;
  int thisTry = 0;
  DEBUG_PRINTLN("waiting for Wifi");
  while (WiFi.status() != WL_CONNECTED && thisTry < tries) {
    delay(1000);
    WiFi.printDiag(Serial);
    DEBUG_PRINTLN(WiFi.psk().c_str());
    thisTry += 1;
  }
  if (WiFi.status() != WL_CONNECTED) {
    return 1;
  }
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("WiFi connected");
  return 0;
}



void loop() {
  timerWrite(timer, 0); //reset timer (feed watchdog)
  int power = analogRead(POWER_PIN) * 2;
  Serial.printf("Read Power: %d, X: %d\n", analogRead(POWER_PIN), power);
  int rssi = WiFi.RSSI();
  double f = double(power) / double(1000);
  bool sense_on = false;
  InfluxData row("particulate");
  row.addTag("sensor", "pm_sensor");
  if (readPMSdata(&pmsSerial)) {
    sense_on = true;
    row.addValue("pm_10_standard", data.pm10_standard);
    row.addValue("pm_25_standard", data.pm25_standard);
    row.addValue("pm_100_standard", data.pm100_standard);
    row.addValue("particles_03µm", data.particles_03um);
    row.addValue("particles_05µm", data.particles_05um);
    row.addValue("particles_10µm", data.particles_10um);
    row.addValue("particles_25µm", data.particles_25um);
    row.addValue("particles_50µm", data.particles_50um);
    row.addValue("particles_100µm", data.particles_100um);
  }
  row.addValue("battery", power);
  row.addValue("RSSI", rssi);
  influx.prepare(row);
  batchCount += 1;
  if (batchCount >= BATCH_SIZE) {
    influx.write();
    batchCount = 0;
    if (!sense_on) {
      esp_deep_sleep_start();
    }
  }
  delay(1000);

}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : DEBUG_PRINTLN("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : DEBUG_PRINTLN("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : DEBUG_PRINTLN("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : DEBUG_PRINTLN("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : DEBUG_PRINTLN("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    DEBUG_PRINTLN("Serial not available");
    return false;
  }
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    DEBUG_PRINTLN("No start byte yet...");
    return false;
  }
  // Now read all 32 bytes
  if (s->available() < 32) {
    DEBUG_PRINTLN("Read less than 32 bytes!");
    return false;
  }
  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }
  // debugging
  //for (uint8_t i=2; i<32; i++) {
  //  DEBUG_PRINT("0x"); DEBUG_PRINTHEX(buffer[i], HEX); DEBUG_PRINT(", ");
  // }
  // DEBUG_PRINTLN();
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    DEBUG_PRINTLN("Checksum failure");
    return false;
  }
  // success!
  return true;
}
