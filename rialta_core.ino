/*
 * Rialta Control Core
 * ternus@cternus.net
 */

#include <Time.h>

#include <Wire.h>

#include <ArduinoJson.h>

#include <TinyGPS.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

//#include <Adafruit_Fingerprint.h>

/***** SENSORS *****/

/* 10DOF */

Adafruit_10DOF                tdof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

/* GPS */

TinyGPS gps;

/* Fingerprint */



//Adafruit_Fingerprint finger;

/* PIR */

/* Rotary Encoders */

/***** OUTPUT *****/

/* LED Strips */

/* Door Unlocking */

#define LOCK_PIN 2
#define UNLOCK_PIN 3
#define UNLOCKED 0
#define LOCKED 1

#define R_ON LOW
#define R_OFF HIGH

#define RELAY_ON(x) digitalWrite(x, R_ON)
#define RELAY_OFF(x) digitalWrite(x, R_OFF)

/* LCD */

#include <SerialLCD.h>

SerialLCD lcd(4,20,9600,RS232);

/* Logging/BT */

#define JSON_BUFFER_SIZE 2048

#define LCD_SERIAL Serial1
#define GPS_SERIAL Serial1
#define FINGERPRINT_SERIAL Serial2




#define LED_PIN 13
#define BLINK_DELAY_FAST 100

int bootStage = 0;

void bootStageBlink() {
  for (int i = 0; i < bootStage; i++) {
    digitalWrite(LED_PIN, HIGH);
    smartDelay(BLINK_DELAY_FAST);
    digitalWrite(LED_PIN, LOW);
    smartDelay(BLINK_DELAY_FAST);
  }
}

void lcdMsg(String msg) {
  lcd.clear();
  lcd.home();
  lcd.print(msg);
}

void debugPrint(String msg) {
  if (Serial) {
    Serial.println(msg);
  }
  lcdMsg(msg);
}

void bootPrint(String msg) {
  bootStage++;
  debugPrint(msg);
  bootStageBlink();
}

void fatal(String msg) {
  debugPrint(msg);
  /* TODO set rotary encoders to all red */
  while(1) {
    bootStageBlink();
    smartDelay(1000);
  }
}


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  delay(500);

  /* setup relays */
  pinMode(LOCK_PIN, OUTPUT);
  pinMode(UNLOCK_PIN, OUTPUT);
  RELAY_OFF(LOCK_PIN);
  RELAY_OFF(UNLOCK_PIN);

  // start RTC time
//  setSyncProvider(getTeensy3Time);
//  dBlink();

  // start serial

//  bootPrint(F("Starting serial"));
  Serial.begin(115200);

  /*  
   *  There's some trickiness here. GPS_SERIAL and LCD_SERIAL are the same,
   *  since the GPS is RX-only and the LCD is TX-only. Serial1 is pin 0 (RX)
   *  and pin 1 (TX) of the Teensy.
   */

  bootPrint(F("Starting GPS serial"));
  GPS_SERIAL.begin(9600);
  smartDelay(0);

  bootPrint(F("Starting LCD"));
  lcd.init();
  lcd.setContrast(40);
  // Set Backlight
  lcd.setBacklightBrightness(80);
  lcd.clear();
  lcd.home();
  bootPrint(F("LCD OK"));

  // start fingerprint
//  finger = Adafruit_Fingerprint(&FINGERPRINT_SERIAL);
//  dBlink();
  
  // start i2c sensors
  /* Initialise the sensors */

  bootPrint(F("Starting 10DOF"));
  if(!accel.begin()) fatal("Ooops, no ADXL345 detected ... Check your wiring!");
  if(!mag.begin()) fatal("Ooops, no LSM303 detected ... Check your wiring!");
  if(!bmp.begin()) fatal("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
  if(!gyro.begin()) fatal("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");

  bootPrint(F("Boot OK"));

}

void loop() {
  // put your main code here, to run repeatedly:
  smartDelay(1000);
  pushData();
}

void pushData() {
  StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["time"] = now();
  JsonObject& sensors = root.createNestedObject("sensors");

  /* GPS */
  JsonObject& jgps = sensors.createNestedObject("gps");
  float flat, flon;
  uint32_t age;
  jgps["satellites"] = gps.satellites();
  jgps["hdop"] = gps.hdop();
  gps.f_get_position(&flat, &flon, &age);
  jgps["flat"] = flat;
  jgps["flon"] = flon;
  jgps["age"] = age;
  jgps["altitude"] = gps.f_altitude();
  jgps["speed"] = gps.f_speed_kmph();
  jgps["course"] = gps.f_course();

  /* 10DOF */
  JsonObject &tend = sensors.createNestedObject("dof");
  JsonObject &jbmp = tend.createNestedObject("bmp");
  sensors_event_t event;
  bmp.getEvent(&event);
  jbmp["pressure"] = event.pressure;
  float temperature;
  bmp.getTemperature(&temperature);
  jbmp["temperature"] = temperature;
  jbmp["altitude"] = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, 
                                            event.pressure, temperature);
  JsonObject &jgyro = tend.createNestedObject("gyro");
  gyro.getEvent(&event);
  jgyro["x"] = event.gyro.x;
  jgyro["y"] = event.gyro.y;
  jgyro["z"] = event.gyro.z;
  JsonObject &jmag = tend.createNestedObject("mag");
  mag.getEvent(&event);
  jmag["x"] = event.magnetic.x;
  jmag["y"] = event.magnetic.y;
  jmag["z"] = event.magnetic.z;
  JsonObject &jaccel = tend.createNestedObject("accel");
  accel.getEvent(&event);
  jaccel["x"] = event.acceleration.x;
  jaccel["y"] = event.acceleration.y;
  jaccel["z"] = event.acceleration.z;

  /* now send out the data */

  root.prettyPrintTo(Serial);
  
  
}

/* Door locks */

void setDoorState(bool state) {
  debugPrint(state ? "locking" : "unlocking");
  RELAY_ON(state ? LOCK_PIN : UNLOCK_PIN);
  smartDelay(1000); /* Wait for door to (un)lock */
  RELAY_OFF(state ? LOCK_PIN : UNLOCK_PIN);
}

#define LOCK_DOOR() setDoorState(LOCKED);
#define UNLOCK_DOOR() setDoorState(UNLOCKED);

/* GPS */

static void smartDelay(unsigned long ms)
{
  
  unsigned long age;  
  int Year;
  byte Month, Day, Hour, Minute, Second;
  unsigned long start = millis();
  do 
  {
    while (GPS_SERIAL.available())
      gps.encode(GPS_SERIAL.read());
  } while (millis() - start < ms);

  String cmd;
  while (Serial.available()) {
      cmd = Serial.readString();
      if (cmd.indexOf("unlock") != -1) {
          UNLOCK_DOOR();
      } else if (cmd.indexOf("lock") != -1) {
          LOCK_DOOR();
      }
  }
  
  /* set time from gps */
  gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);
  if (age < 500)
    setTime(Hour, Minute, Second, Day, Month, Year);
}

