// define DEBUG_SERIAL to enable serial debug
#define DEBUG_SERIAL

// Screen Layout
//
//  0123456789012345678901234567890123456789
// 0T(C): 32.5  Hum(%): 77  Pr(Pa): 1024
// 1UV: 7  CO2(ppm): 45  TVOC(ppb): 888
// 2PM(Î¼g/m3) 1:77  2.5: 77  10: 777
// 3Bat(V): 8.3
//
// Screen Layout
//  01234567890123456789
// 0T(C): 32.5  H(%): 77
// 1UV: 7  Pr(hPa): 1024
// 2Bat(V): 8.3
// 3TVOC(ppb): 888
// 4CO2(ppm): 45 
// 5PM(Î¼g/m3)1/2.5/10
// 612 / 34 / 567
// 7


//**************************************
// OLED LCD Screen Indicator

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_ADDR        0x3C
#define SCREEN_WIDTH     128 // OLED display width, in pixels
#define SCREEN_HEIGHT    64 // OLED display height, in pixels. 32 or 64
#define OLED_FONT_WIDTH  6
#define OLED_FONT_HEIGHT 8
#define OLED_RESET       4 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup_OLED_I2C() {
  #ifdef DEBUG_SERIAL
    Serial.println(F("OLED I2C SSD1306 test"));
  #endif

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 failed"));
    while(true);
  }

  // Clear the buffer
  oled.clearDisplay();

  //oled.setTextSize(1);      // Normal 1:1 pixel scale
  oled.setTextColor(WHITE); // Draw white text
  //screens_print("Weather Station Init", 0, 1);
  //display.cp437(true);         // Use full 256 char 'Code Page 437' font

  /*oled.drawPixel(0, 0, WHITE);
  oled.drawPixel(127, 0, WHITE);
  oled.drawPixel(0, 63, WHITE);
  oled.drawPixel(127, 63, WHITE);
  oled.drawPixel(63, 31, WHITE);
  oled.println("Weather Station Init");
  oled.display(); */

  #ifdef DEBUG_SERIAL
    Serial.println(F("OLED I2C ready"));
  #endif
}

void one_loop_start_OLED_I2C() {
  oled.clearDisplay();
}

void one_loop_end_OLED_I2C() {
  oled.display();
}

void screens_print(String s, int col, int li) {
  oled.setCursor(col * OLED_FONT_WIDTH, li * OLED_FONT_HEIGHT);
  oled.print(s);
}

//**************************************
// Temperature and Humidity
// Sensor SI7021 (legacy)

#include "Adafruit_Si7021.h"

Adafruit_Si7021 si7021 = Adafruit_Si7021();

void setup_Si7021() {

  #ifdef DEBUG_SERIAL
    Serial.println(F("Si7021 test"));
  #endif
  
  if (!si7021.begin()) {
    Serial.println(F("Si7021 failed"));
    while (true);
  }

  #ifdef DEBUG_SERIAL
    Serial.print(F("Found model "));
    switch(si7021.getModel()) {
      case SI_Engineering_Samples:
        Serial.print(F("SI engineering samples")); break;
      case SI_7013:
        Serial.print(F("Si7013")); break;
      case SI_7020:
        Serial.print(F("Si7020")); break;
      case SI_7021:
        Serial.print(F("Si7021")); break;
      case SI_UNKNOWN:
      default:
        Serial.print(F("Unknown"));
    }
    Serial.print(F(" Rev("));
    Serial.print(si7021.getRevision());
    Serial.print(F(")"));
    Serial.print(F(" Serial #")); Serial.print(si7021.sernum_a, HEX); Serial.println(si7021.sernum_b, HEX);
    Serial.println(F("Si7021 ready"));
  #endif
}

void one_loop_Si7021() {
  #ifdef DEBUG_SERIAL
    Serial.print(F("\tHumidity: "));
    Serial.print(si7021.readHumidity(), 2);
    Serial.print(F("\tTemp: "));
    Serial.print(si7021.readTemperature(), 2);
  #endif

  String sTH = "";
  sTH = sTH + F("T(C): ") + String(si7021.readTemperature(), 1) + F(" H(%): ") + String(si7021.readHumidity(), 0);
  screens_print(sTH, 0, 0);
}

//**************************************
// CO and TVOC and temperature
// Sensor CCS811

#include "Adafruit_CCS811.h"

#define CCS811_TEMPERATURE_OFFSET  25.0

Adafruit_CCS811 ccs811;

void setup_CCS811() {
  
  #ifdef DEBUG_SERIAL
    Serial.println(F("CCS811 test"));
  #endif
  
  if(!ccs811.begin()){
    Serial.println(F("CCS811 failed"));
    while(true);
  }

  //calibrate temperature sensor
  while(!ccs811.available());
  //ccs.setTempOffset(ccs.calculateTemperature() - CCS811_TEMPERATURE_OFFSET);
  #ifdef DEBUG_SERIAL
    Serial.println(F("Found sensor, available now"));
  #endif
}

void one_loop_CCS811() {
  if(ccs811.available()){
    if(!ccs811.readData()){
      #ifdef DEBUG_SERIAL
        Serial.print(F("\tCO2(ppm): "));
        Serial.print(ccs811.geteCO2());
        Serial.print(F("\tTVOC(ppb): "));
        Serial.print(ccs811.getTVOC());
      #endif
      //do not show temp, it is very inaccurate
      //Serial.print(F("ppb\tTemp:"));
      //Serial.print(temp);
      //create 2 blocks to save on mem for string allocation
      {
        String sTVOC = F("TVOC(ppb): ");
        sTVOC = sTVOC + ccs811.getTVOC();
        screens_print(sTVOC, 0, 2);
      }
      {
        String sCO2 = F("CO2(ppm): ");
        sCO2 = sCO2 + ccs811.geteCO2();
        screens_print(sCO2, 0, 3);
      }
    }
    else{
      Serial.println(F("CCS811 loop failed"));
    }
  }
}

//**************************************
// UV
// Sensor VEML6075

#include "Adafruit_VEML6075.h"

Adafruit_VEML6075 uv = Adafruit_VEML6075();

void setup_VEML6075() {
  #ifdef DEBUG_SERIAL
    Serial.println(F("VEML6075 test"));
  #endif
  if (!uv.begin()) {
    Serial.println(F("VEML6075 failed"));
    while(true);
  }
  #ifdef DEBUG_SERIAL
    Serial.println(F("Found VEML6075 sensor"));
  #endif
}

void one_loop_VEML6075() {
  int uvi = uv.readUVI();
  if(uvi < 0)
    uvi = 0;
  #ifdef DEBUG_SERIAL
    Serial.print(F("\tUVIndex: "));
    Serial.print(uvi);
  #endif

  String sUV = F("UV: ");
  sUV = sUV + uvi;
  screens_print(sUV, 0, 1);
}

//**************************************
// PM (particule matters)
// Sensor PMS7003

#include <SoftwareSerial.h>

#define PMS_7300_SET_PIN A1
#define WRITE_PIN_33V 168 //5V is 255, 3.3V is 168
#define ON_OFF_TIME_IN_SECONDS 60 //switch on and off switch

SoftwareSerial pmsSerial(2, 3);

void setup_PMS7003() {
  #ifdef DEBUG_SERIAL
    Serial.println(F("PMS7003 serial on"));
  #endif
  pinMode(PMS_7300_SET_PIN, OUTPUT);
  setFanOn(true);
  pmsSerial.begin(9600);
}

struct pms5003dataStruct {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
 
struct pms5003dataStruct pms5003data;

void empty_buffer_PMS7003() {
  while (pmsSerial.read()!=-1) {};
}

void setFanOn(boolean isOn) {
  if(isOn)
    analogWrite(PMS_7300_SET_PIN, WRITE_PIN_33V);
   else
    analogWrite(PMS_7300_SET_PIN, 0);  
}

/*
void setFanOnOffBasedOnTime() {
  int minutes = millis() / 1000 / ON_OFF_TIME_IN_SECONDS;
  bool isPms7300ShouldBeUp = ((minutes % 2) == 1);
  if(isPms7300ShouldBeUp != isPms7300Up) {
    isPms7300Up = isPms7300ShouldBeUp;
    setFanOnOff();
  }
}
*/

void one_loop_PMS7003() {
  if (readPMSdata(&pmsSerial)) {
    // reading data was successful!
    /*
    Serial.println();
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Concentration Units (standard)"));
    Serial.print(F("PM 1.0: ")); Serial.print(pms5003data.pm10_standard);
    Serial.print(F("\t\tPM 2.5: ")); Serial.print(pms5003data.pm25_standard);
    Serial.print(F("\t\tPM 10: ")); Serial.println(pms5003data.pm100_standard);
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Concentration Units (environmental)"));
    Serial.print(F("PM 1.0: ")); Serial.print(pms5003data.pm10_env);
    Serial.print(F("\t\tPM 2.5: ")); Serial.print(pms5003data.pm25_env);
    Serial.print(F("\t\tPM 10: ")); Serial.println(pms5003data.pm100_env);
    Serial.println(F("---------------------------------------"));
    Serial.print(F("Particles > 0.3um / 0.1L air:")); Serial.println(pms5003data.particles_03um);
    Serial.print(F("Particles > 0.5um / 0.1L air:")); Serial.println(pms5003data.particles_05um);
    Serial.print(F("Particles > 1.0um / 0.1L air:")); Serial.println(pms5003data.particles_10um);
    Serial.print(F("Particles > 2.5um / 0.1L air:")); Serial.println(pms5003data.particles_25um);
    Serial.print(F("Particles > 5.0um / 0.1L air:")); Serial.println(pms5003data.particles_50um);
    Serial.print(F("Particles > 10.0 um / 0.1L air:")); Serial.println(pms5003data.particles_100um);
    Serial.println(F("---------------------------------------"));
    */
    #ifdef DEBUG_SERIAL
      Serial.print(F("\tPM 1.0: ")); Serial.print(pms5003data.pm10_env);
      Serial.print(F("\tPM 2.5: ")); Serial.print(pms5003data.pm25_env);
      Serial.print(F("\tPM 10: ")); Serial.print(pms5003data.pm100_env);    
    #endif
    String sStats = F(" ");
    sStats = sStats + pms5003data.pm10_env + " / " + pms5003data.pm25_env + " / " + pms5003data.pm100_env;
    screens_print(F("PM(mug/m3) 1/2.5/10"), 0, 4);
    screens_print(sStats, 0, 5);
  }
  empty_buffer_PMS7003();
}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
 
  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print(F("0x"));
    Serial.print(buffer[i], HEX);
    Serial.print(F(", "));
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&pms5003data, (void *)buffer_u16, 30);
 
  if (sum != pms5003data.checksum) {
    //Serial.println(F("Checksum failure"));
    return false;
  }
  // success!
  return true;
}

//**************************************
// Athmospheric Pressure and Temperature
// Sensor BMP280

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp280;

void setup_BMP280() {
  #ifdef DEBUG_SERIAL
    Serial.println(F("BMP280 test"));
  #endif
  if (!bmp280.begin()) {  
    Serial.println(F("BMP280 failed"));
    while (true);
  }
  #ifdef DEBUG_SERIAL
    Serial.println(F("Found BMP280 sensor"));
  #endif
}

void one_loop_BMP280() {
  #ifdef DEBUG_SERIAL
    Serial.print(F("\t[Temp: "));
    Serial.print(bmp280.readTemperature());
    Serial.print(F("]"));
    Serial.print(F("\tPressure(hPa): "));
    Serial.print(bmp280.readPressure()/100);
  #endif

  String sPr = F("Pr(hPa): ");
  sPr = sPr + String(bmp280.readPressure()/100, 0);
  screens_print(sPr, 7, 1);
}

//**************************************
// No
// Sensor MICS-4514

/*
const int NO2_pin = A0;      // Analog PIN 6 to read the NO2-sensor

void setup_MICS4514() {
  Serial.println(F("MICS4514 setup (nothing to do)"));
}

void one_loop_MICS4514() {
  //https://myscope.net/auswertung-der-airpi-gas-sensoren/
  int v = analogRead(NO2_pin);
  float vf = v;
  float R0 = 1022000; //fresh air, estimated
  //Serial.print("\tMICS4514.NO2.Vout: " + String(vf));
  float RsR0 = (1024.0 - vf) / vf * 22000 / R0; //1024 to avoid 0
  float ppmNO2 = pow(10, 0.9682*log(RsR0)/log(10) - 0.8108);
  Serial.print("\tNO2(ppm): " + String(ppmNO2));
}

*/

//**************************************
// Battery Indicator

#define BATTERY_INDICATOR_PIN A2      // Analog PIN 6 to read the NO2-sensor

void setup_BatteryIndicator() {
  #ifdef DEBUG_SERIAL
    Serial.println(F("BatteryIndicator setup (nothing to do)"));
  #endif
}

void one_loop_BatteryIndicator() {
  float v = analogRead(BATTERY_INDICATOR_PIN); //0 to 1023, 1023 mapping to 5V
  float vBattery = 2 * 5 * v / 1023; //*2 as we use 2 equal resistors to devide voltage
  #ifdef DEBUG_SERIAL
    Serial.print("\tBattery(V): " + String(vBattery));
  #endif
  String sBat = F("Bat(V): ");
  sBat = sBat + String(vBattery, 1);
  screens_print(sBat, 0, 6);
}

//**************************************
// I2C Port Scanner

#ifdef DEBUG_SERIAL
void scanI2C() {
  Serial.println(F("i2c scan starting"));  
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0) {
      Serial.print(F("Found i2c address: "));
      Serial.print(i, DEC);
      Serial.print(F(" (0x"));
      Serial.print(i, HEX);
      Serial.println (")");
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println(F("i2c scan done."));  
}

#endif

//**************************************
// Main Loop

#define LOOP_DELAY_MS 1000

void setup() {
  Serial.begin(9600);
  // wait for serial port to open
  while (!Serial) {
    delay(10);
  }
  #ifdef DEBUG_SERIAL
    scanI2C();
  #endif
  setup_OLED_I2C();
  setup_BatteryIndicator();
  //setup_MICS4514();
  setup_Si7021();
  setup_CCS811();
  setup_VEML6075();
  setup_BMP280();
  setup_PMS7003();
}

void loop() {
  #ifdef DEBUG_SERIAL
    Serial.print(F("Time: "));
    Serial.print(millis());
  #endif

  one_loop_start_OLED_I2C();
  one_loop_BatteryIndicator();
  //one_loop_MICS4514();
  one_loop_Si7021();
  one_loop_CCS811();
  one_loop_VEML6075();
  one_loop_BMP280();
  one_loop_PMS7003();
  one_loop_end_OLED_I2C();

  #ifdef DEBUG_SERIAL
    Serial.println();
  #endif
  delay(LOOP_DELAY_MS);
}
