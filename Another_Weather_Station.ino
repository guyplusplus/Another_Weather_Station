// define DEBUG_SERIAL to enable serial debug
#define XDEBUG_SERIAL

// Screen Layout
//
//  0123456789012345678901234567890123456789
// 0T(C): 32.5  Hum(%): 77  Pr(Pa): 1024
// 1UV: 7  CO2(ppm): 45  TVOC(ppb): 888
// 2PM(μg/m3) 1:77  2.5: 77  10: 777
// 3Bat(V): 8.3
//
// Screen 0
//  01234567890123456789
// 0T(C): 32.5  H(%): 77
// 1UV: 7  Pr(hPa): 1024
// 2Bat(V): 8.3
// 3
//
// Screen 1
//  01234567890123456789
// 0CO2(ppm): 45 
// 1TVOC(ppb): 888
// 2PM(μg/m3)1/2.5/10
// 377 / 77 / 777

//**************************************
// I2C LCD Screen Indicator

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
int lcd_status = 0; //0 off, 1 on

void setup_LCD_I2C() {
  #ifdef SERIAL_DEBUG
    Serial.println("LCD I2C test");
  #endif
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,1);
  lcd.print("Weather Station Init");
  lcd.display();
  lcd_status = 1;
  #ifdef SERIAL_DEBUG
    Serial.println("LCD I2C ready");
  #endif
}


//**************************************
// OLED LCD Screen Indicator

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_FONT_WIDTH 6
#define OLED_FONT_HEIGHT 8

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int oled_status = 0; //0 off, 1 on

void setup_OLED_I2C() {
  #ifdef SERIAL_DEBUG
    Serial.println("OLED I2C test");
  #endif

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("OLED SSD1306 allocation failed"));
    oled_status = 0;
    return;
  }
  oled_status = 1;

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  //oled.display();
  //delay(2000); // Pause for 2 seconds to show adafruit logo

  // Clear the buffer
  oled.clearDisplay();

  oled.setTextSize(1);      // Normal 1:1 pixel scale
  oled.setTextColor(WHITE); // Draw white text
  oled.setCursor(0 * OLED_FONT_WIDTH, 1 * OLED_FONT_HEIGHT);
  //display.cp437(true);         // Use full 256 char 'Code Page 437' font

  oled.println("Weather Station Init");
  oled.display(); 

  #ifdef SERIAL_DEBUG
    Serial.println("OLED I2C ready");
  #endif
}

void one_loop_end_OLED_I2C() {
  if(oled_status == 1)
    oled.display();
}


//**************************************
// All Screens

int screen_page = 0; //0 or 1

void setup_screens() {
  setup_OLED_I2C();
  setup_LCD_I2C();
}

void clear_screens() {
  if(oled_status == 1)
    oled.clearDisplay();
  if(lcd_status == 1)
    lcd.clear();
}

void one_loop_start_screens() {
  int new_screen_page = ((int)(millis() / 1000 / 5)) % 2;
  if(screen_page != new_screen_page) {
    screen_page = new_screen_page;
    clear_screens();
  }
  else {
    //clear only oled
    if(oled_status == 1)
      oled.clearDisplay();
  }
}

void one_loop_end_screens() {
  one_loop_end_OLED_I2C();
}

void screens_print(String s, int col, int li) {
  if(li<4 && screen_page == 0) {
    if(lcd_status == 1) {
      lcd.setCursor(col, li);
      lcd.print(s);
    }
    if(oled_status == 1) {
      oled.setCursor(col * OLED_FONT_WIDTH, li * OLED_FONT_HEIGHT);
      oled.print(s);
    }
    return;
  }
  if(li>=4 && screen_page == 1) {
    if(lcd_status == 1) {
      lcd.setCursor(col, li - 4);
      lcd.print(s);
    }
    if(oled_status == 1) {
      oled.setCursor(col * OLED_FONT_WIDTH, (li - 4) * OLED_FONT_HEIGHT);
      oled.print(s);
    }
  }
}

//**************************************
// Temperature and Humidity
// Sensor SI7021 (legacy)

#include "Adafruit_Si7021.h"

Adafruit_Si7021 sensor = Adafruit_Si7021();

void setup_Si7021() {

  #ifdef SERIAL_DEBUG
    Serial.println("Si7021 test");
  #endif
  
  if (!sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (true);
  }

  #ifdef DEBUG_SERIAL
    Serial.print("Found model ");
    switch(sensor.getModel()) {
      case SI_Engineering_Samples:
        Serial.print("SI engineering samples"); break;
      case SI_7013:
        Serial.print("Si7013"); break;
      case SI_7020:
        Serial.print("Si7020"); break;
      case SI_7021:
        Serial.print("Si7021"); break;
      case SI_UNKNOWN:
      default:
        Serial.print("Unknown");
    }
    Serial.print(" Rev(");
    Serial.print(sensor.getRevision());
    Serial.print(")");
    Serial.print(" Serial #"); Serial.print(sensor.sernum_a, HEX); Serial.println(sensor.sernum_b, HEX);
    Serial.println("Si7021 ready");
  #endif
}

void one_loop_Si7021() {
  #ifdef DEBUG_SERIAL
    Serial.print("\tHumidity: ");
    Serial.print(sensor.readHumidity(), 2);
    Serial.print("\tTemp: ");
    Serial.print(sensor.readTemperature(), 2);
  #endif

  String sTH = "T(C): " + String(sensor.readTemperature(), 1) + " H(%): " + String(sensor.readHumidity(), 0);
  screens_print(sTH, 0, 0);
}

//**************************************
// CO and TVOC and temperature
// Sensor CCS811

#include "Adafruit_CCS811.h"

Adafruit_CCS811 ccs;

void setup_CCS811() {
  
  #ifdef SERIAL_DEBUG
    Serial.println("CCS811 test");
  #endif
  
  if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(true);
  }

  //calibrate temperature sensor
  while(!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);
  #ifdef SERIAL_DEBUG
    Serial.println("Found sensor, available now");
  #endif
}

void one_loop_CCS811() {
  if(ccs.available()){
    float temp = ccs.calculateTemperature();
    if(!ccs.readData()){
      #ifdef SERIAL_DEBUG
        Serial.print("\tCO2(ppm): ");
        Serial.print(ccs.geteCO2());
        Serial.print("\tTVOC(ppb): ");
        Serial.print(ccs.getTVOC());
      #endif
      //do not show temp, it is very inaccurate
      //Serial.print("ppb\tTemp:");
      //Serial.print(temp);
      String sCO2 = "";
      sCO2 = sCO2 + "CO2(ppm): " + ccs.geteCO2();
      String sTVOC = "";
      sTVOC = sTVOC + "TVOC(ppb): " + ccs.getTVOC();

      screens_print(sCO2, 0, 4);
      screens_print(sTVOC, 0, 5);
    }
    else{
      Serial.println("one_loop_CCS811 ERROR!");
    }
  }
}

//**************************************
// UV
// Sensor VEML6075

#include "Adafruit_VEML6075.h"

Adafruit_VEML6075 uv = Adafruit_VEML6075();

void setup_VEML6075() {
  #ifdef SERIAL_DEBUG
    Serial.println("VEML6075 test");
  #endif
  if (!uv.begin()) {
    Serial.println("Failed to communicate with VEML6075 sensor, check wiring?");
    while(true);
  }
  #ifdef SERIAL_DEBUG
    Serial.println("Found VEML6075 sensor");
  #endif
}

void one_loop_VEML6075() {
  int uvi = uv.readUVI();
  if(uvi < 0)
    uvi = 0;
  #ifdef SERIAL_DEBUG
    Serial.print("\tUVIndex: ");
    Serial.print(uvi);
  #endif

  String sUV = "";
  sUV = sUV + "UV: " + uvi;
  screens_print(sUV, 0, 1);
}

//**************************************
// PM (particule matters)
// Sensor PMS7003

#include <SoftwareSerial.h>

const int pms7300SetPin = A1;
const int WRITE_PIN_33V = 168; //5V is 255, 3.3V is 168
const int ON_OFF_TIME_IN_SECONDS = 60; //switch on and off switch
SoftwareSerial pmsSerial(2, 3);
bool isPms7300Up = true;

void setup_PMS7003() {
  #ifdef SERIAL_DEBUG
    Serial.println("PMS7003 serial on");
  #endif
  setFanOnOff();
  pinMode(pms7300SetPin, OUTPUT);
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

void setFanOnOff() {
  if(isPms7300Up)
    analogWrite(pms7300SetPin, WRITE_PIN_33V);
   else
    analogWrite(pms7300SetPin, 0);  
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
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: "); Serial.print(pms5003data.pm10_standard);
    Serial.print("\t\tPM 2.5: "); Serial.print(pms5003data.pm25_standard);
    Serial.print("\t\tPM 10: "); Serial.println(pms5003data.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: "); Serial.print(pms5003data.pm10_env);
    Serial.print("\t\tPM 2.5: "); Serial.print(pms5003data.pm25_env);
    Serial.print("\t\tPM 10: "); Serial.println(pms5003data.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(pms5003data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(pms5003data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(pms5003data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(pms5003data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(pms5003data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(pms5003data.particles_100um);
    Serial.println("---------------------------------------");
    */
    #ifdef SERIAL_DEBUG
      Serial.print("\tPM 1.0: "); Serial.print(pms5003data.pm10_env);
      Serial.print("\tPM 2.5: "); Serial.print(pms5003data.pm25_env);
      Serial.print("\tPM 10: "); Serial.print(pms5003data.pm100_env);    
    #endif
    String sTitle = "PM(mug/m3)1/2.5/10";
    String sStats = "";
    sStats = sStats + pms5003data.pm10_env + " / " + pms5003data.pm25_env + " / " + pms5003data.pm100_env;
    // 2PM(μg/m3)1/2.5/10
    // 377 / 77 / 777
    screens_print(sTitle, 0, 6);
    screens_print(sStats, 0, 7);
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
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
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
    //Serial.println("Checksum failure");
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

Adafruit_BMP280 bmp;

void setup_BMP280() {
  #ifdef SERIAL_DEBUG
    Serial.println("BMP280 test");
  #endif
  if (!bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (true);
  }
  #ifdef SERIAL_DEBUG
    Serial.println("Found BMP280 sensor");
  #endif
}

void one_loop_BMP280() {
  #ifdef SERIAL_DEBUG
    Serial.print("\tTemp: ");
    Serial.print(bmp.readTemperature());
    Serial.print("\tPressure(Pa): ");
    Serial.print(bmp.readPressure());
  #endif

  String sPr = "";
  sPr = sPr + "Pr(hPa): " + String(bmp.readPressure()/100, 0);
  screens_print(sPr, 7, 1);
}

//**************************************
// No
// Sensor MICS-4514

/*
const int NO2_pin = A0;      // Analog PIN 6 to read the NO2-sensor

void setup_MICS4514() {
  Serial.println("MICS4514 setup (nothing to do)");
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

const int BatteryIndicator_pin = A2;      // Analog PIN 6 to read the NO2-sensor

void setup_BatteryIndicator() {
  #ifdef SERIAL_DEBUG
    Serial.println("BatteryIndicator setup (nothing to do)");
  #endif
}

void one_loop_BatteryIndicator() {
  float v = analogRead(BatteryIndicator_pin); //0 to 1023, 1023 mapping to 5V
  float vBattery = 2 * 5 * v / 1023; //*2 as we use 2 equal resistors to devide voltage
  #ifdef SERIAL_DEBUG
    Serial.print("\tBattery(V): " + String(vBattery));
  #endif
  String sBat = "";
  sBat = sBat + "Bat(V): " + String(vBattery, 1);
  screens_print(sBat, 0, 2);
}

//**************************************
// Main Loop

#ifdef SERIAL_DEBUG
void scanI2C() {
  Serial.println("i2c scan starting");  
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0) {
      Serial.print("Found i2c address: ");
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println (")");
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println("i2c scan done.");  
}
#endif

void setup() {
  Serial.begin(9600);
  // wait for serial port to open
  while (!Serial) {
    delay(10);
  }
  #ifdef SERIAL_DEBUG
    scanI2C();
  #endif
  setup_screens();
  setup_BatteryIndicator();
  //setup_MICS4514();
  setup_Si7021();
  setup_CCS811();
  setup_VEML6075();
  setup_BMP280();
  setup_PMS7003();
  clear_screens();
}

void loop() {
  #ifdef SERIAL_DEBUG
    Serial.print("Time: ");
    Serial.print(millis());
  #endif

  one_loop_start_screens();
  one_loop_BatteryIndicator();
  //one_loop_MICS4514();
  one_loop_Si7021();
  one_loop_CCS811();
  one_loop_VEML6075();
  one_loop_BMP280();
  one_loop_PMS7003();
  one_loop_end_screens();

  #ifdef SERIAL_DEBUG
    Serial.println();
  #endif
  delay(1000);
}
