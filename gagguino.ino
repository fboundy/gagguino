#include <WiFi.h>
#include <Adafruit_MAX31865.h>
#include <avr/EEPROM.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#define I2C_ADDRESS          0x3C     

//Digital Pins
//#define SER_RX                0
//#define SER_TX                1
// #define SPARE                2
// #define SPARE                3
#define SPI_SD_SS               4
#define SPI_MAX_SS              5
#define HEAT_RELAY              6
//#define WIFI_HSK              7
//#define SPARE                 8
//#define SPARE                 9 
#define SPI_WIFI_SS            10
//#define SPI_MOSI             11
//#define SPI_MISO             12
//#define SPI_SCK              13

//Analogue Pin
#define PRESSURE_PIN            0
#define FLOW_PIN                1
//#define SPARE                 2
//#define SPARE                 3
//#define IC2_SDA               4
//#define IC2_SCK               5

#define PID_CYCLE             200
#define PWM_CYCLE            1000

//PID Gains
#define BREW_TEMP           103.0
#define WINDUP_GUARD_GAIN      35
#define P_GAIN                 15
#define I_GAIN                 10
#define D_GAIN                 40

#define PGAIN_ADR               0
#define IGAIN_ADR               4
#define DGAIN_ADR               8

//#define BREW_TEMP_ADDRESS    12

SSD1306AsciiWire oled;
Adafruit_MAX31865 max = Adafruit_MAX31865(SPI_MAX_SS);

boolean writeDebug = false;
boolean plotData = true;
boolean debugPID = false;
boolean useSavedGain = false;

boolean useWifi = true;
char ssid[] = "Dlink12";     //  your network SSID (name) 
char pass[] = "4d9a4d4652";    // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

float currTemp;
float setTemp;
float lastTemp;

float pGain;
float iGain;
float dGain;

float iState;

boolean heaterState;

float heatPower = 0;
int heatCycles = 0;

unsigned long currentTime;
unsigned long lastPID;
unsigned long lastPWM;
unsigned long lastPress;

long elapsedTime = 0;
 
void setup() {
  pinMode(SPI_WIFI_SS, OUTPUT);
  pinMode(SPI_MAX_SS, OUTPUT);
  pinMode(HEAT_RELAY, OUTPUT);
  pinMode(PRESSURE_PIN, INPUT);
  pinMode(FLOW_PIN, INPUT);
  
  // initialize serial:
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  
  _turnHeatElementOnOff(0); 

  if (useWifi){
    digitalWrite(SPI_WIFI_SS, LOW);
  // attempt to connect using WPA2 encryption:
//    Serial.println("Attempting to connect to WPA network...");
    oled.println("Connecting to Wifi");
    oled.print("Network: ");
//    display.setTextSize(2);
    oled.println(ssid);
    oled.println();

    status = WiFi.begin(ssid, pass);

  // if you're not connected, stop here:
    if ( status != WL_CONNECTED) { 
//      Serial.println("Couldn't get a wifi connection");
      oled.println("FAILED");
      useWifi = false;
    } 
  // if you are connected, print out info about the connection:
    else {
//      Serial.println("Connected to network");
      oled.println("CONNECTED");
      oled.println();
  // print your WiFi shield's IP address:
      IPAddress ip = WiFi.localIP();
      oled.print("IP Address: ");
      oled.println(ip);
//      Serial.print("IP Address: ");
//      Serial.println(ip);
   }
  } else {
//    Serial.println("WiFi shield disabled");
      oled.print("WiFi Disabled");
  }
 
  digitalWrite(SPI_WIFI_SS, HIGH);  
  digitalWrite(SPI_MAX_SS, LOW);
  
  max.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
  
  if (!plotData){Serial.println("MAX31865: 2 Wire mode initialised");}

  lastPID = millis();
  lastPWM = millis();
  setTemp = BREW_TEMP;

  pGain = readFloat(PGAIN_ADR);
  iGain = readFloat(IGAIN_ADR);
  dGain = readFloat(DGAIN_ADR);

  if (!plotData){
    Serial.println("Gains from EEPROM:");
    Serial.print(pGain);
    Serial.print(" ");
    Serial.print(iGain);
    Serial.print(" ");
    Serial.println(dGain);
  }

  if (pGain != pGain or !useSavedGain){
    pGain = P_GAIN;
    iGain = I_GAIN;
    dGain = D_GAIN;
    Serial.println("Using manual gains:");
    Serial.print(pGain);
    Serial.print(" ");
    Serial.print(iGain);
    Serial.print(" ");
    Serial.println(dGain);
  }
  delay(2000);
  oled.clear();
}

void loop() {
// OLED Pt100 resistors
  #define RREF                430.0
  #define RNOMINAL            100.0

  #define PRESS_CYCLE          1000

// Danfoss pressure range:
// 
// High:  650 psig   44.82 barg  90% reading = 0.9 x 1023 = 921
// Low:     0 psig    0.00 barg  10% reading = 0.1 x 1023 = 102
//
// Gradient  press_M = 44.82 / (921-102) =  0.05476
// Intercept press_C = -102 * .05476     = -5.586
// 
// Manual calibration suggests => C = -5.1

  unsigned int  deltaTime;
  float         tempRTD;  
  float         tempSum = 0;
  int           tempCount = 0;
  int           pressCount = 0;
  float         pressSum = 0;
  float         currPress;
  
  
  tempRTD = max.temperature(RNOMINAL, RREF);
  //  maxFaults();

  if (tempRTD > -50){
    tempSum += tempRTD;
    tempCount ++;
    currentTime = millis();
    currTemp = tempSum / tempCount;
  }

  pressSum += analogRead(PRESSURE_PIN) * 0.05476 -5.1;
  pressCount ++;
  currPress = pressSum / pressCount;

  dispTempPress(currTemp, currPress);

  if (writeDebug){
    Serial.print(tempCount);
    Serial.print(" ");
    Serial.print(tempRTD);
    Serial.print(" ");
    Serial.print(tempSum);
    Serial.print(" ");
    Serial.println(currTemp);
  }

  if (plotData){
    Serial.print(currentTime);
    Serial.print(" ");
    Serial.print(setTemp);
    Serial.print(" ");
    Serial.print(currTemp);    
    Serial.print(" ");
    Serial.print(currPress);    
    Serial.print(" ");
  }
  
  if (currentTime - lastPID >= PID_CYCLE){
    lastPID = currentTime;
    heatPower = updatePID();
    
    tempSum = 0;
    tempCount = 0;
  }

  if (currentTime - lastPress >= PRESS_CYCLE){
    lastPress = currentTime;
    pressSum = 0;
    pressCount = 0;
  }
  
  setHeatCycles(heatPower);
  updatePWM();
  
  if (plotData){
//    Serial.println(heatPower);    
    Serial.println();    
  }
}



double rand2()
{
    srand(millis());
    return (double)rand() / (double)RAND_MAX ;
}

float updatePID(){
  // these local variables can be factored out if memory is an issue, 
  // but they make it more readable
  float err;
  float windupGuard;
  float pTerm;
  float iTerm;
  float dTerm;

  // determine how badly we are doing
  err = setTemp - currTemp;

  // the pTerm is the view from now, the pgain judges 
  // how much we care about error we are this instant.
  pTerm = pGain * err;

  // iState keeps changing over time; it's 
  // overall "performance" over time, or accumulated error
  iState += err * PID_CYCLE / 1000.0;

  // to prevent the iTerm getting huge despite lots of 
  //  error, we use a "windup guard" 
  // (this happens when the machine is first turned on and
  // it cant help be cold despite its best efforts)

  // not necessary, but this makes windup guard values 
  // relative to the current iGain
  windupGuard = WINDUP_GUARD_GAIN / iGain;  

  if (iState > windupGuard) 
    iState = windupGuard;
  else if (iState < -windupGuard) 
    iState = -windupGuard;
  iTerm = iGain * iState;

  // the dTerm, the difference between the temperature now
  //  and our last reading, indicated the "speed," 
  // how quickly the temp is changing. (aka. Differential)
  dTerm = (dGain * (currTemp - lastTemp) * 1000 / PID_CYCLE);

  // now that we've use lastTemp, put the current temp in
  // our pocket until for the next round
  lastTemp = currTemp;
  
  if (debugPID){
    Serial.print("Set: ");
    Serial.print(setTemp);
    Serial.print("  Curr: ");
    Serial.print(currTemp);
    Serial.print("  Err: ");
    Serial.print(err);
    Serial.print("  P: ");
    Serial.print(pTerm);
    Serial.print("  I: ");
    Serial.print(iTerm);
    Serial.print("  D: ");
    Serial.println(dTerm);
  }
  // the magic feedback bit
  return  pTerm + iTerm - dTerm;
}

void updatePWM()
{
  unsigned int deltaPWM;
  
  currentTime = millis();
  deltaPWM = currentTime - lastPWM;
  
  if(currentTime - lastPWM >= PWM_CYCLE or lastPWM > currentTime) { //second statement prevents overflow errors
    // begin cycle
    _turnHeatElementOnOff(1);  // 
    lastPWM = currentTime;   

  if (!plotData and !debugPID){
    Serial.print(currTemp);
    Serial.print(" ");
    Serial.print(setTemp);
    Serial.print(" ");
    Serial.print(heatPower);
    Serial.print(" ");
    Serial.println(heatCycles);
    }
  } 
  
  if (currentTime - lastPWM >= heatCycles) {
    _turnHeatElementOnOff(0);
  }
}

void _turnHeatElementOnOff(boolean on) {
  digitalWrite(HEAT_RELAY, on);     //turn pin high
  heaterState = on;
}

void setHeatCycles(float power){
  if (power <= 0.0) {
    power = 0.0;
  }     
  if (power >= 100.0) {
    power = 100.0;
  }
  heatCycles = power /100 * PWM_CYCLE;  
}

void maxFaults(){
  // Check and print any faults
  uint8_t fault = max.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    max.clearFault();  
  }
}

float readFloat(int address) {
  float out;
  eeprom_read_block((void *) &out, (unsigned char *) address ,4 );
  return out;
}

void writeFloat(float value, int address) {
  eeprom_write_block((void *) &value, (unsigned char *) address ,4);
}

//void printVal(){
void printVal(float v, int d, int p){
//prints value v at x,y with d decimals and p before the point
  int l ;
  
  if (v <= 1){
    l = 1;
  } else {
    l = int(log10(v)) + 1;
  }
  
  for (int i = p; i > l; i--){
    oled.print(" ");
  }

  oled.print(v,d);
}

void dispTempPress(float t, float p){
// OLED layout

  #define TEMP_X1   0
  #define TEMP_X2  28
  #define PRESS_X1 73
  #define PRESS_X2 95
  #define LABEL_Y   0
  #define VAL_Y    10
  #define UNIT_Y   29
  #define TEMP_DEC                 1  // 1 decimal 
  #define TEMP_SP                  4  // 3 spaces before decimal
  #define PRESS_DEC                1  // 1 decimal 
  #define PRESS_SP                 2  // 3 spaces before decimal

  char deg = 160;

  oled.home();
    
  oled.set1X();
  oled.print(" Temp:");
  oled.set2X();
  printVal(t, 1, 3);
  oled.set1X();
  oled.setCursor(96,1);
  oled.print(deg);
  oled.println(" C");
  
  oled.println();
  oled.print(" Press: ");
  oled.set2X();
  printVal(p, 1, 2);
  oled.set1X();
  oled.setCursor(96,4);
  oled.println(" barg");

  oled.println();
  oled.print(" Flow:  ");
  oled.set2X();
  printVal(0, 1, 2);
  oled.set1X();
  oled.setCursor(96,7);
  oled.println(" ml");
  
}

