//3.1.1 switch to PID library
//3.1   SD card support
//3.0   new master

#define VERSION "3.1.1"


#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_MAX31865.h>
//#include <avr/EEPROM.h>
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <SD.h>
#include <TimeLib.h>

#define I2C_ADDRESS          0x3C
#define SPI_SLAVES              3

//Digital Pins
//#define SER_RX                0
//#define SER_TX                1
#define FLOW_PIN                2
// #define SPARE                3 <===
#define SPI_SD_SS               4
#define SPI_MAX_SS              5
#define HEAT_RELAY              6
//#define WIFI_HSK              7
//#define SPARE                 8 <===
//#define SPARE                 9 <===
#define SPI_WIFI_SS            10
//#define SPI_MOSI             11
//#define SPI_MISO             12
//#define SPI_SCK              13

//Analogue Pin
#define PRESSURE_PIN            0
//#define SPARE                 1 <===
//#define SPARE                 2 <===
//#define SPARE                 3 <===
//#define IC2_SDA               4 
//#define IC2_SCK               5

#define PID_CYCLE            1000
#define PWM_CYCLE            1000
#define TUNE_CYCLE            100
#define TUNE_SIG                5

//PID Gains
#define BREW_TEMP             95.0
#define WINDUP_GUARD            20
#define P_GAIN                  20
#define I_GAIN                  1
#define D_GAIN                  100
#define NTP_PACKET_SIZE         48

//#define PGAIN_ADR               0
//#define IGAIN_ADR               4
//#define DGAIN_ADR               8

//#define BREW_TEMP_ADDRESS    12

WiFiUDP Udp;

byte packetBuffer[ NTP_PACKET_SIZE];  //buffer to hold incoming and outgoing packets 

SSD1306AsciiWire oled;

Adafruit_MAX31865 max = Adafruit_MAX31865(SPI_MAX_SS);
//WiFiServer server(80);

// debugging flags

boolean writeDebug   = false;
boolean plotData     = true;
boolean ardPlot      = false;
boolean debugPID     = false;
boolean tunePID      = false;
boolean useSavedGain = false;
boolean useWifi      = true;
boolean useSD        = true;

// system setup
int spiSlaves[SPI_SLAVES];
int activeSlave;

unsigned int localPort = 2390;        // local port to listen for UDP packets


File logFile;
char data[] = "";
char fileName[] = "YYYYMMDD.CSV";

// PID variables
double currTemp;
double setTemp;
float lastTemp;
float lastPress;
int   pwmCount;
unsigned long nLoop;

float pGain;
float iGain;
float dGain;

float iState;
float heatPower = 0;
int heatCycles = 0;
boolean heaterState;

unsigned long currentTime;
unsigned long lastPID;
unsigned long lastPWM;
unsigned long lastPressTime;
unsigned long timeOffset;

long elapsedTime = 0;
unsigned long pulseCount;

void setup() {
  // initialize serial:
  Serial.begin(115200);

  spiSlaves[0] = SPI_WIFI_SS;
  spiSlaves[1] = SPI_MAX_SS;
  spiSlaves[2] = SPI_SD_SS;

  // set all SPI slave control lines to OUTPUT
  for (int i = 0; i < SPI_SLAVES; i++){
    pinMode(spiSlaves[i], OUTPUT);
  }
  
  // set actuators to output
  pinMode(HEAT_RELAY, OUTPUT);

  // set sensors to input
  pinMode(PRESSURE_PIN, INPUT);
  pinMode(FLOW_PIN, INPUT);
  
 // spiSlaveSelect(SPI_SD_SS);

  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&SH1106_128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  debugPrint("Gagguino ");
  debugPrintln(VERSION);

  _turnHeatElementOnOff(0); 

  if (useWifi){
      connectWiFi();
  } else {
      debugPrintln("WiFi Disabled");
  }

  if (useSD){
  spiSlaveSelect(SPI_SD_SS);
  debugPrint("SD Card init: ");

  if (!SD.begin(SPI_SD_SS)) {
    debugPrintln("FAIL");
    useSD = false;
  } else { 
    debugPrintln("OK");
  }

//  if (useSD){
//    setupLogFile();
//  }
}
  
 
  spiSlaveSelect(SPI_MAX_SS);  
  debugPrint("MAX31865 (2):");

  if (max.begin(MAX31865_2WIRE)){
    debugPrintln("OK");
  } else {
    debugPrintln("FAIL");
  }
  
  
  lastPID = millis();
  lastPWM = millis();
  setTemp = BREW_TEMP;

//  pGain = readFloat(PGAIN_ADR);
//  iGain = readFloat(IGAIN_ADR);
//  dGain = readFloat(DGAIN_ADR);

//  if (!ardPlot){
//    Serial.println("Gains from EEPROM:");
//    Serial.print(pGain);
//    Serial.print(" ");
//    Serial.print(iGain);
//    Serial.print(" ");
//    Serial.println(dGain);
//  }

  pGain = P_GAIN;
  iGain = I_GAIN;
  dGain = D_GAIN;
  if(!ardPlot){
    Serial.print(pGain);
    Serial.print(" ");
    Serial.print(iGain);
    Serial.print(" ");
    Serial.println(dGain);
  }

  delay(2000);
  oled.clear();

//  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowInt, CHANGE);
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

  float         currPress;

  currentTime=millis();
  
  if (currentTime - lastPID >= PID_CYCLE){
    spiSlaveSelect(SPI_MAX_SS);
    currTemp = max.temperature(RNOMINAL, RREF);
    if (currTemp < 0) {currTemp = lastTemp;}
    lastPID = currentTime;
    heatPower = updatePID();
    if (heatPower > 100.0){heatPower = 100.0;}
    if (heatPower < 0.0){heatPower = 0.0;}
    heatCycles = heatPower / 100 * PWM_CYCLE;
    
    dispTempPress(currTemp, lastPress, nLoop);
    
    if (useSD){
      spiSlaveSelect(SPI_SD_SS);
      writeSD(currentTime, setTemp, currTemp, currPress, 0);  
    }
    if (plotData){
      if(!ardPlot){
        Serial.print(currentTime);
        Serial.print(" ");
        Serial.print(nLoop);
        Serial.print(" ");
        Serial.print(currPress);
        Serial.print(" ");
        Serial.print(heatPower);
        Serial.print(" ");
//        Serial.print(newHeat);
//        Serial.print(" ");
      }
      Serial.print(setTemp);
      Serial.print(" ");
      Serial.println(currTemp);    
//      Serial.print(" ");
//      Serial.println(heatPower);
    }

    nLoop = 0;
  }

  if (currentTime - lastPressTime >= PRESS_CYCLE){
    currPress = analogRead(PRESSURE_PIN) * 0.05476 -5.1;
    if (currPress < 0){currPress = lastPress;}
    lastPressTime = currentTime;

    dispTempPress(lastTemp, currPress, 0);

    lastPress = currPress;
  }

  if (currentTime - lastPWM >= heatCycles) {
    _turnHeatElementOnOff(0);
  }

  if(currentTime - lastPWM >= PWM_CYCLE) { //second statement prevents overflow errors
    // begin cycle
    _turnHeatElementOnOff(1);  // 
    lastPWM = currentTime;   

    if (tunePID){
      pwmCount++;
      if(pwmCount == TUNE_CYCLE){
        setTemp = (2 * BREW_TEMP - TUNE_SIG) - setTemp;
        pwmCount = 0;
      }
    }
  } 
  nLoop ++;

  if (timeStatus() != timeNotSet) {
//    if (now() != prevDisplay) { //update the display only if time has changed
//      prevDisplay = now();
//      digitalClockDisplay();  
//    }
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
  
  if (iState > WINDUP_GUARD / iGain) 
    iState = WINDUP_GUARD / iGain;
  else if (iState < -WINDUP_GUARD / iGain) 
    iState = -WINDUP_GUARD / iGain;
  iTerm = iGain * iState;

  // the dTerm, the difference between the temperature now
  //  and our last reading, indicated the "speed," 
  // how quickly the temp is changing. (aka. Differential)
  dTerm = (dGain * (currTemp - lastTemp) * 1000 / PID_CYCLE);

  // now that we've use lastTemp, put the current temp in
  // our pocket until for the next round
  lastTemp = currTemp;
  
  if (debugPID){
    Serial.print("  ");
    Serial.print(pTerm);
    Serial.print("  ");
    Serial.print(iTerm);
    Serial.print("  ");
    Serial.print(dTerm);
    Serial.print("  ");
    Serial.println(pTerm + iTerm - dTerm);
  }
  // the magic feedback bit
  return  pTerm + iTerm - dTerm;
}


void _turnHeatElementOnOff(boolean on) {
  digitalWrite(HEAT_RELAY, on);     //turn pin high
  heaterState = on;
}

float setHeatCycles(float power){
  if (power <= 0.0) {
    power = 0.0;
  }     
  if (power >= 100.0) {
    power = 100.0;
  }
  return (power /100 * PWM_CYCLE);  
}

void maxFaults(){
  // Check and print any faults
  uint8_t fault = max.readFault();
  if (fault and !plotData) {
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

//* ***************************************************************************************
//*                                EEPROM functions - OK                                  *
//* ***************************************************************************************


//float readFloat(int address) {
//  float out;
//  eeprom_read_block((void *) &out, (unsigned char *) address ,4 );
//  return out;
//}

//void writeFloat(float value, int address) {
//  eeprom_write_block((void *) &value, (unsigned char *) address ,4);
//}

//* ***************************************************************************************
//*                                OLED functions - OK                                    *
//* ***************************************************************************************


void printValOled(float v, int d, int p){
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

void printValLogFile(float v, int d, int p){
  int l ;

  if (v <= 1){
    l = 1;
  } else {
    l = int(log10(v)) + 1;
  }
  
  for (int i = p; i > l; i--){
    logFile.print(" ");
  }

  logFile.print(v,d);
}

void dispTempPress(float t, float p, int n){
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
  printValOled(t, 1, 3);
  oled.set1X();
  oled.setCursor(96,1);
  oled.print(deg);
  oled.println(" C");
  
  oled.println();
  oled.print(" Press: ");
  oled.set2X();
  printValOled(p, 1, 2);
  oled.set1X();
  oled.setCursor(96,4);
  oled.println(" barg");

  oled.println();
  oled.print(" Flow:  ");
  oled.set2X();
  printValOled(0, 1, 2);
  oled.set1X();
  oled.setCursor(96,7);
  oled.println(" ml");
  if (n > 0){
    oled.setCursor(0,7);
    oled.println(n);
  }
  
}

void spiSlaveSelect(int ss){
  if (activeSlave != ss){
    for (int i = 0; i < SPI_SLAVES; i++){
      digitalWrite(spiSlaves[i],!(ss == i));
    }
    activeSlave = ss;
  }
}

//* ***************************************************************************************
//*                                SD Card functions - NOT TESTED                         *
//* ***************************************************************************************



void setupLogFile(){
    oled.print("Log init: ");
    if(!ardPlot){
      Serial.print("Log init file ");
      Serial.print(fileName);
      Serial.print(": ");
    }

    if (SD.exists(fileName)){
      SD.remove(fileName);
      Serial.println("removed");
    }
    
    logFile = SD.open(fileName, FILE_WRITE);
    delay(100);
  
    if(logFile){
    // send a standard http response header
      logFile.println("HTTP/1.1 200 OK");
//      logFile.println("Content-Type: text/html");
//      logFile.println("Connnection: close");
//      logFile.println();
//      logFile.println("<!DOCTYPE HTML>");
      logFile.println("<html>");
      // add a meta refresh tag, so the browser pulls again every 5 seconds:
//      logFile.println("<meta http-equiv=\"refresh\" content=\"5\">");
      logFile.close();
      oled.println("OK");
      if(!ardPlot){Serial.println("OK");}
    } else {
      if(!ardPlot){Serial.println("FAIL");}
      oled.println("FAIL");
    }  
}

void writeSD(long xTime, float sTemp, float xTemp, float xPress, float xVol){
  logFile = SD.open(fileName, FILE_WRITE);

  if(logFile){
    printValLogFile(xTime, 1, 10);
    printValLogFile(sTemp, 1, 5);
    printValLogFile(xTemp, 1, 5);
    printValLogFile(xPress, 1, 5);    
    printValLogFile(xVol, 2, 4);    
    logFile.println();
  }

  logFile.close();
}

//* ***************************************************************************************
//*                                HTTP Server function - NOT TESTED                      *
//* ***************************************************************************************



//void serveLog()  // listen for incoming clients
//{ 
//  int lastSlave = activeSlave;
  
//  spiSlaveSelect(SPI_WIFI_SS);
//  WiFiClient client = server.available();
  
//  if (client) {
//    if (!plotData){Serial.println("new client");}
    // an http request ends with a blank line
//    boolean currentLineIsBlank = true;
//    while (client.connected()) {
//      if (client.available()) {
//        char c = client.read();
//        if(!ardPlot){Serial.write(c);}
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
//        if (c == '\n' && currentLineIsBlank) {
//          spiSlaveSelect(SPI_SD_SS);
//          logFile = SD.open(fileName);
//          if (logFile) {
//            if(!ardPlot){
//              Serial.print(fileName);
//              Serial.println(":");
//            }

            // read from the file until there's nothing else in it:
//            while (logFile.available()) {

//              char data = logFile.read();
//              spiSlaveSelect(SPI_WIFI_SS);
//              client.print(data);
//              if(!ardPlot){Serial.write(data);}
//            }
            // close the file:
//            spiSlaveSelect(SPI_SD_SS);
//            logFile.close();
//          } 
//          else {
            // if the file didn't open, print an error:
//            if(!ardPlot){Serial.println("error opening test.txt");}
//          }
//          spiSlaveSelect(SPI_WIFI_SS);
//          if(!ardPlot){Serial.println(" done");}
//          client.println(" done"); 
//          client.println("<br />");       
//          client.println("</html>");
//          break;
//        }
//        if (c == '\n') {
          // you're starting a new line
//          currentLineIsBlank = true;
//        } 
//        else if (c != '\r') {
          // you've gotten a character on the current line
//          currentLineIsBlank = false;
//        }
//      }
//    }
    // give the web browser time to receive the data
//    delay(1);
    // close the connection:
//    client.stop();
//    if(!ardPlot){Serial.println("client disonnected");}
//  }
//}

//* ***************************************************************************************
//*                                Time functions                                         *
//* ***************************************************************************************

unsigned long getNtpTime(){
  IPAddress timeServerAddress(130,88,200,4); // ntp2c.mcc.ac.uk NTP server
  const int NTP_TIMEOUT = 10000;
  unsigned long epoch;
  
  Serial.print("Getting time from ");
  Serial.println(timeServerAddress);
  sendNTPpacket(timeServerAddress); // send an NTP packet to a time server
    // wait to see if a reply is available
    unsigned long startTime = millis();
  while (millis() - startTime < NTP_TIMEOUT and !Udp.parsePacket()){  
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("NTP Receive: ");
  if ( Udp.parsePacket() ) { 
    Serial.println("OK"); 
    // We've received a packet, read the data from it
    Udp.read(packetBuffer,NTP_PACKET_SIZE);  // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);  
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;  
 //   Serial.print("Seconds since Jan 1 1900 = " );
 //   Serial.println(secsSince1900);               

    // now convert NTP time into everyday time:
 //   Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;     
    // subtract seventy years:
    epoch = secsSince1900 - seventyYears;  
  } else {
    Serial.println("FAIL");
  }
  return epoch;
}

// send an NTP request to the time server at the given address 
unsigned long sendNTPpacket(IPAddress& address)
{
  //Serial.println("1");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE); 
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  //Serial.println("2");
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49; 
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  
  //Serial.println("3");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:        
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  //Serial.println("4");
  Udp.write(packetBuffer,NTP_PACKET_SIZE);
  //Serial.println("5");
  Udp.endPacket(); 
  //Serial.println("6");
}

//* ***************************************************************************************
//*                                Interrupts                                             *
//* ***************************************************************************************

void flowInt(){
  pulseCount++;
}

void connectWiFi(){
  char ssid[] = "Dlink12";      //  your network SSID (name) 
  char pass[] = "4d9a4d4652";   // your network password
  int status = WL_IDLE_STATUS;  // the Wifi radio's status

  spiSlaveSelect(SPI_WIFI_SS);
  oled.println(F("Connecting to Wifi"));
  oled.print(F("Network: "));
  oled.println(ssid);

  status = WiFi.begin(ssid, pass);

  if ( status != WL_CONNECTED) { 
    debugPrintln("FAIL");
    useWifi = false;
  } else {
    debugPrintln("CONNECTED as ");
    IPAddress ip = WiFi.localIP();
    oled.println(ip);
    Serial.println(ip);

    Serial.print(F("Starting UDP: "));
    if (Udp.begin(localPort)){
      Serial.println("OK");
      timeOffset = getNtpTime();
      long y = year(timeOffset);
      long m = month(timeOffset);
      long d = day(timeOffset);
      int h = hour(timeOffset);
      int n = minute(timeOffset);
      int s = second(timeOffset);
      Serial.print(F("Date: "));
      Serial.print(d);
      Serial.print("/");
      Serial.print(m);
      Serial.print("/");
      Serial.println(y);
      Serial.print("Time: ");
      Serial.print(h);
      Serial.print(":");
      Serial.print(n);
      Serial.print(":");
      Serial.println(s);
      setTime(timeOffset);
      long formattedTime = y*10000 + m * 100 + d;
      ltoa(formattedTime ,fileName ,10);
      Serial.println(formattedTime);
      Serial.println(fileName);
//      char newFile[] = F("yyyymmdd.txt");      for (int i=1; i<
      fileName[8] = '.';
      fileName[9] = 't';
      fileName[10] = 'x';
      fileName[11] = 't';
      fileName[12] = 0;
      Serial.println(fileName);
    } else {
      Serial.println(F("FAIL"));
    }       
  }
//    server.begin();   
}

void debugPrintln(char c[]){
    oled.println(c);
    if(!ardPlot){Serial.println(c);}  
}

void debugPrint(char c[]){
    oled.print(c);
    if(!ardPlot){Serial.print(c);}  
}

