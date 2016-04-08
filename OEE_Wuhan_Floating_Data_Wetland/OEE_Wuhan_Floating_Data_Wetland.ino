/*****************************************************************************************************************
 * Retrive and post information about water quality using Atlas Scientific pH and dissolved oxygen sensors 
 * and post them to an IOT cloud server using an Adafruit FONA. This is or the floating data wetlands project 
 * portion of OEE (Office of Environmental Experiments) http://oee.ccastellanos.com
 * 
 * Much of the FONA code is borrowed and modified from an instructable by Kina Smith: http://is.gd/p1eNfb
 * 
 * Much of the sensor code is modified from example code provided by Atlas Scientific: http://atlas-scientific.com
 * 
 * created March 28, 2016
 * by Carlos Castellanos
*****************************************************************************************************************/

#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#include <Adafruit_DotStar.h>
#include <LowPower.h>
#include <avr/pgmspace.h>

// default baud rate for all serial communication
#define BAUD_RATE 9600

/*******************
ADAFRUIT DOTSTAR LED
*******************/
#define NUMPIXELS 15        // Number of LEDs we are driving

// variables related to the blinking/fading of the LEDs
uint16_t brightness =  0;   // how bright the LED is
uint16_t fadeamount =  5;   // how many points to fade the LED by
#define BRIGHTNESS 0
#define FADEAMOUNT 5
unsigned long currentTime;  // this is for timeing of the fading/blinking of the LED's
unsigned long loopTime;     // so is this

// create a list of colors for the lights
// we are basically creating an HSV gradient here
#define NUMCOLORS = 25;
// list of colors from red to magenta to blue; the higher the index of the array the higher the water quality
// NOTE: The PROGMEM keyword is a variable modifier, it should be used only with the datatypes defined in pgmspace.h.
// It tells the compiler "put this information into flash memory", instead of into SRAM, where it would normally go.
// more info: http://www.arduino.cc/en/Reference/PROGMEM
const uint32_t PROGMEM colorList[NUMCOLORS] = 
{0xFF0000,0xFF0015,0xFF002A,0xFF003F,0xFF0055,0xFF006A,0xFF007F,0xFF0094,0xFF00AA,0xFF00BF,0xFF00D4,0xFF00E9,0xFF00FF,
0xE900FF,0xD400FF,0xBF00FF,0xAA00FF,0x9400FF,0x7F00FF,0x6A00FF,0x5400FF,0x3F00FF,0x2A00FF,0x1500FF,0x0000FF};

// Here's how to control the LEDs from any two pins:
// The below code is for software SPI on pins 8 & 9
//#define DATAPIN    8
//#define CLOCKPIN   9
/*Adafruit_DotStar strip = Adafruit_DotStar(
  NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);*/
// The last parameter is optional -- this is the color data order of the
// DotStar strip, which has changed over time in different production runs.
// Your code just uses R,G,B colors, the library then reassigns as needed.
// Default is DOTSTAR_BRG, so change this if you have an earlier strip.

// Hardware SPI is a little faster, but must be wired to specific pins
// (Arduino Uno & Pro/Pro Mini = pin 11 for data, 13 for clock, other boards are different).
// And that's what we're doing!
Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_BRG);

/************
ADAFRUIT FONA
************/
#define FONA_RX 2  // FONA serial rx pin (from FONA tx)
#define FONA_TX 3  // FONA serial tx pin (to FONA rx)
#define FONA_RST 4 // FONA RESET pin
#define FONA_KEY 5 // FONA KEY pin
#define FONA_PS 6  // FONA Power Status pin

String APN = "__PUT YOUR APN HERE!!__"; //Set APN for Mobile Service

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX); // initialize software serial
String fonaResponse;                                      // global variable for pulling AT command responses from inside functions
const uint32_t fonaKeyTime = 2000;                        // Time needed to turn on the FONA
const uint32_t fonaATtimeOut = 10000;                     // How long we will give an AT command to complete

// ====== IOT server (Thingspeak) URL Building ====== //
const String writeKey = "8G6ITUXGW2H7LYRX";                                                    // Write API Key for writing data to Thingspeak OEE Wuhan channel
//const String readKey = "FRMG4PS99P4TVTW7";                                                   // Read API Key for reading data from Thingspeak OEE Wuhan channel
//const String thingSpeakChannelID = "106708";                                                 // Thingspeak Channel ID
const String IOT_URL = "https://api.thingspeak.com/update/";                                   // Thingspeak update url (can also use Thingspeak IP: 184.106.153.149")
//const uint16_t NUM_FIELDS = 8;                                                               // number of fields in data stream
//const String fieldNames[NUM_FIELDS] = {"lat","long","ph","do","sat","temp","wqi","quality"}; // actual data fields
//String fieldData[NUM_FIELDS];                                                                // holder for the data values
String Lat;                                                                                    // variables to hold location data (lat & long)
String Long;
//String Date;                                                                                 // variables to hold date & time
//String Time;
const String defaultLat = "30.646";
const String defaultLong = "114.239";

// ====== WeChat Bot URL Building ====== //




/***********************************************
ATLAS SCIENTIFIC WATER QUALITY SENSORS (PH & DO)
***********************************************/
#define DO_RX 7  // Serial rx pin for DO sensor (from DO tx)
#define DO_TX 8  // Serial tx pin for DO sensor (to DO rx)
#define PH_RX 9  // Serial rx pin for PH sensor (from PH tx)
#define PH_TX 10 // Serial tx pin for PH sensor (to PH rx)

SoftwareSerial doSS = SoftwareSerial(DO_RX, DO_TX); //initialize software serial for DO sensor
SoftwareSerial phSS = SoftwareSerial(PH_RX, PH_TX); //initialize software serial for PH sensor

byte startup=0;                       //This is used to make sure the Arduino takes over control of the sensor circuits properly.
byte arduino_only=0;                  //If you would like to operate the pH Circuit with the Arduino only and not use the serial monitor to send it commands set this to 1.
                                      //The data will still come out on the serial monitor, so you can see it working.  

// === Dissolved Oxygen === //
char DO_data[20];                     //we make a 20 byte character array to hold incoming data from the D.O. 
char computerdata_do[20];             //we make a 20 byte character array to hold incoming data from a pc/mac/other. 
byte received_from_computer_do=0;     //we need to know how many characters have been received.                                 
byte received_from_do_sensor=0;       //we need to know how many characters have been received. 
byte string_received_do=0;            //used to identify when we have received a string from the D.O. circuit.
byte string_converted_do=0;           //used to identify when the string from the d.o. circuit has been converted (into a float).
float DO_float=0;                     //used to hold a floating point number that is the D.O. 
float sat_float=0;                    //used to hold a floating point number that is the percent saturation.
char *DO;                             //char pointer used in string parsing 
char *sat;                            //char pointer used in string parsing

// === pH === //
char ph_data[20];                     //we make a 20 byte character array to hold incoming data from the pH. 
char computerdata_ph[20];             //we make a 20 byte character array to hold incoming data from a pc/mac/other. 
byte received_from_computer_ph=0;     //we need to know how many characters have been received.                                 
byte received_from_ph_sensor=0;       //we need to know how many characters have been received.
byte string_received_ph=0;            //used to identify when we have received a string from the pH circuit.
byte string_converted_ph=0;           //used to identify when the string from the pH circuit has been converted (into a float).
float ph=0;                           //used to hold a floating point number that is the pH. 

// === Water Temperature === //
// grab temperature from analog temperature sensor (TMP36 or LM35)
// this is important for calibrating the DO & pH sensors
const int tempSensor = A0;  // analog sensor pin
char temp_data[8];          // 8 byte character array to hold the temperature data 
float waterTemp = 20;       // used to hold a floating point number for the temperature (default is 20C)

// === Water Quality Index === //
// This is arrived at by taking the d.o. saturation % and the pH levels and essentially averaging them
// then mapping them to a value between 0-100.
// According to the National Sanitation Foundation (in the U.S.A.), water quality ratings are as follows:
// 91-100: excellent, 71-90: good, 51-70: medium, 26-50: bad, 0-25: very bad
// we will use these ratings
uint8_t WQI;           // water quality index
String waterQuality;   // String of the water quality rating

/************************
INTERRUPT/POWER SAVE CODE
************************/
volatile byte blinkFlag = 0;  // this variable is updated in the ISR
byte asleep = 0;              // variable that holds whether we are asleep or not
#define SLEEP_SECS = 900;     // Sleep time (900 secs = 15 min)

void setup() {
  Serial.begin(BAUD_RATE);        // enable the hardware serial port
  
  pinMode(FONA_RESET, OUTPUT);    // set-up this pin as an output so we can hard reset the FONA if necessary
  pinMode(FONA_PS, INPUT);        // set-up this pin as an input so we can get the power status of the FONA
  pinMode(FONA_KEY, OUTPUT);      // set-up this pin as an output so we can turn the FONA on/off
  fonaSS.begin(BAUD_RATE);        // enable the FONA software serial port
  turnOnFONA();                   // turn on the FONA

  doSS.begin(BAUD_RATE);          // enable the DO sensor software serial port
  phSS.begin(BAUD_RATE);          // enable the pH sensor software serial port

  strip.begin();                  // Initialize LED pins for output
  strip.clear();                  // Set all pixel data to zero
  strip.show();                   // Turn all LEDs off ASAP
  
  // this is for fading/blinking the LEDs
  currentTime = millis();
  loopTime = currentTime; 

  // Enable the interrupt on the FONA rx pin
  // so that serial activity will wake the device.
  // The internal pull-up is turned on
  // so the pin is in a well defined state (HIGH),
  // even if a serial port is not attached.
  pinMode(FONA_RX, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FONA_RX), blinkISR, CHANGE); // attach the interrupt
}

void loop() {
  // setup a gprs context on the FONA
  setupGPRS();
  // take the d.o. & ph sensor readings
  // d.o.
  // if we see that the d.o. Circuit has sent a character.
  if(doSS.available() > 0) {
    // We read the data sent from do Circuit until we see a <CR>.
    // We also count how many characters have been received
    // (readBytesUntil() returns the number of characters read into the buffer. A 0 means no valid data was found.)
    received_from_do_sensor = doSS.readBytesUntil(13,DO_data,20);
    // We add a 0 to the spot in the array just after the last character we received.
    // This will stop us from transmitting incorrect data that may have been left in the buffer.
    DO_data[received_from_do_sensor] = 0;
    // a flag used when the Arduino is controlling the d.o. Circuit to let us know that a complete string has been received.
    string_received_do = 1;
    // let's transmit the data received from the D.O. to the serial monitor.
    Serial.println(DO_data);
  }
  // a flag used when the Arduino is controlling the d.o. Circuit to let us know that the string has been converted (to a float).
  string_converted_do = 0;

  // pH
  // if we see that the pH Circuit has sent a character.
  if(phSS.available() > 0) {
    // We read the data sent from pH Circuit until we see a <CR>.
    // We also count how many characters have been received
    // (readBytesUntil() returns the number of characters read into the buffer. A 0 means no valid data was found.)
    received_from_ph_sensor = phSS.readBytesUntil(13,ph_data,20);
    // We add a 0 to the spot in the array just after the last character we received.
    // This will stop us from transmitting incorrect data that may have been left in the buffer.
    ph_data[received_from_ph_sensor] = 0;
    // a flag used when the Arduino is controlling the pH Circuit to let us know that a complete string has been received.
    string_received_ph = 1;
    // lets transmit that data received from the pH Circuit to the serial monitor.
    Serial.println(ph_data);
  }
   // a flag used when the Arduino is controlling the pH Circuit to let us know that the string has been converted (to a float).
   string_converted_ph = 0;
   
  // If the variable arduino_only is set to 1 we will call this function,
  // which lets the Arduino take over control of the d.o. & pH circuits
  // this is the function that actually asks the sensors to take a reading
  if(arduino_only==1) {
    ArduinoSensorControl();
  }

  // analyze sensor readings and determine a water quality index (0-100)
  // average of the WQI of the pH & d.o.
  WQI = round((calculateWQIdo() + calculateWQIph()) / 2);
  // get the water quality rating
  if(WQI >= 0 && WQI <= 25) {
    waterQuality = "very bad";
  } else if(WQI >= 26 && WQI <= 50) {
    waterQuality = "bad";
  } else if(WQI >= 51 && WQI <= 70) {
    waterQuality = "medium";
  } else if(WQI >= 71 && WQI <= 90) {
    waterQuality = "good";
  } else if(WQI >=91) {
    waterQuality = "excellent";
  }

  // update the LED colors based upon the most recent sensor readings & resulting water quality index
  updateLEDColor()

  // update the IOT server with the most recent sensor data
  // first get the location from the FONA
  if(!getLocation()) { // set the location to the defaults if there was an error
    Lat = defaultLat;
    Long = defaultLong;
  }
  // now try and send the data
  makeHTTPRequest();
  // flush the FONA serial port
  flushFONA();
  
  // if the blinkFlag = 1 (we received a message from WeChat), blink the lights
  if(blinkFlag) {
    // blink the lights, send message about water quality
    blinkLEDs();
    // set blinkFlag back to 0
    blinkFlag = 0;
  }
  
  // put the Arduino & FONA to sleep once the sensor readings have been taken and processed
  if (string_converted_do==1 && string_converted_ph==1) {
    gotoSleep();
  }
  // run the wake up function if we are just getting out of sleep (i.e. gotoSleep() was just called)
  if(asleep) wakeUp();
}

// === Interrupt Service Routine == //
void blinkISR() {
    blinkFlag = 1;
}

// === Sleep/Wake Power-saving functions === //
void gotoSleep() {
  // put Arduino, sensors & FONA to sleep
  
  // first clear the serial read & write buffers for the d.o. & ph sensors
  doSS.read();
  doSS.flush();
  phSS.read();
  phSS.flush();

  // set the sleep flag to 1
  asleep = 1;
  
  // put the d.o & pH sensors to sleep
  doSS.print("SLEEP\r");
  phSS.print("SLEEP\r");

  // put the FONA to sleep


  // put Arduino to sleep
  for(uint16_t i = 0; i < (SLEEP_SECS / 4); i++) {
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
  }
}

void wakeUp() {
  // wake sensors & FONA (Arduino is already awake)

  // FONA

  // D.O. & pH sensors
  // After the d.o. & pH sensors wake up, several consecutive readings (16 for d.o., 4 for pH) should be taken before the readings are considered valid.
  // so let's take those readings now
  for(byte i = 0; i < 16; i++) {
    doSS.print("R\r");       // send the d.o. sensor the command to take a single reading.
    phSS.print("R\r");       // send the pH sensor the command to take a single reading.
    delay(1);                // short delay to give the sensors time to take the readings
  }
  // clear the read buffers
  doSS.read();
  phSS.read();

  // set the sleep flag back to 0
  asleep = 0;
  
  // set startup flag to 0 so the d.o. & pH sensors go through the startup process again when they wake up
  startup = 0;
}

// === Dotstar LED functions == //
void blinkLEDs() {
  // this will really be more of a fast pulse/fade
  // we will quickly fade up and down the LEDs 3 times

  byte flag = 0;

  while(flag < 6) {
    currentTime = millis();
    if(currentTime >= (loopTime + 4)) {             // we change the brightness every 4 ms
      strip.setBrightness(brightness);              // set the brightness of the LEDs
      brightness = brightness + fadeAmount;         // change the brightness for next time through the loop
      if(brightness == 0 || brightness == 255) {    // reverse the direction of the fading at the ends of the fade
        fadeAmount = -fadeAmount;
        flag++;                                     // increment the flag (after 3 fade in/outs we exit the loop)
      }
      loopTime = currentTime;                       // update loopTime
    }
  }
  
  strip.setBrightness(255);                         // set strip back to full brightness before we exit the function
  brightness = BRIGHTNESS;                          // also set the brightness variable back to its original setting
  fadeamount = FADEAMOUNT;                          // and the fade amount as well
}

void updateLEDColor() {
  // make sure we are at full brightness
  strip.setBrightness(255);

  // determine the index of the color array to access from the water quality index
  uint8_t index = round(WQI / (100 / NUMCOLORS)) - 1;

  // Remember that colors are stored in flash memory
  // so they need to be read using the pgmspace.h functions.
  uint32_t color = pgm_read_dword_near(&colorList[index]);
  // set the color for all the pixels/leds
  for (uint8_t i = 0; i < NUMPIXELS; ++i) {
      strip.setPixelColor(i, color);
  }
  // show the updated pixels
  strip.show();
}

// === Water Quality Sensor Functions === //
void parseDOdata() {
  // The d.o. reading sensor actually returns two readings by default:
  // the d.o. reading itself (in mg/L) and the saturation percentage
  // we need to parse this data
  byte parseFlag = 0;
  // look for a comma in the data stream
  for(byte i=0;i<=received_from_do_sensor;i++) {
    if(DO_data[i]==',') {
      parseFlag = 1;
    }
  }
  
  // if you found the comma, then do some parsing!
  // use the strtok() function from the C Standard library (which is part of avr-libc)
  // to break the string into a series of tokens using "," as the delimiter.
  if(parseFlag) {
    DO = strtok(DO_data, ",");         // parse the string at the comma to get the DO data.
    sat = strtok(NULL, ",");           // parse the string at the comma to get the saturation data.

    Serial.print("DO:");               // We now print each value we parsed seperatly. 
    Serial.println(DO);                // send the d.o. value to the serial monitor
    DO_float=atof(DO);                 // turn the string into to float value.

    Serial.print("%sat:");             // We now print each value we parsed seperatly.  
    Serial.println(sat);               // send the %sat value to the serial monitor 
    sat_float=atof(sat);               // turn the string into to float value.

    string_converted_do = 1;           // set the string converted flag to 1 (true).
  }
}

void ArduinoSensorControl() {
      if(startup == 0) {                    // if the Arduino just booted up, we need to set some things up first. 
          doSS.print("C,0\r");              // take the d.o. Circuit out of continuous mode.   
          phSS.print("C,0\r");              // take the pH Circuit out of continuous mode. 
          delay(50);                        // on start up sometimes the first command is missed. 
          doSS.print("C,0\r");              // so, let’s send it twice.
          phSS.print("C,0\r");
          delay(50);                        // a short delay after the circuits were taken out of continuous mode is used to make sure we don't overload them with commands.

          doSS.print("RESPONSE,0\r");       // disable response code so we don't get the "*OK" message when working in arduino_only mode
          phSS.print("RESPONSE,0\r");

          delay(10);                        // another short delay

          // now lets get the temperature of the water (in Celsius)
          // this makes sure the d.o. & ph sensors stay properly calibrated
          uint16_t sensorVal = analogRead(tempSensor);
          // Convert the analog reading (which goes from 0 - 1023) to a voltage in millovolts (0 - 5000 mV):
          float tempMV = sensorVal * (5000 / 1024.0);
          // now use this formula to convert the voltage to a temperature:
          // Temp in °C = [(Vout in mV) - 500] / 10
          waterTemp = (tempMV - 500) / 10;
          // convert waterTemp val to a char array (string), then send the temperature to the d.o. & pH sensors
          dtostrf(waterTemp, 4, 2, temp_data);   // this is the standard avr-libc function for converting floats to Strings - http://is.gd/KausLc
          doSS.print("T,"+temp_data+"\r");       // send the temperature to the d.o. sensor
          phSS.print("T,"+temp_data+"\r");       // send the temperature to the ph sensor
          
          startup = 1;                           // startup is completed, let's not do this again during normal operations. 
      }

  doSS.print("R\r");                       //send the d.o. sensor the command to take a single reading.
  if(string_received_do == 1) {            //did we get data back from the do Circuit?
    // if DO_data[0] is a digit and not a letter (these are ascii values)
    if((DO_data[0] >= 48) && (DO_data[0] <=57)) {
      // we need to parse the do data
      parseDOdata();                       //do some parsing!
      string_received_do = 0;              //reset the string received flag.
    }

  }
  
  phSS.print("R\r");                       //send the ph sensor the command to take a single reading.
  if(string_received_ph == 1) {            //did we get data back from the ph Circuit?
    ph = atof(ph_data);                    //convert string into a float
    string_converted_ph = 1;               //set the string converted flag to 1 (true).
    if(ph>=7.5){Serial.println("high\r");} //This is the proof that it has been converted into a float.
    if(ph<7.5){Serial.println("low\r");}   //This is the proof that it has been converted into a float.
    string_received_ph = 0;                //reset the string received flag.
  }
  // small delay to make sure we get the data back from the sensors without having to loop
  // this way we only have to do this once before going to sleep
  // remember delays do not disable interrupts and serial communication should still work
  delay(2);
}

// water quality calculations for D.O.
uint8_t calculateWQIdo() {
  // returns the water quality index (WQI) for d.o.
  // note: the "in" arrays should have increasing values

  // convert the saturation % value into an int
  uint8_t satPercentage = round(sat_float);
  // doSatIn[] holds the measured saturation % values from a d.o. sensor
  uint8_t doSatIn[]  = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140};
  // wqOut[] holds the corresponding WQI values
  uint8_t wqOut[] = {2,7,12,19,30,44,57,75,87,95,99,96,90,84,78};

  return (uint8_t)multiMap(satPercentage, doSatIn, wqOut, 15);
}

// water quality calculations for pH
uint8_t calculateWQIph() {
  // returns the water quality index (WQI) for pH
  // note: the "in" arrays should have increasing values
  
  // phIn[] holds the measured saturation % values from a d.o. sensor
  float phIn[]  = {1,2,3,4,5,6,7,8,9,10,11,12,13};
  // doWQout[] holds the corresponding WQI values
  float wqOut[] = {0,2,4,9,27,55,88,84,49,20,8,3,0};

  return (uint8_t)round(FmultiMap(ph, phIn, wqOut, 13));
}

// === Non-linear mapping functions === //
int multiMap(int val, int* _in, int* _out, uint8_t size) {
  // this function allows for remapping of non-linear conversions/distributions of sensor readings.
  // this will help us calcluate the water quality index from the pH & d.o levels (which map non-linearly to water quality)
  // taken from here: http://playground.arduino.cc/Main/MultiMap
  // & based upon the reMap function by Martin Nawrath / KHM 2010
  // http://interface.khm.de/index.php/lab/interfaces-advanced/nonlinear-mapping/
  
  // note: the _in array should have increasing values
  
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}

float FmultiMap(float val, float * _in, float * _out, uint8_t size) {
  // this function allows for remapping of non-linear conversions/distributions of sensor readings.
  // this will help us calcluate the water quality index from the pH & d.o levels (which map non-linearly to water quality)
  // taken from here: http://playground.arduino.cc/Main/MultiMap
  // & based upon the reMap function by Martin Nawrath / KHM 2010
  // http://interface.khm.de/index.php/lab/interfaces-advanced/nonlinear-mapping/
  // this is the float version

  // note: the _in array should have increasing values
  
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}

// === FONA GSM/GPRS functions === //
boolean getLocation() {
  String content = "";
  char character;
  byte complete = 0;
  char c;
  unsigned long commandClock = millis();                              // Start the timeout clock
  fonaSS.println("AT+CIPGSMLOC=1,1");
  while(!complete && commandClock <= millis()+ATtimeOut) {            // Need to give the modem time to complete command
    while(!fonaSS.available() && commandClock <= millis()+ATtimeOut); // wait while there is no data
    while(fonaSS.available()) {                                       // if there is data to read...
      c = fonaSS.read();
      if(c == 0x0A || c == 0x0D) {
      } else {
        content.concat(c);
      }
    }
    if(content.startsWith("+CIPGSMLOC: 0,")) {
      Serial.println("Got Location");
      Long = content.substring(14, 24);
      Lat = content.substring(25, 34);
      //Date = content.substring(35, 45);
      //Time = content.substring(46,54);
      return 1;
    } else {
      Serial.print("ERROR: ");
      Serial.println(content);
      return 0;
    }
    complete = 1; //this doesn't work.
  }
}

void setupGPRS() { //all the commands to setup a GPRS context and get ready for HTTP command
    //the sendATCommand sends the command to the FONA and waits until the recieves a response before continueing on. 
    Serial.print("Disable echo: ");
    if(sendATCommand("ATE0")) { //disable local echo
        Serial.println(fonaResponse);
    }
    Serial.print("Set to TEXT Mode: ");
    if(sendATCommand("AT+CMGF=1")){ //sets SMS mode to TEXT mode....This MIGHT not be needed. But it doesn't break anything with it there. 
        Serial.println(fonaResponse);
    }
    Serial.print("Attach GPRS: ");
    if(sendATCommand("AT+CGATT=1")){ //Attach to GPRS service (1 - attach, 0 - disengage)
        Serial.println(fonaResponse);
    }
    Serial.print("Set Connection Type To GPRS: "); //AT+SAPBR - Bearer settings for applications based on IP
    if(sendATCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"")){ //3 - Set bearer perameters
        Serial.println(fonaResponse);
    }
    Serial.print("Set APN: ");
    if(setAPN()) {
        Serial.println(fonaResponse);
    }
    if(sendATCommand("AT+SAPBR=1,1")) { //Open Bearer
        if(fonaResponse == "OK") {
            Serial.println("Engaged GPRS");
        } else {
            Serial.println("GPRS Already on");
        }
    }
}

void makeHTTPRequest() { //Make HTTP GET (or POST) request then close out GPRS connection
    /*  Lots of other options in the HTTP setup, see the datasheet: google -sim800_series_at_command_manual 
     *  Or go here: https://www.adafruit.com/products/1946 and scroll to the bottom
     */
    Serial.print("HTTP Initialized: ");
    //this checks if it is on. If it is, it's turns it off then back on again. (This Is probably not needed. )
    if(sendATCommand("AT+HTTPINIT")){ //initialize HTTP service. If it's already on, this will throw an Error. 
        if(fonaResponse != "OK") { //if you DO NOT respond OK (ie, you're already on)
            Serial.print("Failed, Restarting: ");
            if(sendATCommand("AT+HTTPTERM")) { //TURN OFF
                Serial.print("Trying Again: ");
                if(sendATCommand("AT+HTTPINIT")) { //TURN ON
                    Serial.println(fonaResponse);
                }
            }
        } else {
            Serial.println(fonaResponse);
        }
        Serial.println(fonaResponse);
    }
    Serial.print("Set Bearer Profile ID: ");
    if(sendATCommand("AT+HTTPPARA=\"CID\",1")){ //Mandatory, Bearer profile identifier
        Serial.println(fonaResponse);
    }
    Serial.print("Send URL: "); 
    if(sendURL()){ //sets the URL for Thingspeak.
        Serial.println(fonaResponse);
    }
    Serial.print("Make GET Request: ");
    if(sendATCommand("AT+HTTPACTION=0")){ //make get request =0 - GET, =1 - POST, =2 - HEAD
        Serial.println(fonaResponse);
    }
    Serial.print("Delay for 2sec....");
    delay(2000); //wait for a bit for stuff to complete
    Serial.println("OK");
    Serial.println("Flush Serial Port....");
    flushFONA(); //Flush out the Serial Port
    Serial.print("Read HTTP Response: ");
    if(sendATCommand("AT+HTTPREAD")){ //Read the HTTP response and print it out
        Serial.println(fonaResponse);
    }
    Serial.print("Delay for 2sec...");
    delay(2000);//wait some more
    Serial.println("OK");
    Serial.println("Flush Serial Port.....");
    flushFONA(); //Flush out the Serial Port
    Serial.print("Terminate HTTP: ");
    if(sendATCommand("AT+HTTPTERM")){ //Terminate HTTP session. (You can make multiple HTTP requests while HTTPINIT is active. Maybe even to multiple URL's? I don't know)
        Serial.println(fonaResponse);
    }
    Serial.print("Disengage GPRS: ");
    if(sendATCommand("AT+SAPBR=0,1")){ //disengages the GPRS context.
        Serial.println(fonaResponse);
    }
}

boolean sendATCommand(char Command[]) { //Send an AT command and wait for a response
    byte complete = 0; // have we collected the whole response?
    char c; //capture serial stream
    String content; //place to save serial stream
    unsigned long commandClock = millis(); //timeout Clock
    fonaSS.println(Command); //Print Command
    while(!complete && commandClock <= millis() + fonaATtimeOut) { //wait until the command is complete
        while(!fonaSS.available() && commandClock <= millis()+fonaATtimeOut); //wait until the Serial Port is opened
        while(fonaSS.available()) { //Collect the response
            c = fonaSS.read(); //capture it
            if(c == 0x0A || c == 0x0D); //disregard all new lines and carrige returns (makes the String matching eaiser to do)
            else content.concat(c); //concatonate the stream into a String
        }
        //Serial.println(content); //Debug
        fonaResponse = content; //Save it out to a global Variable (How do you return a String from a Function?)
        complete = 1;  // label as Done.
    }
    if (complete == 1) return 1; //Is it done? return a 1
    else return 0; //otherwise don't (this will trigger if the command times out) 
    /*
        Note: This function may not work perfectly...but it works pretty well.
        I'm not totally sure how well the timeout function works. It'll be worth testing. 
        Another bug is that if you send a command that returns with two responses, an OK, 
        and then something else, it will ignore the something else and just say DONE as soon as the first response happens. 
        For example, HTTPACTION=0, returns with an OK when it's intiialized, then a second response when the action is complete. 
        OR HTTPREAD does the same. That is poorly handled here, hence all the delays up above. 
    */
}

boolean setAPN() { //Set the APN. See sendATCommand for full comments on flow
    byte complete = 0;
    char c;
    String content;
    unsigned long commandClock = millis();                      // Start the timeout clock
    fonaSS.print("AT+SAPBR=3,1,\"APN\",\"");
    fonaSS.print(APN);
    fonaSS.print("\"");
    fonaSS.println();
    while(!complete && commandClock <= millis() + fonaATtimeOut) {
        while(!fonaSS.available() && commandClock <= millis() + fonaATtimeOut);
        while(fonaSS.available()) {
            c = fonaSS.read();
            if(c == 0x0A || c == 0x0D);
            else content.concat(c);
        }
        fonaResponse = content;
        complete = 1; 
    }
    if (complete == 1) return 1;
    else return 0;
}

boolean sendURL() { //builds url for Thingspeak GET Request, sends request and waits for reponse. See sendATCommand() for full comments on the flow
  byte complete = 0;
  char c;
  String content;
  unsigned long commandClock = millis();                      // Start the timeout clock
  //Print all of the URL components out into the Serial Port
  fonaSS.print("AT+HTTPPARA=\"URL\",\"");
  fonaSS.print(IOT_URL);
  fonaSS.print("?api_key=");
  fonaSS.print(writeKey);
  fonaSS.print("&lat=");
  fonaSS.print(Lat);
  fonaSS.print("&long=");
  fonaSS.print(Long);
  fonaSS.print("&field1=");
  fonaSS.print(ph_data);
  fonaSS.print("&field2=");
  fonaSS.print(DO);
  fonaSS.print("&field3=");
  fonaSS.print(sat);
  fonaSS.print("&field4=");
  fonaSS.print(temp_data);
  fonaSS.print("&field5=");
  fonaSS.print(String(WQI));
  fonaSS.print("&field6=");
  fonaSS.print(waterQuality);
  fonaSS.print("\"");
  fonaSS.println(); 
  /*
  //>>>>>DEBUG<<<<<<
  Serial.print("AT+HTTPPARA=\"URL\",\"");
  Serial.print("http://data.sparkfun.com/input/");
  Serial.print(publicKey);
  Serial.print("?private_key=");
  Serial.print(privateKey);
  for (int i_url=0; i_url<NUM_FIELDS; i_url++) {
    Serial.print("&");
    Serial.print(fieldNames[i_url]);
    Serial.print("=");
    Serial.print(fieldData[i_url]);
  }
  Serial.print("\"");
  Serial.println();
  //>>>>>>>>>>>>>>>>>>>>
  */
  while(!complete && commandClock <= millis() + fonaATtimeOut) {
    while(!fonaSS.available() && commandClock <= millis() + fonaATtimeOut);
    while(fonaSS.available()) {
      c = fonaSS.read();
      if(c == 0x0A || c == 0x0D);
      else content.concat(c);
    }
    fonaResponse = content;
    complete = 1; 
  }
  if (complete == 1) return 1;
  else return 0;
}

void flushFONA() { //if there is anything is the fonaSS serial buffer, clear it out and print it in the Serial Monitor.
    char inChar;
    while (fonaSS.available()){
        inChar = fonaSS.read();
        Serial.write(inChar);
        delay(20);
    }
}

void turnOnFONA() { //turns FONA ON
  if(! digitalRead(FONA_PS)) { //Check if it's On already. LOW is off, HIGH is ON.
    Serial.print("FONA was OFF, Powering ON: ");
    digitalWrite(FONA_KEY,LOW); //pull down power set pin
    unsigned long KeyPress = millis();
    while(KeyPress + fonaKeyTime >= millis()) {} //wait two seconds
    digitalWrite(FONA_KEY,HIGH); //pull it back up again
    Serial.println("FONA Powered Up");
  } else {
    Serial.println("FONA Already On, Did Nothing");
  }
}

void turnOffFONA() { //turns FONA OFF
  if(digitalRead(FONA_PS)) { //check if FONA is OFF
    Serial.print("FONA was ON, Powering OFF: "); 
    digitalWrite(FONA_KEY,LOW);
    unsigned long KeyPress = millis();
    while(KeyPress + fonaKeyTime >= millis()) {}
    digitalWrite(FONA_KEY,HIGH);
    Serial.println("FONA is Powered Down");
  } else {
    Serial.println("FONA is already off, did nothing.");
  }
}
 
void serialEvent() {
  // this interrupt will trigger when the data coming from the serial monitor (pc/mac/other) is received.

/**********************************************************************************
  Get data from the pH & DO sensors on the serial monitor. This is good for testing
 **********************************************************************************/

  // if Arduino_only equals 1 this function will be bypassed. 
  if(arduino_only!=1) {
    // first dissolved oxygen
      
    // We read the data sent from the serial monitor(pc/mac/other) until we see a <CR>.
    // We also count how many characters have been received
    // (readBytesUntil() returns the number of characters read into the buffer. A 0 means no valid data was found.)
    received_from_computer_do = Serial.readBytesUntil(13,computerdata_do,20);
    // We add a 0 to the spot in the array just after the last character we received.
    // This will stop us from transmitting incorrect data that may have been left in the buffer.
    computerdata_do[received_from_computer_do] = 0;
    // We transmit the data received from the serial monitor(pc/mac/other) through the soft serial port to the do Circuit. 
    doSS.print(computerdata_do);
    // all data sent to the do circuit must end with a <CR>.        
    doSS.print('\r'); 

    // now pH

    // we read the data sent from the serial monitor(pc/mac/other) until we see a <CR>.
    // We also count how many characters have been received
    // (readBytesUntil() returns the number of characters read into the buffer. A 0 means no valid data was found.) 
    received_from_computer_ph = Serial.readBytesUntil(13,computerdata_ph,20);
    //we add a 0 to the spot in the array just after the last character we received.
    // This will stop us from transmitting incorrect data that may have been left in the buffer.
    computerdata_ph[received_from_computer_ph] = 0;
    // we transmit the data received from the serial monitor(pc/mac/other) through the soft serial port to the ph circuit. 
    phSS.print(computerdata_ph);
    // all data sent to the ph circuit must end with a <CR>.        
    phSS.print('\r'); 
   }
}





