/*************************************************************************************************************
 * Retrieve and post information about water quality using Atlas Scientific pH and dissolved oxygen sensors 
 * and post them to an IOT cloud server using an Adafruit FONA. This is for the floating data wetlands project 
 * portion of OEE (Office of Environmental Experiments) http://oee.ccastellanos.com
 * 
 * Much of the FONA code is borrowed and modified from Adafruit's own exmaple code:
 * https://learn.adafruit.com/adafruit-fona-mini-gsm-gprs-cellular-phone-module
 * And Hans Huth's solar-powered FONA project:
 * http://fritzing.org/projects/fona-and-solar-lipo-battery-charger/
 * 
 * Much of the sensor code is modified from example code provided by Atlas Scientific:
 * http://atlas-scientific.com
 * 
 * created March 28, 2016
 * by Carlos Castellanos
*************************************************************************************************************/

#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_DotStar.h>
#include <LowPower.h>
#include <avr/pgmspace.h>
#include <Adafruit_FONA.h>

// baud rates for serial communication
#define BAUD_RATE 115200
#define SENSOR_BAUD_RATE 9600
#define FONA_BAUD_RATE 4800

/*******************
ADAFRUIT DOTSTAR LED
*******************/
#define NUMPIXELS 15        // Number of LEDs we are driving

// variables related to the blinking/fading of the LEDs
uint16_t brightness =  0;   // how bright the LED is
uint16_t fadeAmount =  5;   // how many points to fade the LED by
#define BRIGHTNESS 0
#define FADEAMOUNT 5
unsigned long currentTime;  // this is for timeing of the fading/blinking of the LED's
unsigned long loopTime;     // so is this

// create a list of colors for the lights
// we are basically creating an HSV gradient here
#define NUMCOLORS 25
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
// Default is DOTSTAR_BRG, which apprently doesn't work with the latest
// production runs. DOTSTAR_BGR worked for me.

// Hardware SPI is a little faster, but must be wired to specific pins
// (Arduino Uno & Pro/Pro Mini = pin 11 for data, 13 for clock, other boards are different).
// And that's what we're doing!
Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_BGR);

/********************************
ADAFRUIT FONA & IOT COMMUNICATION
********************************/
#define FONA_RX 3     // Arduino serial rx pin (from FONA tx)
#define FONA_TX 4     // Arduino serial tx pin (to FONA rx)
#define FONA_RESET 5  // FONA RESET pin
#define FONA_KEY 6    // FONA KEY pin
#define FONA_PS 12    // FONA POWER STATUS pin

//const char APN[] = "uninet";                            //Set APN for Mobile Service (China Unicom) ("cmhk" for China Mobile HK)

SoftwareSerial fonaSS = SoftwareSerial(FONA_RX, FONA_TX); // initialize software serial
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RESET);           // initialize FONA object

// ====== IOT server (Thingspeak) URL Building ====== //
//const char PROGMEM writeKey[] = "8G6ITUXGW2H7LYRX";                                 // Write API Key for writing data to Thingspeak OEE Wuhan channel
//const String readKey = "FRMG4PS99P4TVTW7";                                          // Read API Key for reading data from Thingspeak OEE Wuhan channel
//const String thingSpeakChannelID = "106708";                                        // Thingspeak Channel ID
const String IOT_URL = "http://184.106.153.149/update?key=8G6ITUXGW2H7LYRX";          // Thingspeak update url (can also use api.thingspeak.com/update/)
String Lat;                                                                           // variables to hold location data (lat & long)
String Long;
//String Date;                                                                        // variables to hold date & time
//String Time;
const String defaultLat = "22.2975845"; // HK //"30.646";    // Wuhan
const String defaultLong = "114.1737909"; //"114.239";
String tempUrl;                                                                       // temp variable used for url-building

// ====== WeChat Bot URL Building ====== //
// TODO, v.2



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
byte arduino_only=1;                  //To operate the pH & D.O. circuits with the Arduino only and not use the serial monitor to send it commands set this to 1.
                                      //The data will still come out on the serial monitor, so you can see it working.

// === Dissolved Oxygen === //
char DO_data[20];                     //we make a 20 byte character array to hold incoming data from the D.O. 
char computerdata_do[20];             //we make a 20 byte character array to hold incoming data from a pc/mac/other. 
byte received_from_computer_do=0;     //we need to know how many characters have been received.                                 
//byte received_from_do_sensor=0;       //we need to know how many characters have been received. 
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
//byte received_from_ph_sensor=0;       //we need to know how many characters have been received.
byte string_received_ph=0;            //used to identify when we have received a string from the pH circuit.
byte string_converted_ph=0;           //used to identify when the string from the pH circuit has been converted (into a float).
float ph=0;                           //used to hold a floating point number that is the pH. 

// === Water Temperature === //
// grab temperature from analog temperature sensor (TMP36) or a thermistor (NTC B25/50: 3950)
// this is important for calibrating the D.O. & pH sensors
#define TEMPSENSORPIN A0                       // analog sensor pin
char temp_data[8];                             // 8 byte character array to hold the temperature data 
float waterTemp = 20;                          // used to hold a floating point number for the temperature (default is 20C for D.O. and 25C for pH)
#define VCC 5000                               // VCC (in millivolts)
#define NUM_THERMISTOR_SAMPLES 5               // how many samples to take and average (if using a thermistor), more takes longer but is smoother
int thermistorSamples[NUM_THERMISTOR_SAMPLES]; // array to hold the thermistor samples
#define THERMISTOR_FIXED_RESISTOR_VAL 10000    // the thermistor is set up as a basic voltage divider circuit so we need the value of the fixed resistor
#define THERMISTOR_RESISTANCE_NOMINAL 10000    // resistance at 25 degrees C
#define THERMISTOR_TEMPERATURE_NOMINAL 25      // temp. for nominal resistance (almost always 25 C)
#define BCOEFFICIENT 3950                      // The beta coefficient of our thermistor (usually 3000-4000)

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
#define SLEEP_SECS 896        // Sleep time (900 secs = 15 min)
//#define INTERRUPT_PIN 2       // FONA RING INDICATOR (RI) pin connected to this pin. Phone call or SMS will trigger it 


void setup() {
  Serial.begin(BAUD_RATE);        // enable the hardware serial port

  doSS.begin(SENSOR_BAUD_RATE);   // enable the DO sensor software serial port
  phSS.begin(SENSOR_BAUD_RATE);   // enable the pH sensor software serial port
  
  pinMode(FONA_RESET, OUTPUT);    // set-up this pin as an output so we can hard reset the FONA if necessary
  pinMode(FONA_PS, INPUT);        // not being used in this version
  pinMode(FONA_KEY,OUTPUT);
  //fonaSS.begin(BAUD_RATE);      // enable the FONA software serial port
  turnOnFONA();                   // turn on the FONA
  //fona.setGPRSNetworkSettings(F("uninet"), F("uninet"), F("uninet")); // Wuhan
  fona.setGPRSNetworkSettings(F("cmhk")); // HK

  strip.begin();                  // Initialize LED pins for output
  strip.clear();                  // Set all pixel data to zero
  strip.show();                   // Turn all LEDs off ASAP
  
  // this is for fading/blinking the LEDs
  currentTime = millis();
  loopTime = currentTime;

  // reserve memory space for some strings to avoid fragmentation
  Lat.reserve(15);
  Long.reserve(15);
  tempUrl.reserve(180);

  // Enable the interrupt on the interrupt pin
  // so that a phone call or sms on the FONA
  // will wake the Arduino
  // The internal pull-up is turned on
  // so the pin is in a well defined state (HIGH)
  // The FONA RI pin will be attached to this pin.
  // FONA RI is HIGH by default and will pulse LOW
  // on a call or SMS, which will trigger the interrupt
  //pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), blinkISR, FALLING); // attach the interrupt
}

void loop() {
  // first read the sensors
  ArduinoSensorControl();
  
  // analyze sensor readings and determine a water quality index (0-100)
  // average of the WQI of the pH & d.o.
  WQI = round((calculateWQIdo() + calculateWQIph()) / 2);
  Serial.print(F("Water Quality Index: "));
  Serial.print(WQI);
  // get the water quality rating
  if(WQI >= 0 && WQI <= 25) {
    waterQuality = F("very_bad");
  } else if(WQI >= 26 && WQI <= 50) {
    waterQuality = F("bad");
  } else if(WQI >= 51 && WQI <= 70) {
    waterQuality = F("medium");
  } else if(WQI >= 71 && WQI <= 90) {
    waterQuality = F("good");
  } else if(WQI >= 91) {
    waterQuality = F("excellent");
  }
  Serial.print(F(": "));
  Serial.println(waterQuality);

  // update the LED colors based upon the most recent sensor readings & resulting water quality index
  updateLEDColor();

   // update the IOT server with the most recent sensor data
   // turn on GPRS on the FONA & send data to IOT server (Thingspeak)
   fonaSerial->listen(); // By default, the last intialized port is listening, so we need to explicitly select the port we want to listen to
   while(!fonaSerial->isListening()) {;}
   Serial.println(F("Turning on GPRS"));
   if (!fona.enableGPRS(true))  {
      Serial.println(F("Failed to turn on GPRS"));
      fona.enableGPRS(false); // turn off gprs
    } else {
       Serial.println(F("FONA GPRS is ON"));
       //delay(5000);
       sendIOTData();
       // turn GPRS off
       if (!fona.enableGPRS(false)) {
        Serial.println(F("Failed to turn off GPRS."));
       } else {
        Serial.println(F("Looks good!"));
       }
    }

  // if the blinkFlag = 1 (we received a message from WeChat or the web), blink the lights (TODO)
  if(blinkFlag) {
    // blink the lights, send message about water quality
    blinkLEDs();
    // set blinkFlag back to 0
    blinkFlag = 0;
  }
  
  // put the Arduino & FONA to sleep once the sensor readings have been taken and processed
  if (string_converted_do==1 && string_converted_ph==1) {
    string_converted_do = 0;
    string_converted_ph = 0;
    gotoSleep();
  } else {
    flushSerial();
    flushDO();
    flushPH();
    flushFONA();
  }
  // run the wake up function if we are just getting out of sleep (i.e. gotoSleep() was just called)
  if(asleep) {
    wakeUp();
  } else {
    delay(60000);
  }

/*
  delay(60000);
  flushSerial();
  flushDO();
  flushPH();
  flushFONA();
*/
}

// === Interrupt Service Routine == //
/*void blinkISR() {
    blinkFlag = 1;
}*/

// === Sleep/Wake Power-saving functions === //
void gotoSleep() {
  // put Arduino, sensors & FONA to sleep

  Serial.println(F("sleeping..."));

  // flush arduino serial buffer
  flushSerial();
  
  // first clear the serial read & write buffers for the d.o. & ph sensors
  flushDO();
  flushPH();

  // put the d.o & pH sensors to sleep
  doSS.print(F("SLEEP\r"));
  phSS.print(F("SLEEP\r"));
  
  // set the sleep flag to 1
  asleep = 1;
  
  // put the FONA to sleep
  // TODO (next version, have it sleep overnight, & wake up after x hours)
  // first flush the FONA serial buffer 
  flushFONA();


  // put Arduino to sleep
  for(uint16_t i = 0; i < (SLEEP_SECS / 4); i++) {
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
  }
}

void wakeUp() {
  // wake sensors & FONA (Arduino is already awake)
  Serial.println(F("waking up..."));

  // Wake up FONA
  // TODO (next version, have it sleep overnight, & wake up after x hours)

  // D.O. & pH sensors
  // After the d.o. & pH sensors wake up, several consecutive readings (16 for d.o., 4 for pH) should be taken before the readings are considered valid.
  // so let's take those readings now
  for(byte i = 0; i < 16; i++) {
    doSS.print(F("R\r"));    // send the d.o. sensor the command to take a single reading.
    phSS.print(F("R\r"));    // send the pH sensor the command to take a single reading.
    delay(2);                // short delay to give the sensors time to take the readings
  }
  // clear the read buffers
  flushDO();
  flushPH();

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
      strip.show();                                 // show the updated pixels
      brightness = brightness + fadeAmount;         // change the brightness for next time through the loop
      if(brightness == 0 || brightness == 255) {    // reverse the direction of the fading at the ends of the fade
        fadeAmount = -fadeAmount;
        flag++;                                     // increment the flag (after 3 fade in/outs we exit the loop)
      }
      loopTime = currentTime;                       // update loopTime
    }
  }
  
  strip.setBrightness(255);                         // set strip back to full brightness before we exit the function
  strip.show();                                     // show the updated pixels
  brightness = BRIGHTNESS;                          // also set the brightness variable back to its original setting
  fadeAmount = FADEAMOUNT;                          // and the fade amount as well
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
  for (uint16_t i = 0; i < NUMPIXELS; ++i) {
      strip.setPixelColor(i, color);
  }
  // show the updated pixels
  strip.show();
}

// === Water Quality Sensor Functions === //
void parseDOdata(byte *receivedFromDOsensor) {
  // The d.o. reading sensor actually returns two readings by default:
  // the d.o. reading itself (in mg/L) and the saturation percentage
  // we need to parse this data
  byte parseFlag = 0;
  // look for a comma in the data stream
  for(byte i=0;i<=*receivedFromDOsensor;i++) {
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

    //Serial.print("DO:");             // We now print each value we parsed seperatly. 
    //Serial.println(DO);              // send the d.o. value to the serial monitor
    DO_float=atof(DO);                 // turn the string into to float value.

    //Serial.print("%sat:");           // We now print each value we parsed seperatly.  
    //Serial.println(sat);             // send the %sat value to the serial monitor 
    sat_float=atof(sat);               // turn the string into to float value.

    string_converted_do = 1;           // set the string converted flag to 1 (true).
  }
}

void ArduinoSensorControl() {
  // take the d.o. & pH sensor readings

  // If the variable arduino_only is set to 1 we will
  // let the Arduino take over control of the d.o. & pH circuits
  // this is where we actually ask the sensors to take a reading
  if(arduino_only==1) {
        if(startup == 0) {                 // if the Arduino just booted up, we need to set some things up first.
            flushDO();                     // first flush the sensor serial buffers
            flushPH();
            
            doSS.print(F("C,0\r"));        // take the d.o. Circuit out of continuous mode.   
            phSS.print(F("C,0\r"));        // take the pH Circuit out of continuous mode. 
            delay(50);                     // on start up sometimes the first command is missed.
            doSS.print(F("C,0\r"));        // so, let’s send it twice. (this is actually pretty important)
            phSS.print(F("C,0\r"));
            delay(50);                     // a short delay after the circuits were taken out of continuous mode is used to make sure we don't overload them with commands.
            doSS.print(F("RESPONSE,0\r")); // disable response code so we don't get the "*OK" message when working in arduino_only mode
            phSS.print(F("RESPONSE,0\r"));
            delay(10);                     // another short delay
            doSS.print(F("O,%,1\r"));      // make sure d.o. circuit transmits the sat %
            delay(10);                     // another short delay
            
  /*
   * This code is for a TMP36 analog tmeperature sensor
   * 
            // now lets get the temperature of the water (in Celsius)
            // this makes sure the d.o. & ph sensors stay properly calibrated
            uint16_t sensorVal = analogRead(TEMPSENSORPIN);
            // Convert the analog reading (which goes from 0 - 1023) to a voltage in millovolts (0 - 5000 mV):
            float tempMV = sensorVal * (VCC / 1023.0);
            // now use this formula to convert the voltage to a temperature:
            // Temp in °C = [(Vout in mV) - 500] / 10
            waterTemp = (tempMV - 500) / 10;
  */
  
  /*
   * This code is for an NTC 10k B25/50: 3950 Thermistor
  */
            // read the analog sensor pin
            // we take a few of readings in a row and average them, this gives us more accurate results
            // take N samples in a row, with a slight delay
            uint8_t i;
            for (i=0; i< NUM_THERMISTOR_SAMPLES; i++) {
              thermistorSamples[i] = analogRead(TEMPSENSORPIN);
              delay(10);
            }
   
            // average all the samples out
            float average = 0;
            for (i=0; i< NUM_THERMISTOR_SAMPLES; i++) {
              average += thermistorSamples[i];
            }
            average /= NUM_THERMISTOR_SAMPLES;
            
            // the thermistor is set up as a basic voltage divider circuit
            // we first need to determine the resistance of the thermistor in order to determine the temperature
            // so first convert the voltage coming from the analog pin into a resistance
            average = (1023 / average)  - 1;
            average = THERMISTOR_FIXED_RESISTOR_VAL / average; // THERMISTOR_FIXED_RESISTOR_VAL is the value of the fixed resistor (in ohms)
  
            // now we need to convert to temperature
            // we are using the simplified B parameter version of the Steinhart-Hart equation: https://en.wikipedia.org/wiki/Thermistor.
            // this is fine since our temps will be ithin a fairly small range (20-25C)
            // this code is taken from: https://learn.adafruit.com/thermistor
            float waterTemp;
            waterTemp = average / THERMISTOR_RESISTANCE_NOMINAL;          // (R/Ro)
            waterTemp = log(waterTemp);                                   // ln(R/Ro)
            waterTemp /= BCOEFFICIENT;                                    // 1/B * ln(R/Ro)
            waterTemp += 1.0 / (THERMISTOR_TEMPERATURE_NOMINAL + 273.15); // + (1/To)
            waterTemp = 1.0 / waterTemp;                                  // Invert
            waterTemp -= 273.15;                                          // convert to C
            
            // convert waterTemp val to a char array (string), then send the temperature to the d.o. & pH sensors
            dtostrf(waterTemp, 4, 2, temp_data);  // this is the standard avr-libc function for converting floats to Strings - http://is.gd/KausLc
            doSS.print(F("T,"));                  // send the temperature to the d.o. sensor
            doSS.print(temp_data);
            doSS.print(F("\r"));
            phSS.print(F("T,"));                  // send the temperature to the ph sensor
            phSS.print(temp_data);
            phSS.print(F("\r"));
  
            Serial.print(F("Water temp = "));
            Serial.println(temp_data);
            startup = 1;                         // startup is completed, let's not do this again during normal operations.
        }

    // -- d.o. -- //
    doSS.listen();          // By default, the last intialized port is listening, so we need to explicitly select the port we want to listen to
    while(!doSS.isListening()) {;}
    doSS.print(F("R\r"));   // send the d.o. sensor the command to take a single reading.
    // small delay to make sure we get the data back from the sensors without having to loop
    // this way we only have to do this once before going to sleep
    // remember delays do not disable interrupts and serial communication should still work
    delay(1000);
    // if we see that the d.o. circuit has sent a character.
    byte received_from_do_sensor;
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
    } else {
      Serial.println(F("nothing from d.o."));
    }

    if(string_received_do == 1) {                    // did we get data back from the do Circuit?
      if((DO_data[0] >= 48) && (DO_data[0] <=57)) {  // if DO_data[0] is a digit and not a letter (these are ascii values)
        // we need to parse the do data
        parseDOdata(&received_from_do_sensor);       // do some parsing!
        string_received_do = 0;                      // reset the string received flag.
      }
    }

    // -- pH -- //
    phSS.listen();           // By default, the last intialized port is listening, so we need to explicitly select the port we want to listen to
    while(!phSS.isListening()) {;}
    phSS.print(F("R\r"));    // send the ph sensor the command to take a single reading
    // small delay to make sure we get the data back from the sensors without having to loop
    // this way we only have to do this once before going to sleep
    // remember delays do not disable interrupts and serial communication should still work
    delay(1000);
    
    // if we see that the pH circuit has sent a character.
    if(phSS.available() > 0) {
      // We read the data sent from pH Circuit until we see a <CR>.
      // We also count how many characters have been received
      // (readBytesUntil() returns the number of characters read into the buffer. A 0 means no valid data was found.)
      byte received_from_ph_sensor = phSS.readBytesUntil(13,ph_data,20);
      // We add a 0 to the spot in the array just after the last character we received.
      // This will stop us from transmitting incorrect data that may have been left in the buffer.
      ph_data[received_from_ph_sensor] = 0;
      // a flag used when the Arduino is controlling the pH Circuit to let us know that a complete string has been received.
      string_received_ph = 1;
      // lets transmit that data received from the pH Circuit to the serial monitor.
      Serial.println(ph_data);
    } else {
      Serial.println(F("nothing from pH"));
    }

    if(string_received_ph == 1) {                    // did we get data back from the ph Circuit?
      ph = atof(ph_data);                            // convert string into a float
      string_converted_ph = 1;                       // set the string converted flag to 1 (true).
      //if(ph>=7.5){Serial.println(F("high\r"));}    // This is the proof that it has been converted into a float.
      //if(ph<7.5){Serial.println(F("low\r"));}      // This is the proof that it has been converted into a float.
      string_received_ph = 0;                        // reset the string received flag.
    }
  }
}

// water quality calculations for D.O.
uint8_t calculateWQIdo() {
  // returns the water quality index (WQI) for d.o.
  // note: the "in" arrays should have increasing values

  // convert the saturation % value into an int
  uint8_t satPercentage = round(sat_float);
  // doSatIn[] holds the measured saturation % values from a d.o. sensor
  int doSatIn[]  = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140};
  // wqOut[] holds the corresponding WQI values
  int wqOut[] = {2,7,12,19,30,44,57,75,87,95,99,96,90,84,78};

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

// === FONA GSM/GPRS/IOT functions === //
void sendIOTData() {
  flushSerial();
  Serial.println(F("Sending data to IOT server..."));
   
      // try and get the GSM location
      /*float latF, longF;
      if (fona.getGSMLoc(&latF, &longF)) {
        Serial.print(F("Getting GSM loc: "));
        char tempLat[15];
        char tempLong[15];
        dtostrf(latF, 4, 8, tempLat);  // this is the standard avr-libc function for converting floats to Strings - http://is.gd/KausLc
        dtostrf(longF, 4, 8, tempLong);
        Lat = String(tempLat);
        Long = String(tempLong);
      } else { // if unsucessful just use the defaut settings
        Lat = defaultLat;
        Long = defaultLong;
      }*/
          
      // Get website URL and ping it
      // Need to read a string into an array, 
      // since fona.HTTP_GET_start() is expecting an array for url.
      uint16_t statuscode;
      int16_t len;
      char url[180]; // char array buffer to hold the entire url to send to IOT server (Thingspeak)
      tempUrl.concat(IOT_URL);
      tempUrl.concat(F("&lat="));
      tempUrl.concat(defaultLat);
      tempUrl.concat(F("&long="));
      tempUrl.concat(defaultLong);
      tempUrl.concat(F("&field1="));
      tempUrl.concat(ph_data);
      tempUrl.concat(F("&field2="));
      tempUrl.concat(DO);
      tempUrl.concat(F("&field3="));
      tempUrl.concat(sat);
      tempUrl.concat(F("&field4="));
      tempUrl.concat(temp_data);
      tempUrl.concat(F("&field5="));
      tempUrl.concat(String(WQI));
      tempUrl.concat(F("&field6="));
      tempUrl.concat(waterQuality);

      // Get tempUrl into the array url
      tempUrl.toCharArray(url,180);
      Serial.println(url);
      tempUrl = ""; // empty the string!

      // Make the post
      if (!fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&len)) {
       // post failed
       Serial.print(F("Failed: "));
      } else  {
       // Post successed 
       while (len > 0) {
        while (fona.available()) {
         char c = fona.read();
            // Serial.write is too slow, we'll write directly to Serial register!
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); // Wait until data register empty.
            UDR0 = c;
#else
            Serial.write(c);
#endif
         len--;
         if (! len) {}// break;
         }
       }
       Serial.println(F("\n****"));
       fona.HTTP_GET_end();
     }
     blinkLEDs(); // in this version, we will blink the lights after sensor readings are taken and the data is sent to the IOT server
}

// === Serial port utility functions === //
void flushFONA() { //if there is anything is the fona serial buffer, clear it out and print it in the Serial Monitor.
    fonaSerial->listen();
    while (fona.available()) {
      Serial.write(fona.read());
   }
}

void turnOnFONA() { //turns FONA ON
  fonaSerial->begin(FONA_BAUD_RATE); // open the FONA serial port
  
  if (! fona.begin(*fonaSerial)) { // if FONA not powered/reset
    Serial.println(F("Couldn't find FONA"));
  } else {
    Serial.println(F("Initializing FONA: please wait 15 secs..."));
    delay(15000); // give the FONA 15 secs to initialize
    Serial.println(F("FONA on and initialized"));
  }
}

void flushDO() { //if there is anything is the d.o. serial buffer, clear it out and print it in the Serial Monitor.
    doSS.listen();
    while (doSS.available()) {
      Serial.write(doSS.read());
   }
}
void flushPH() { //if there is anything is the pH serial buffer, clear it out and print it in the Serial Monitor.
    phSS.listen();
    while (phSS.available()) {
      Serial.write(phSS.read());
   }
}

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

void serialEvent() {
  // this interrupt will trigger when the data coming from the serial monitor (pc/mac/other) is received.

/**********************************************************************************
  Get data from the pH & DO sensors on the serial monitor. This is good for testing
 **********************************************************************************/

  // if arduino_only equals 1 this function will be bypassed. 
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


