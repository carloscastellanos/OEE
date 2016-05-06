/**
 * Serial Read
 * 
 * Read data from the serial port and change the color of a rectangle
 * when a switch connected to a Wiring or Arduino board is pressed and released.
 * This example works with the Wiring / Arduino program that follows below.
 */

/* For more on serial communication in Processing:
http://processing.org/reference/libraries/serial/index.html
*/
import processing.serial.*;

static final int BAUD = 9600; // serial baud rate
Serial myPort;  // Create object from Serial class
float val;      // Data received from the serial port
static final int LF = 10;    // Linefeed in ASCII

void setup() 
{
  size(300, 300);
  // I know that the first port in the serial list on my mac
  // is always my FTDI adaptor, so I usually open Serial.list()[0].
  // On Windows machines, this generally opens COM1.
  // Open whatever port is the one you're using.
  String portName = Serial.list()[4];
  println(portName);
  myPort = new Serial(this, portName, BAUD);
  myPort.clear();
}

/*void serialEvent(Serial myPort){
    while ( myPort.available() > 0) {  // If data is available,
    String myString = myPort.readStringUntil(LF); // read serial data until we see a linefeed character
      if (myString != null) {
        println(myString);  // Prints String
        val = float(myString);  // Converts to float
      }
    }
    myPort.clear();
}*/

void draw() {
  background(255);             // Set background to white
  
  while ( myPort.available() > 0) {  // If data is available,
    String myString = myPort.readStringUntil(LF); // read serial data until we see a linefeed character
      if (myString != null) {
        println(myString);  // Prints String
        val = float(myString);  // Converts to float
      }
   }
    myPort.clear();
  
  // map the sensor values
  // to the luminance and/or size of the square
  float mappedVal = map(val,10.0,50.0,0,255); // this is for temperature
  fill(mappedVal);
  rect(50, 50, 200, 200);
  
  println(val);
}



/*

// Wiring / Arduino Code
// Code for sensing a switch status and writing the value to the serial port.

int switchPin = 4;                       // Switch connected to pin 4

void setup() {
  pinMode(switchPin, INPUT);             // Set pin 0 as an input
  Serial.begin(9600);                    // Start serial communication at 9600 bps
}

void loop() {
  if (digitalRead(switchPin) == HIGH) {  // If switch is ON,
    Serial.write(1);               // send 1 to Processing
  } else {                               // If the switch is not ON,
    Serial.write(0);               // send 0 to Processing
  }
  delay(100);                            // Wait 100 milliseconds
}

*/