// Rotary encoder code for active avoidance
// Position is the number of ticks not the physical distance.
// Warning: Speed is scaled by 1000 to avoid floating point math.
// Select your threshold accordingly.

#include <Encoder.h> // get it from https://github.com/PaulStoffregen/Encoder
#include <digitalWriteFast.h> // get it from: https://github.com/NicksonYap/digitalWriteFast

#define VISSTIM 7 // visual stim input from ML
#define OUTPIN 8 // button output to ML
//#define VISSTIM 13 // visual stim input from ML. Debugging only
//#define OUTPIN 11 // button output to ML. Debugging only
#define CAMPIN 12 // camera output to face camera

Encoder myEnc(2,3); // encoder is connected to interrupts 1&2

byte m, *b; // for writing to serial

// pulsing variables
const int sampling = 20; // Hz. Sampling rate for speed estimation
const long interval = 1000L / sampling; // ms. Calculate the period from the sampling rate 
const int pulseDur = 10; // ms. Pulse duration
int pulseCount = 0;

// buffer variables
const int samplingDur = 200; // sampling duration after on/offset.
const int bufferDur = 2 * samplingDur; // so that the buffer is centered around on/offset
const int bufferLen = (bufferDur / interval) + 1;
long bufferArr[bufferLen];

// position variables
long pos = 0L; // position from the encoder
long prevPos = 0L; // previous position to use to calculate the instantaneous speed 
long scale = 2L;

// speed variables
long vel = 0L; // velocity
int threshold = ((4 / scale) * 1000) / samplingDur ; // cm/sec. Threshold to exceed for a correct trial
long beforeVel = 0L;

// timing variables
unsigned long previousMillis = 0;
unsigned long visMillis = 0;
unsigned long visPrevMillis = 0;
unsigned long outMillis = 0;

// state variables
bool pulse = false;
bool visState = false;
bool samplingState = false;
bool outState = false;

// training mode
bool training = true; // change to true if training

// debug mode
bool debug = true;

void setup()
{  
  // Initialize the pins
  pinModeFast(VISSTIM, INPUT);
  pinModeFast(OUTPIN, OUTPUT);
  pinModeFast(CAMPIN, OUTPUT);
  
  Serial.begin(9600); // Note to user: Double check your baudrate to match to MATLAB's
  if (debug) {String text="Buffer length "+String(bufferLen)+" samples at "+String(sampling)+"Hz.";Serial.println(text);}

  // Wait for serial port to connect
  if (!training) { // No need to wait for serial if it is training
    while (!Serial) {
      ; // do nothing as you wait
      }
    }
  if (debug) {Serial.print("Serial begins...\nThreshold is set to ");Serial.println(threshold);}
  myEnc.write(0); // Intialize the encoder with "0"
}

void loop() { 
  // Every loop first read the tick position 
  pos = myEnc.read();
  
  // Time stamp the current loop
  unsigned long currentMillis = millis();

  // Pulse at <sampling> to calculate the instantaneous speed
  // Pulse block starts
  if (!pulse && currentMillis - previousMillis >= interval - pulseDur) {
    previousMillis = currentMillis; // timestamp the this pulse

    // Calculate the instantaneous speed
    // Warning: Speed is scaled by 1000 to avoid floating point math
    vel = ((pos - prevPos) * 1000) / interval;
    if (debug && vel > 0) {Serial.print("Speed: ");Serial.println(vel);}
    
    // Write the position to the serial
    b = (byte *) &pos;
    Serial.write(b, 4);

    // Update the position in the buffer
    bufferArr[pulseCount] = vel;
    pulseCount++;
    if (pulseCount > bufferLen) {
      pulseCount = 0;
      }

    digitalWriteFast(CAMPIN, HIGH); // trigger a camera frame
    
    prevPos = pos; // log the last position to use in the next loop
    pulse = !pulse; // change the pulse state
    }

  if (pulse && currentMillis - previousMillis >= pulseDur) {
    digitalWriteFast(CAMPIN, LOW);
    
    previousMillis = currentMillis;
    pulse = !pulse;
    }
  // Pulse block ends

  // Calculate the average speed from the buffer
  long velSum = 0L;
  for (int i=0; i < bufferLen; i++) {
    velSum = velSum + bufferArr[i];
    }
  long averageVel = velSum / bufferLen;
//  if (debug && averageVel>0) {Serial.print("MeanSpeed: ");Serial.println(averageVel);}

  // Visual stimulus detection block starts
  bool visStim = digitalReadFast(VISSTIM); // check if visual stim is on
  if (!visState && visStim) {
    // Visual stimulus is off
    visState = true;
    beforeVel = averageVel;
    if (debug) {Serial.println("RISING");}
    }
  else if (visState && !visStim) {
    // Visual stimulus is on
    visState = false;
    visMillis = millis(); // time stamp the visual stimulus
    samplingState = true;
    if (debug) {Serial.println("FALLING");}
    }
  // Visual stimulus detection block ends

  // ML pulse block starts
  if (samplingState && currentMillis - visMillis >= samplingDur) {
    long afterVel = averageVel;
    samplingState = false;

    // Send out a pulse to ML if speed difference exceeds the threshold
    if (!outState && afterVel - beforeVel > threshold) {
      digitalWriteFast(OUTPIN, HIGH); // trigger ML button
      if (debug) {Serial.println(" OUT-on");}
      if (debug && averageVel>0) {Serial.print("MeanSpeed: ");Serial.println(averageVel);}
      outMillis = millis(); // time stamp the output pulse
      outState = true;
      }
    else {
      if (debug) {Serial.println("FAIL");}
      }
    }
    
  if (outState && currentMillis - outMillis >= 101) {
    if (debug) {Serial.println("OUT-off");}
    digitalWriteFast(OUTPIN, LOW);
    outState = false;
    }
  // ML pulse block ends

}
