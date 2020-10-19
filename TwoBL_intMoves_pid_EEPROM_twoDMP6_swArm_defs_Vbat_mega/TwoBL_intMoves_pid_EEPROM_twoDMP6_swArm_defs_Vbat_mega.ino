/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "Definitions.h"
#include "TimerOne.h"
#include "TimerFive.h"
#include "EEPROM.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//IMUs declaration
#ifdef CAM_MPU
MPU6050 camIMU;
#endif
#ifdef FR_MPU
MPU6050 frIMU(0x69);
#endif

// MPU control/status vars
#ifdef CAM_MPU
bool camIMUReady = false;  // set true if DMP init was successful
uint8_t camIMUdevStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t camIMUpacketSize;    // expected DMP packet size (default is 42 bytes)
uint16_t camIMUfifoCount;     // count of all bytes currently in FIFO
uint8_t camIMUfifoBuffer[64]; // FIFO storage buffer
#endif
#ifdef FR_MPU
bool frIMUReady = false;  // set true if DMP init was successful
uint8_t frIMUdevStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t frIMUpacketSize;    // expected DMP packet size (default is 42 bytes)
uint16_t frIMUfifoCount;     // count of all bytes currently in FIFO
uint8_t frIMUfifoBuffer[64]; // FIFO storage buffer
#endif

// orientation/motion vars
#ifdef CAM_MPU
Quaternion camIMUq;               // [w, x, y, z] quaternion container
VectorFloat camIMUgravity;        // [x, y, z]  gravity vector
float camIMUypr[3] = {0, 0, 0};   // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
#endif
#ifdef FR_MPU
Quaternion frIMUq;                // [w, x, y, z] quaternion container
VectorFloat frIMUgravity;         // [x, y, z]  gravity vector
float frIMUypr[3] = {0, 0, 0};    // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
#endif

//PID Vars
double pitchSetpoint = 0 , camIMUpitch, frIMUpitch, pitchOutput, pitchErrorSum = 0;
double rollSetpoint = 0, camIMUroll, frIMUroll, rollOutput, rollErrorSum = 0;
double pitchKp, pitchKi, pitchKd;
double rollKp, rollKi, rollKd;
double sampleFreq;
double sampleTime;
byte PIDbound;

//rest of vars
char pitchDir = 1, rollDir = 1;
String serialStr;
unsigned long pitchIntPeriod = 10;  //initial pitch int period in msec
unsigned long rollIntPeriod = 10;   //initial roll int period in msec
bool ledState = false;
bool intTriggered = false;
byte ledCount = 0;
unsigned int batCount = 0;
unsigned int LedBlinkTime = 100; //1000msec, based on 100Hz interrupt

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  //reserve memory for variable length serial commands
  serialStr.reserve(40);

  //FAST PWM
#ifdef FAST_PWM
  TCCR3A = _BV(WGM30);
  TCCR4A = _BV(WGM40);
#endif

  //PhaseCorrect PWM
#ifdef PHASE_PWM
  TCCR3A = _BV(WGM30);
  TCCR3B = _BV(WGM32);
  TCCR4A = _BV(WGM40);
  TCCR4B = _BV(WGM42);
#endif

  //Set Timer 3&4 prescalers to 0 || motor PWM frequency = 31372.55 Hz
  TCCR3B = TCCR3B & 0b11111000 | 0x01;
  TCCR4B = TCCR4B & 0b11111000 | 0x01;

  //pin mode declarations
  pinMode(ROLLOUT1, OUTPUT);
  pinMode(ROLLOUT2, OUTPUT);
  pinMode(ROLLOUT3, OUTPUT);
  pinMode(PITCHOUT1, OUTPUT);
  pinMode(PITCHOUT2, OUTPUT);
  pinMode(PITCHOUT3, OUTPUT);
  pinMode(ENROLL, OUTPUT);
  pinMode(ENPITCH, OUTPUT);
  pinMode(ARMSW, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

#ifdef SERIAL_ENABLED
  // initialize serial communication
  Serial.begin(115200);
  Serial.println("Ready!");
#endif

  //setup Timer2 for 100Hz interrupt
  //  TCCR2A = 0;
  TCCR2A = _BV(WGM21);
  //  TCCR2B = 0;
  TCCR2B = TCCR2B & 0b11111000 | 0x07;
  TCNT2 = 0;
  OCR2A = 155; // = (16*10^6)/(1024*Hz)-1 < 256
  TIMSK2 = _BV(OCIE2A);

  //attach arm switch interrupt
  attachInterrupt(digitalPinToInterrupt(ARMSW), switched, CHANGE);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock.

  //initialise IMUs
#ifdef CAM_MPU
  cameraIMUsetup();
#endif
#ifdef FR_MPU
  frameIMUsetup();
#endif

  //disarm motors
  disarm();

  //initialize motor interrupt timers
  Timer1.initialize(rollIntPeriod * 1000);
  Timer5.initialize(pitchIntPeriod * 1000);

  //PID initialization
  eeprom_get();
  sampleFreq = 100;
  sampleTime = 1 / sampleFreq;

#ifdef SERIAL_ENABLED
  //printout Serial commands, wait 4sec for user to read
  serialCommandsPrint();
  delay(4000);
#endif

  //get initial Vbat
  voltage();

  //reset IMU FIFOs for clean start
#ifdef CAM_MPU
  camIMU.resetFIFO();
#endif
#ifdef FR_MPU
  frIMU.resetFIFO();
#endif
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  // if IMU programming failed, don't try to do anything
#ifdef CAM_MPU
  if (!camIMUReady) return;  //halt if MPU is missing
#endif
#ifdef FR_MPU
  if (!frIMUReady) return;  //halt if MPU is missing
#endif


  if (intTriggered) {
    //get IMU data, calculate PIDs
#ifdef CAM_MPU
    cameraIMUdata();
    PIDcalc(camIMUpitch, pitchSetpoint, &pitchOutput, camIMUroll, rollSetpoint, &rollOutput);
#endif
#ifdef FR_MPU
    frameIMUdata();
    PIDcalc(frIMUpitch, pitchSetpoint, &pitchOutput, frIMUroll, rollSetpoint, &rollOutput);
#endif

    //decode PID outputs into motor driving signals
    pitchOutDecode(pitchOutput);
    rollOutDecode(rollOutput);

#ifdef DATAOUT
    //serial print IMU data
    Serial.print(camIMUpitch);
    Serial.print("\t");
    Serial.print(pitchOutput);
    Serial.print("\t");
    Serial.print(frIMUpitch);
    Serial.print("\tP-R\t");
    Serial.print(camIMUroll);
    Serial.print("\t");
    Serial.print(rollOutput);
    Serial.print("\t");
    Serial.println(frIMUroll);
#endif

    //increment status counters
    ledCount++;
    batCount++;

    intTriggered = false;
  }

  //LED blink routine
  if (ledCount >= LedBlinkTime) {
    ledState = !ledState;
    digitalWrite(LED, ledState);
    ledCount = 0;
  }

  //check Vbat
  if (batCount >= VBATCHECKTIME) {
    voltage();
    batCount = 0;
  }
}

//Timer2 interrupt routine
ISR(TIMER2_COMPA_vect) {
  intTriggered = true;
}
