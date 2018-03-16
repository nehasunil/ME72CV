#include <RoboClaw.h>

#include <XBOXRECV.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

#include <Servo.h>
#include <Wire.h>

const int SERVOPIN = 9;
const int MOTORPIN = 1;
const int SOLENOIDPIN = 6;
const int CTRL_ID = 0;
const int TSENS[4] = {2.1,4,1.5,1.1};
const long STICKTHRESH = 7500; // lowest analog stick value to register as input
const int MAXLSTICK = 32768;

SoftwareSerial serial(0, MOTORPIN);  
RoboClaw roboclaw(&serial,10000);
USB Usb;
XBOXRECV Xbox(&Usb);
Servo steerServo;

#define address 0x80
#define SLAVE_ADDRESS 0x04

bool CTRL_CONNECTED = false;
bool RECV_CONNECTED = false;
bool BCKWD_VALON = false;
bool FWD_VALON = false;
bool GUN_VALON = false;
bool AUTO_STOP;
int VEL; // 
int BCKWD_VAL; // left trigger value
int FWD_VAL; // right trigger value
int GUN_VAL;
int SENSBIN = 0; 

float LSTICKSTEER;
int STEERPOS; // Position of Steering Servo
int SERVOZERO = 68;
int STEERSERVORANGE = 30;

int AUTO_VEL = 45;
volatile int servo_pi;

// COMMENT OUT IF USING TRIGS
float LSTICKFWD;

void setup() {
  //Serial.begin(38400);
  roboclaw.begin(115200);
  Wire.begin(SLAVE_ADDRESS);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
  steerServo.attach(SERVOPIN);
  STEERPOS = SERVOZERO;
  AUTO_STOP = true;
  Wire.onReceive(receiveData);
  servo_pi = 100;
  pinMode(SOLENOIDPIN,OUTPUT);
}

void loop() {

  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    // Receiver connected. 
    RECV_CONNECTED = true;
    if (Xbox.Xbox360Connected[CTRL_ID]) {
      // Controller connected.
      CTRL_CONNECTED = true;

      if (Xbox.getAnalogHat(LeftHatY, CTRL_ID) > STICKTHRESH || Xbox.getAnalogHat(LeftHatY, CTRL_ID) < -STICKTHRESH) {
        LSTICKFWD = (float) Xbox.getAnalogHat(LeftHatY, CTRL_ID)/MAXLSTICK;
        FWD_VAL = LSTICKFWD * (126);
      }
      else {
        FWD_VAL = 0;
      }

      
      // GUN
      if (Xbox.getButtonPress(L2, CTRL_ID)) {
        // If there's a RT value, set it accordingly for gun. 
        GUN_VALON = true;
        GUN_VAL = 126;
      } else if (GUN_VALON) {
        // If no RT value, set it to zero. 
        GUN_VALON = false; 
        GUN_VAL = 0; 
      }

      if (Xbox.getButtonPress(R2, CTRL_ID)) { 
        digitalWrite(SOLENOIDPIN, HIGH);
        delay(5);
        digitalWrite(SOLENOIDPIN, LOW);
      } else { 
        digitalWrite(SOLENOIDPIN, LOW);
      }

      // Emergency stop
      if (Xbox.getButtonClick(A, CTRL_ID)) { 
        if (AUTO_STOP) {
          AUTO_STOP = false;
        } else {
          AUTO_STOP = true;
        }
      }
      
      if (Xbox.getButtonClick(R1, CTRL_ID)) {
        SENSBIN++;
        SENSBIN = min( SENSBIN, 3 ); // cap multiplier at 3
        //AUTO_STOP = false;
      }
      if (Xbox.getButtonClick(L1, CTRL_ID)) {
        SENSBIN--;
        SENSBIN = max( SENSBIN, 0 ); // floor multiplier at 0
        AUTO_STOP = true;
      }


      // DISPLAY SENSERATE: Display sensrate with controller LED's
      if (SENSBIN == 0) Xbox.setLedOn(LED1, CTRL_ID);
      else if (SENSBIN == 1) Xbox.setLedOn(LED2, CTRL_ID);
      else if (SENSBIN == 2) Xbox.setLedOn(LED3, CTRL_ID);
      else if (SENSBIN == 3) Xbox.setLedOn(LED4, CTRL_ID);

      // In case nothing happens... 
      LSTICKSTEER = 0; 
      
      // Direction: Left analog stick X-axis
      if (Xbox.getAnalogHat(LeftHatX, CTRL_ID) > STICKTHRESH || Xbox.getAnalogHat(LeftHatX, CTRL_ID) < -STICKTHRESH) {
        LSTICKSTEER = (float) Xbox.getAnalogHat(LeftHatX, CTRL_ID)*STEERSERVORANGE/MAXLSTICK;
      }
      else if (!AUTO_STOP){
        LSTICKSTEER = -(servo_pi - 100);
      }
      else {
        LSTICKSTEER = 0;
        //Serial.println("here");
      }

      
    } else if(CTRL_CONNECTED) { 
      // Controller disconnected. Stop motor. 
      CTRL_CONNECTED = false;
      FWD_VAL = 0;
      BCKWD_VAL = 0;
    }
  }
  else if (RECV_CONNECTED) {
    // Receiver disconnected. Stop motor.
    RECV_CONNECTED = false;
    FWD_VAL = 0; 
    BCKWD_VAL = 0; 
  }

  // 
  VEL = (FWD_VAL - BCKWD_VAL)/TSENS[SENSBIN];
  
    if (!AUTO_STOP){
      roboclaw.ForwardM2(address, AUTO_VEL);
    }
    else if (VEL >= 0) {
      roboclaw.BackwardM2(address, VEL); 
    }
    else{
      roboclaw.ForwardM2(address, -VEL);
    }

  roboclaw.ForwardM1(address, GUN_VAL);
  

  
  STEERPOS = SERVOZERO - LSTICKSTEER;
  steerServo.write(STEERPOS);

 
  //delay(5);
}

void receiveData(int byteCount){
  while(Wire.available()) {
    int temp1 = Wire.read();
    int temp2 = Wire.read(); // only returns 0s unless filtered out, temp 1 is always 100
    
    if (temp1 != 0 && temp1 != 100) {
      servo_pi = temp1;
      //Serial.print("data received TEMP1: ");
      //Serial.println(servo_pi);                                                                                                                                                                                                                                            
    }
    else if (temp2 != 0) {
      servo_pi = temp2;
      //Serial.print("data received: ");
      //Serial.println(servo_pi);
    }
  }
}
