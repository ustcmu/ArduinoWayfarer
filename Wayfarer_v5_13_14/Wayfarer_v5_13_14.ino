


// Recieves serial input to signal user which direction to travel

/*---------------- Includes ---------------------------------*/
#include <Wire.h>
#include <LSM303.h>
#include <Timers.h>
#include <ble_mini.h>

LSM303 compass;

/*---------------- Module Defines ---------------------------*/
#define   SIGNALTIMER      1      //timer to trigger vibe signals
int signalDuration =       140;

#define   CLICKTIMER       2
#define   CLICK_DURATION   1000

#define   REPEATTIMER      3
//signalRepeatTime

#define   BATTERYTIMER     4
#define   BATTERY_DURATION   1000

//Define pins for haptic signals for the 8 directions
#define   FRONTRIGHT_MOTOR 3
#define   FRONTLEFT_MOTOR  9
#define   BACKRIGHT_MOTOR  6
#define   BACKLEFT_MOTOR   5
#define   EN_FRONT_MOTOR   4
#define   EN_BACK_MOTOR    7

//Define pins for the 2 LED colors
#define   LED_PIN_GREEN    12
#define   LED_PIN_RED      13
#define   LED_BRIGHTNESS   150

//Define patterns for the different directions
#define   FORWARD_PATTERN    1
#define   FRONTRIGHT_PATTERN 2
#define   RIGHT_PATTERN      3
#define   BACKRIGHT_PATTERN  4
#define   BACK_PATTERN       5
#define   BACKLEFT_PATTERN   6
#define   LEFT_PATTERN       7
#define   FRONTLEFT_PATTERN  8
#define   ARRIVED_PATTERN    9        //All motors pulse on and off for 2 seconds
#define   BOOTUP_PATTERN     10       //All motors slow fade up
#define   WAYPOINT_PATTERN   11       //All motors two quck pulses
#define   START_PATTERN      12       //Two quick circuits
#define   PAUSE_PATTERN      13       //All motors slow fade down
#define   TROUBLE_PATTERN    14       //Continuous circling for 5 seconds
#define   ARVD              -500
#define   BTUP              -600
#define   WYPNT             -700
#define   START             -800
#define   PAUSE             -900
#define   TRBL              -950

//Define states for main loop
#define   SIGNAL_STATE       1
#define   SEARCH_STATE       2

/*---------------- Module Function Prototypes ---------------*/

/*---------------- Module Level Variables -------------------*/
// Used to read in from the serial connection
int strNum = 0;          //delimeter to parse the input into mode and angle
int mode = 0;            //mode to store whether the phone has accurate heading data
String angleString;

// Used to define the pattern of vibe motor signals
int signalLevel = 0;     //pwm value to write current motor
int signalNum = 1;       //current index of sequence pattern
int signalRepeatTime;    //how long to play the signal
int repeatNum;           //Number of times to repeat the signal
boolean FL = false;      //frontleft motor on state
boolean FR = false;      //frontright motor on state
boolean BL = false;      //backleft motor on state
boolean BR = false;      //backright motor on state

static int state = SIGNAL_STATE;    //used when playing a pattern to keep track of the sequence
static int SignalPattern;   //used to store the currently playing pattern

boolean batteryOK = true;

//boolean BLE = false;
boolean BLE = true;

/*---------------- Arduino Main Functions -------------------*/
void setup()
{ 
  if(BLE) BLEMini_begin(57600);
  else {
    Serial.begin(9600);
    Serial.println("...initializing...");
  }
  
  delay(1000);

  Wire.begin();      //For compass communications
  compass.init();
  compass.enableDefault();
  // Calibration values. Use the Calibrate example program to get the values for
  // your compass.
  compass.m_min.x = -581; compass.m_min.y = -731; compass.m_min.z = -1097;
  compass.m_max.x = +615; compass.m_max.y = +470; compass.m_max.z = 505;
  delay(1000);
  
  // Phone sends declination information along with 
  
  pinMode(LED_PIN_RED, OUTPUT);
  
  pinMode(FRONTRIGHT_MOTOR, OUTPUT);
  pinMode(FRONTLEFT_MOTOR, OUTPUT);
  pinMode(BACKRIGHT_MOTOR, OUTPUT);
  pinMode(BACKLEFT_MOTOR, OUTPUT);
  pinMode(EN_FRONT_MOTOR, OUTPUT);
  pinMode(EN_BACK_MOTOR, OUTPUT);
  digitalWrite(EN_FRONT_MOTOR, HIGH);
  digitalWrite(EN_BACK_MOTOR, HIGH);
  
  TMRArd_InitTimer(SIGNALTIMER, signalDuration);    //Start signal timer
  TMRArd_InitTimer(CLICKTIMER, CLICK_DURATION);
  TMRArd_InitTimer(REPEATTIMER, signalRepeatTime);    //Start the repeat timer
  TMRArd_InitTimer(BATTERYTIMER, BATTERY_DURATION);  //Battery timer for battery status checks
  
//  if(compass.enableDoubleClick()) {Serial.println("Ready");}
  
  choosePattern(BTUP);  //bootup pattern
}

void loop()  {
  while (BLEMini_available() || Serial.available() > 0) {              // If the bluetooth sent any characters
    char serialBuffer;
    if(BLE) serialBuffer = BLEMini_read();
    else {
      serialBuffer = Serial.read();
//      Serial.println(serialBuffer);
    }
    
    if (serialBuffer == '#') {strNum++;continue;}
    if (strNum == 1) {
      mode = serialBuffer - '0';                          //convert char to int
      if(mode < 1 || mode > 8) mode = 0;   //ignore extraneous characters
      digitalWrite(LED_PIN_RED, HIGH);    //turn on led to signify serial data  
    }
    if (strNum == 2) {
      angleString += serialBuffer;    //add chars to angle string
    }
  }
  if (strNum == 3) {
    int angleIn = angleString.toInt();    //convert angle string to int
//    Serial.print("Mode: "); Serial.println(mode); Serial.print("Angle: "); Serial.println(angleIn);
    if(mode == 1)      choosePattern(angleIn);                //phone provided correct target angle
    else if(mode == 2) calcHeadingDirection(angleIn);  //phone provided target angle but doesn't know which way user is facing
    else if(mode == 3) choosePattern(ARVD);  //user has arrived at destination
    else if(mode == 4) choosePattern(WYPNT);  //user has arrived at waypoint
    else if(mode == 5) choosePattern(START);  //user has started navigation
    else if(mode == 6) choosePattern(PAUSE);  //user has paused navigation
    else if(mode == 7) choosePattern(TRBL);  //user has paused navigation
    while(Serial.available()>0) Serial.read();    //flush serial buffer
    while (BLEMini_available()) BLEMini_read();
    mode = 0;                      //housekeeping: reset variables for next serial
    strNum = 0;
    angleString = "";
    digitalWrite(LED_PIN_RED, LOW);        //turn off led to show serial empty
  }
  
  // Housekeeping: turn off battery status led after timer expires
  if(TMRArd_IsTimerExpired(BATTERYTIMER) == 1) {
      TMRArd_ClearTimerExpired(BATTERYTIMER);
      digitalWrite(LED_PIN_GREEN, LOW);
      digitalWrite(LED_PIN_RED, LOW);
  }
  
  switch(state) {
    case(SEARCH_STATE):      //searching for user input
    if(compass.checkClick() == 12 && TMRArd_IsTimerExpired(CLICKTIMER)) respondToClick();    //timer limits the number of inputs per sec
    break;
    
    case(SIGNAL_STATE):
    if(TMRArd_IsTimerExpired(REPEATTIMER) == 1 || repeatNum == 0) {    //exit signaling loop (back to search)
      TMRArd_ClearTimerExpired(REPEATTIMER);
      state = SEARCH_STATE;
      repeatNum = 0;
      allMotorsOff();        //turn off all motors
    }
    else if(TMRArd_IsTimerExpired(SIGNALTIMER) == 1) {      //counts time between notes in pattern
      TMRArd_ClearTimerExpired(SIGNALTIMER);
      getCurrMotors();
      if(FL) analogWrite(FRONTLEFT_MOTOR, signalLevel);    // If FL == 1, turn motor on to signal level specified
      else(digitalWrite(FRONTLEFT_MOTOR, LOW));
      if(FR) analogWrite(FRONTRIGHT_MOTOR, signalLevel);    // If signal level offset needed, subtract it from signalLevel here
      else(digitalWrite(FRONTRIGHT_MOTOR, LOW));
      if(BL) analogWrite(BACKLEFT_MOTOR, signalLevel);
      else(digitalWrite(BACKLEFT_MOTOR, LOW));
      if(BR) analogWrite(BACKRIGHT_MOTOR, signalLevel);
      else(digitalWrite(BACKRIGHT_MOTOR, LOW));
      signalNum++;
      TMRArd_InitTimer(SIGNALTIMER, signalDuration);
    }break;
  }
}

void allMotorsOff() {
  if(!BLE) Serial.println("Stop");
  digitalWrite(FRONTLEFT_MOTOR, LOW);
  digitalWrite(FRONTRIGHT_MOTOR, LOW);
  digitalWrite(BACKLEFT_MOTOR, LOW);
  digitalWrite(BACKRIGHT_MOTOR, LOW);
}

void respondToClick(void) {
  TMRArd_ClearTimerExpired(CLICKTIMER);
  unsigned char dat = 666;
  readVcc();
  if(batteryOK) digitalWrite(LED_PIN_GREEN, LED_BRIGHTNESS);
  if(!batteryOK) digitalWrite(LED_PIN_RED, LED_BRIGHTNESS);
  BLEMini_write(dat);
  if(!BLE) Serial.println("Click");
  TMRArd_InitTimer(CLICKTIMER, CLICK_DURATION);
  TMRArd_InitTimer(BATTERYTIMER, BATTERY_DURATION);
}

void getCurrMotors(void) {
  switch(SignalPattern) {
    case(FORWARD_PATTERN):
      switch(signalNum) {
        case(1):
        if(true) {
          FL = FR = 1;        //which motors are on
          BL = BR = 0;
          signalLevel = 180;  //level for all motors
          signalDuration = 150;  //length of this beat
        }
        break;
          case(2):
        if(true) {
          FL = FR = 0;        //which motors are on
          BL = BR = 0;
          signalLevel = 255;  //level for all motors
          signalDuration = 100;  //length of this beat
        }
        break; 
         case(3):
        if(true) {
          FL = FR = 1;        //which motors are on
          BL = BR = 0;
          signalLevel = 210;  //level for all motors
          signalDuration = 200;  //length of this beat
        }
        break;
          case(4):
        if(true) {
          FL = FR = 0;        //which motors are on
          BL = BR = 0;
          signalLevel = 255;  //level for all motors
          signalDuration = 100;  //length of this beat
        }
        break;
        case(5):
        if(true) {
          motorPulse(FR, FL, 2);
          repeatPattern();
        }break;
      }break;
    
    case(FRONTRIGHT_PATTERN):
    switch(signalNum){
      case(1):
      if(true) {
          BL = 1; FL = FR = BR = 0; signalLevel = 150; 
          signalDuration = 150;
        }break;
       case(2):
       if(true) {
          FL = 1; FR = BL = BR = 0; signalLevel = 175; 
          signalDuration = 175;
        }break;
        case(3):
        if(true) {
          FR = 1; FL = BL = BR = 0; signalLevel = 255; 
          signalDuration = 300;         
        }break;
        case(4):
        if(true) {
          FR = 0; FL = BL = BR = 0; signalLevel = 255; 
          signalDuration = 100;
        }break;
        case(5):
         if(true) {
          motorPulse(FR, -1, 2);
          }break;
          case(6):
        if(true) {
          FR = 0; FL = BL = BR = 0; signalLevel = 255; 
          signalDuration = 100;  
          repeatPattern();                   
        }break;
    }break;
    
    case(RIGHT_PATTERN):
    switch(signalNum) {
      case(1):
      if(true) {
          BL = 1; FL = FR = BR = 0; signalLevel = 150; 
          signalDuration = 150;
        }break;
       case(2):
       if(true) {
          FL = 1; FR = BL = BR = 0; signalLevel = 175; 
          signalDuration = 175;
        }break;
        case(3):
        if(true) {
          FR = 1; FL = BL = BR = 0; signalLevel = 200; 
          signalDuration = 200;         
        }break;
        case(4):
        if(true) {
          FR = BR = 1; FL = BL = 0; signalLevel = 255; 
          signalDuration = 300;         
        }break;
        case(5):
        if(true) {
          FR = FL = BL = BR = 0; signalLevel = 255; 
          signalDuration = 75;
        }break;
          case(6):
          if(true) {
          motorPulse(FR, BR, 2);
          repeatPattern();            
        }break;
      }break;
      
    case(BACKRIGHT_PATTERN):
    switch(signalNum){
      case(1):
      if(true) {
          FL = 1; BL = FR = BR = 0; signalLevel = 255; 
          signalDuration = 150;
        }break;
       case(2):
       if(true) {
          FR = 1; FL = BL = BR = 0; signalLevel = 255; 
          signalDuration = 200;
        }break;
        case(3):
        if(true) {
          BR = 1; FL = BL = FR = 0; signalLevel = 255; 
          signalDuration = 300;         
        }break;
        case(4):
        if(true) {
          BR = 0; FL = BL = FR = 0; signalLevel = 255; 
          signalDuration = 100;
        }break;
        case(5):
        if(true) {
          BR = 1; FL = BL = FR = 0; signalLevel = 255; 
          signalDuration = 300; 
          }break;
          case(6):
        if(true) {
          BR = 0; FL = BL = FR = 0; signalLevel = 255; 
          signalDuration = 100;  
          repeatPattern();                   
        }break;
    }break;
      
    case(BACK_PATTERN):
    switch(signalNum) {
        case(1):
        if(true) {
          FL = FR = 0;        //which motors are on
          BL = BR = 1;
          signalLevel = 180;  //level for all motors
          signalDuration = 175;  //length of this beat
        }
        break;
          case(2):
        if(true) {
          FL = FR = 0;        //which motors are on
          BL = BR = 0;
          signalLevel = 255;  //level for all motors
          signalDuration = 100;  //length of this beat
        }
        break; 
         case(3):
        if(true) {
          FL = FR = 0;        //which motors are on
          BL = BR = 1;
          signalLevel = 210;  //level for all motors
          signalDuration = 200;  //length of this beat
        }
        break;
          case(4):
        if(true) {
          FL = FR = 0;        //which motors are on
          BL = BR = 0;
          signalLevel = 255;  //level for all motors
          signalDuration = 100;  //length of this beat
        }
        break;
        case(5):
        if(true) {
          motorPulse(BR, BL, 2);
          repeatPattern();
        }break;
      }break;

      
    case(BACKLEFT_PATTERN):
    switch(signalNum){
      case(1):
      if(true) {
          FR = 1; BL = FL = BR = 0; signalLevel = 255; 
          signalDuration = 150;
        }break;
       case(2):
       if(true) {
          BR = 1; FL = BL = FR = 0; signalLevel = 255; 
          signalDuration = 200;
        }break;
        case(3):
        if(true) {
          BL = 1; BR = FL = FR = 0; signalLevel = 255; 
          signalDuration = 300;         
        }break;
        case(4):
        if(true) {
          BR = 0; FL = BL = FR = 0; signalLevel = 255; 
          signalDuration = 100;
        }break;
        case(5):
        if(true) {
           BL= 1; FL = BR = FR = 0; signalLevel = 255; 
          signalDuration = 300; 
           }break;
        case(6):
        if(true) {
          BR = 0; FL = BL = FR = 0; signalLevel = 255; 
          signalDuration = 100;  
          repeatPattern();                   
        }break;
    }break;

    case(LEFT_PATTERN):
   switch(signalNum) {
      case(1):
      if(true) {
          BR = 1; FL = FR = BL = 0; signalLevel = 150; 
          signalDuration = 150;
        }break;
       case(2):
       if(true) {
          FR = 1; FL = BL = BR = 0; signalLevel = 175; 
          signalDuration = 175;
        }break;
        case(3):
        if(true) {
          FL = 1; FR = BL = BR = 0; signalLevel = 200; 
          signalDuration = 200;         
        }break;
        case(4):
        if(true) {
          FL = BL = 1; FR = BR = 0; signalLevel = 255; 
          signalDuration = 300;         
        }break;
        case(5):
        if(true) {
          FR  = FL = BL = BR = 0; signalLevel = 255; 
          signalDuration = 75;
        }break;
        case(6):
         if(true) {
          motorPulse(FL, BL, 2);
          repeatPattern();            
        }break;
      }break;

    
    case(FRONTLEFT_PATTERN):
    switch(signalNum){
      case(1):
      if(true) {
          BR = 1; FL = FR = BL = 0; signalLevel = 150; 
          signalDuration = 150;
        }break;
       case(2):
       if(true) {
          FR = 1; FL = BL = BR = 0; signalLevel = 175; 
          signalDuration = 175;
        }break;
        case(3):
        if(true) {
          FL = 1; FR = BL = BR = 0; signalLevel = 255; 
          signalDuration = 300;         
        }break;
        case(4):
        if(true) {
          FR = FL = BL = BR = 0; signalLevel = 255; 
          signalDuration = 100;
        }break;
        case(5):
        if(true) {
          motorPulse(FL, -1, 2);
          }break;
          case(6):
        if(true) {
          FR = FL = BL = BR = 0; signalLevel = 255; 
          signalDuration = 100;  
          repeatPattern();                   
        }break;
    }break;
    
    case(ARRIVED_PATTERN):
    switch(signalNum) {
        case(1):
        if(true) {
          FL = FR = 1; 
          BL = BR = 1; 
          signalLevel = 255; 
          signalDuration = 200;
        }break;
        case(2):
        if(true) {
          FL = FR = BL = BR = 0; 
          signalLevel = 255; 
          signalDuration = 200;
          repeatPattern();
        }break;
    }break;
    
    case(BOOTUP_PATTERN):
    switch(signalNum) {
        case(1):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel = 15; 
          signalDuration = 200;
        }break;
        case(2):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel += 30; 
          signalDuration = 200;
        }break;
        case(3):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel += 75; 
          signalDuration = 200;
        }break;
        case(4):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel += 75; 
          signalDuration = 200;
        }break;
        case(5):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel += 75; 
          signalDuration = 200;
        }break;
        case(6):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel += 75; 
          signalDuration = 200;
        }break;
        case(7):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel += 75; 
          signalDuration = 200;
        }break;
        case(8):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel += 75; 
          signalDuration = 200;
        }break;
        case(9):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel += 75; 
          signalDuration = 200;
          repeatPattern();
        }break;
    }break;
    
    case(PAUSE_PATTERN):
    switch(signalNum) {
        case(1):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel = 255; 
          signalDuration = 300;
        }break;
        case(2):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel -= 30; 
          signalDuration = 300;
        }break;
        case(3):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel -= 75; 
          signalDuration = 300;
        }break;
        case(4):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel -= 75; 
          signalDuration = 300;
        }break;
        case(5):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel -= 75; 
          signalDuration = 300;
        }break;
        case(6):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel -= 75; 
          signalDuration = 300;
        }break;
        case(7):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel -= 75; 
          signalDuration = 300;
        }break;
        case(8):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel -= 75; 
          signalDuration = 300;
        }break;
        case(9):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel -= 75; 
          signalDuration = 300;
          repeatPattern();
        }break;
    }break;
    
    case(START_PATTERN):
    switch(signalNum) {
        case(1):
        if(true) {
          FL = 1; 
          FR = BL = BR = 0; 
          signalLevel = 255; 
          signalDuration = 100;
        }break;
        case(2):
        if(true) {
          FR = 1; 
          FL = BL = BR = 0; 
          signalLevel = 255; 
          signalDuration = 100;
        }break;
        case(3):
        if(true) {
          BR = 1; 
          FL = FR = BL = 0; 
          signalLevel = 255; 
          signalDuration = 100;
        }break;
        case(4):
        if(true) {
          BL = 1; 
          FL = FR = BR = 0; 
          signalLevel = 255; 
          signalDuration = 100;
          repeatPattern();
        }break;
    }break;
    
    case(TROUBLE_PATTERN):
    switch(signalNum) {
        case(1):
        if(true) {
          FL = 1; 
          FR = BL = BR = 0; 
          signalLevel = 255; 
          signalDuration = 200;
        }break;
        case(2):
        if(true) {
          FR = 1; 
          FL = BL = BR = 0; 
          signalLevel = 255; 
          signalDuration = 200;
        }break;
        case(3):
        if(true) {
          BR = 1; 
          FL = FR = BL = 0; 
          signalLevel = 255; 
          signalDuration = 200;
        }break;
        case(4):
        if(true) {
          BL = 1; 
          FL = FR = BR = 0; 
          signalLevel = 255; 
          signalDuration = 200;
          repeatPattern();
        }break;
    }break;
    
    case(WAYPOINT_PATTERN):
    switch(signalNum) {
        case(1):
        if(true) {
          FL = FR = BL = BR = 1;
          signalLevel = 255; 
          signalDuration = 170;
        }break;
        case(2):
        if(true) {
          FL = FR = BL = BR = 0; 
          signalLevel = 255; 
          signalDuration = 400;
        }break;
        case(3):
        if(true) {
          FL = FR = BL = BR = 1; 
          signalLevel = 255; 
          signalDuration = 170;
        }break;
        case(4):
        if(true) {
          FL = FR = BL = BR = 0; 
          signalLevel = 255; signalDuration = 400;
          repeatPattern();      
        }break;
    }break;
  }
}


void motorPulse(int pin1, int pin2, int numCycles){
  // fade in from min to max in increments of 5 points:
  for(int i=0; i<numCycles; i++){
        
  
    for(int fadeValue = 0 ; fadeValue <= 255; fadeValue +=5) { 
    // sets the value (range from 0 to 255):
      analogWrite(pin1, fadeValue);  
      
      if(pin2!=-1)analogWrite(pin2, fadeValue);         
       
    // wait for 30 milliseconds to see the dimming effect    
      delay(30);                            
    } 

  // fade out from max to min in increments of 5 points:
    for(int fadeValue = 255 ; fadeValue >= 0; fadeValue -=5) { 
    // sets the value (range from 0 to 255):
      analogWrite(pin1, fadeValue);        
      analogWrite(pin2, fadeValue);         
 
    // wait for 30 milliseconds to see the dimming effect    
      delay(30);                            
    }
  } 
}

void choosePattern(int targetHeading) {
  state = SIGNAL_STATE;
  signalDuration = 50;
  TMRArd_InitTimer(SIGNALTIMER, signalDuration);
  if ((targetHeading >= 337.5 || targetHeading < 22.5) && targetHeading >= 0) {
    if(!BLE) Serial.println("F");
    SignalPattern = FORWARD_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 10000);
    repeatNum = 4;
    return;
  }
  if (targetHeading >= 22.5 && targetHeading < 67.5) {
    if(!BLE) Serial.println("FR");
    SignalPattern = FRONTRIGHT_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 10000);
    repeatNum = 4;
    return;
  }
  if (targetHeading >= 67.5 && targetHeading < 112.5) {
    if(!BLE) Serial.println("R");
    SignalPattern = RIGHT_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 10000);
    repeatNum = 4;
    return;
  }
  if (targetHeading >= 112.5 && targetHeading < 157.5) {
    if(!BLE) Serial.println("BR");
    SignalPattern = BACK_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 10000);
    repeatNum = 4;
    return;
  }
  if (targetHeading >= 157.5 && targetHeading < 202.5) {
    if(!BLE) Serial.println("B");
    SignalPattern = BACK_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 10000);
    repeatNum = 4;
    return;
  }
  if (targetHeading >= 202.5 && targetHeading < 247.5) {
    if(!BLE) Serial.println("BL");
    SignalPattern = BACK_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 10000);
    repeatNum = 4;
    return;
  }
  if (targetHeading >= 247.5 && targetHeading < 292.5) {
    if(!BLE) Serial.println("L");
    SignalPattern = LEFT_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 10000);
    repeatNum = 4;
    return;
  }
  if (targetHeading >= 292.5 && targetHeading < 337.5) {
    if(!BLE) Serial.println("FL");
    SignalPattern = FRONTLEFT_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 10000);
    repeatNum = 4;
    return;
  }
  if (targetHeading == BTUP) {
    if(!BLE) Serial.println("Btup");
    SignalPattern = BOOTUP_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 20000);
    repeatNum = 1;
    return;
  }
  if (targetHeading == ARVD) {
    if(!BLE) Serial.println("Arvd");
    SignalPattern = ARRIVED_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 10000);
    repeatNum = 4000;
    return;
  }
  if (targetHeading == WYPNT) {
    if(!BLE) Serial.println("Wypnt");
    SignalPattern = WAYPOINT_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 10000);
    repeatNum = 1;
    return;
  }
  if (targetHeading == START) {
    if(!BLE) Serial.println("Start");
    SignalPattern = START_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 10000);
    repeatNum = 3;
    return;
  }
  if (targetHeading == PAUSE) {
    if(!BLE) Serial.println("Pause");
    SignalPattern = PAUSE_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 10000);
    repeatNum = 1;
    return;
  }
  if (targetHeading == TRBL) {
    if(!BLE) Serial.println("Trouble");
    SignalPattern = TROUBLE_PATTERN;
    TMRArd_InitTimer(REPEATTIMER, 10000);
    repeatNum = 30000;
    return;
  }
}

void repeatPattern() {
  signalNum = 0;
  repeatNum --;
  if(!BLE) Serial.print("Repeat");
}

void  calcHeadingDirection(int angleIn){
  compass.read();
  int currentHeading;
  currentHeading = compass.heading((LSM303::vector<int>){0,-1,0});
  if(!BLE) Serial.print("Heading: ");
  if(!BLE) Serial.println(currentHeading);
  float deltaHeading = angleIn - currentHeading;
  
  int targetHeading;
  if (deltaHeading > 0) {
    targetHeading = deltaHeading;
  }
  else {
    targetHeading = deltaHeading + 360;
  }
  
  if(!BLE) Serial.print("Target Heading: ");
  if(!BLE) Serial.println(targetHeading);
  choosePattern(targetHeading);
}

void readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  if(result < 3800) batteryOK = false;
  else batteryOK = true;
//  Serial.println(result);
}
