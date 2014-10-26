#include <AccelStepper.h>
#include <Makeblock.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Servo.h>

// Finite state machine

#define STATE_OFF 0
#define STATE_INITIATION 1
#define STATE_READY 2
#define STATE_GLUE_ARM_FWD 3
#define STATE_APPLY_GLUE_1 4
#define STATE_MOVE_TO_2 5
#define STATE_APPLY_GLUE_2 6
#define STATE_MOVE_TO_3 7
#define STATE_APPLY_GLUE_3 8
#define STATE_MOVE_TO_4 9
#define STATE_APPLY_GLUE_4 10
#define STATE_MOVE_TO_5 11
#define STATE_APPLY_GLUE_5 12
#define STATE_MOVE_TO_READY 13

int fsmState;

/* Carriage stepper */
const int CARRIAGE_STEPPER_PORT = PORT_3;
const int CARRIAGE_STEPPER_PULSE_LENGTH = 30;
const float CARRIAGE_STEPPER_ACCELERATION = 20000;
const float CARRIAGE_STEPPER_MAX_SPEED = 5000;

const int directionPinCarriageStepper = mePort[ CARRIAGE_STEPPER_PORT ].s1;//the direction pin connect to Base Board
const int stepPinCarriageStepper = mePort[ CARRIAGE_STEPPER_PORT ].s2;//the Step pin connect to Base Board
AccelStepper carriageStepper( AccelStepper::DRIVER, stepPinCarriageStepper, directionPinCarriageStepper );

/* Glue arm stepper */
const int GLUE_ARM_STEPPER_PORT = PORT_4;
const int GLUE_ARM_STEPPER_PULSE_LENGTH = 30;
const float GLUE_ARM_STEPPER_ACCELERATION = 20000;
const float GLUE_ARM_MAX_SPEED = 5000;
const float GLUE_ARM_APPLICATION_SPEED = 1000;

const int directionPinGlueArmStepper = mePort[ GLUE_ARM_STEPPER_PORT ].s1;//the direction pin connect to Base Board
const int stepPinGlueArmStepper = mePort[ GLUE_ARM_STEPPER_PORT ].s2;//the Step pin connect to Base Board
AccelStepper glueArmStepper( AccelStepper::DRIVER, stepPinGlueArmStepper, directionPinGlueArmStepper );

/* Glue applicator servo */
const int GLUE_INITIAL_POSITION = 45;
const int GLUE_APPLICATION_POSITION = 75;
const int GLUE_RAISED_POSITION = 45;

MePort glueServoPort(PORT_1);
Servo glueServoDriver;  // create servo object to control a servo 
int glueServoPin = glueServoPort.pin2(); //attaches the servo on PORT_1 SLOT1 to the servo object. NB: It's pin 2. 

// Button
const int BUTTON_PIN = 3; // Number of the button pin
int buttonState = 0;      // For reading the button status

int carriageOrigoSwitch = LOW;
int glueArmOrigoSwitch = LOW;
int button = LOW;

boolean isMovingOut = false;
boolean hasMovedOut = false;
boolean isMovingIn = false;
boolean hasMovedIn = false;
boolean carriageIsMoving = false;
boolean glueArmIsMoving = false;
boolean isApplyingGlue = false;

void setup() {
  // Initialize FSM
  fsmState = STATE_OFF;
  
  // Initialize communications
  Serial.begin( 9600 );
  
  // Initialize button
  pinMode( BUTTON_PIN, INPUT );    
  
  // Initialize carriage stepper
  carriageStepper.setMaxSpeed( CARRIAGE_STEPPER_MAX_SPEED );
  carriageStepper.setAcceleration( CARRIAGE_STEPPER_ACCELERATION );
  
  // Initialize glue arm stepper
  glueArmStepper.setMaxSpeed( 1000 );
  glueArmStepper.setAcceleration( 20000 );
  
  // Initialize glue applicator
  glueServoDriver.attach( glueServoPin );  // Attaches the servo on the servo pin
  glueServoDriver.write( GLUE_INITIAL_POSITION );
  delay( 500 );
}

void loop() {
  
  switch ( fsmState ) {
    case STATE_OFF:
      // Read the state of the button
      buttonState = digitalRead( BUTTON_PIN );
      if ( buttonState == HIGH ) {     
        fsmState = STATE_READY;  
      } 
    break;
      
    case STATE_INITIATION:
       //Serial.println( "INITIATION" );
      if ( carriageOrigoSwitch == LOW && carriageStepper.distanceToGo() == 0 && !isMovingOut && !isMovingIn && !hasMovedOut ) {
        isMovingOut = true;
        carriageStepper.move( 100 );
      } else if ( carriageOrigoSwitch == LOW && carriageStepper.distanceToGo() > 0 && isMovingOut ) {
        carriageStepper.run();
      } else if ( carriageOrigoSwitch == HIGH && isMovingOut ) {
        carriageStepper.stop();
        isMovingOut = false;
        hasMovedOut = true;
        carriageStepper.move( -100 );
        isMovingIn = true;
      } else if ( carriageOrigoSwitch == HIGH && carriageStepper.distanceToGo() > 0 && isMovingIn ) {
        carriageStepper.run();
      } else if ( carriageOrigoSwitch == LOW && isMovingIn ) {
        carriageStepper.stop();
        isMovingIn = false;
        hasMovedIn = true;
        carriageStepper.move( 0 );
        fsmState = STATE_READY;
      }
      break;
      
    case STATE_READY:
      carriageStepper.setMaxSpeed( CARRIAGE_STEPPER_MAX_SPEED );
      fsmState = STATE_GLUE_ARM_FWD;
      break;
     
   case STATE_GLUE_ARM_FWD:
     if ( !glueArmIsMoving ) {
       glueArmStepper.move( 1754 );
       glueArmStepper.setMaxSpeed( GLUE_ARM_MAX_SPEED );
       glueArmIsMoving = true;
     } else if ( glueArmIsMoving && glueArmStepper.distanceToGo() == 0 ) {
       glueArmIsMoving = false;
       fsmState = STATE_APPLY_GLUE_1;
     } else {
       glueArmStepper.run();      
     }
     break;
 
   case STATE_APPLY_GLUE_1:
     if ( !isApplyingGlue ) {
       glueServoDriver.write( GLUE_APPLICATION_POSITION );
       delay( 300 );
       isApplyingGlue = true;
     } else if ( isApplyingGlue ) {
        if ( !glueArmIsMoving ) {
          glueArmStepper.moveTo( 0);
          glueArmStepper.setMaxSpeed( GLUE_ARM_APPLICATION_SPEED );
          glueArmIsMoving = true;
        } else if ( glueArmIsMoving && glueArmStepper.distanceToGo() == 0 ) {
          glueArmIsMoving = false;
          glueServoDriver.write( GLUE_RAISED_POSITION );
          isApplyingGlue = false;
          fsmState = STATE_MOVE_TO_2;
        } else {
          glueArmStepper.run();
        }
     }
     break; 
     
   case STATE_MOVE_TO_2:
     if ( !carriageIsMoving && !glueArmIsMoving ) {
       carriageStepper.move( 1575 );
       carriageIsMoving = true;
       glueArmStepper.move( 1754 );
       glueArmStepper.setMaxSpeed( GLUE_ARM_MAX_SPEED );       
       glueArmIsMoving = true;
     } else if ( carriageIsMoving && carriageStepper.distanceToGo() == 0 && glueArmIsMoving && glueArmStepper.distanceToGo() == 0 ) {
       carriageIsMoving = false;
       glueArmIsMoving = false;
       fsmState = STATE_APPLY_GLUE_2; 
     } else {
       carriageStepper.run();
       glueArmStepper.run();
     }
     break;

   case STATE_APPLY_GLUE_2:
     if ( !isApplyingGlue ) {
       glueServoDriver.write( GLUE_APPLICATION_POSITION );
       delay( 300 );
       isApplyingGlue = true;
     } else if ( isApplyingGlue ) {
        if ( !glueArmIsMoving ) {
          glueArmStepper.moveTo( 0);
          glueArmStepper.setMaxSpeed( GLUE_ARM_APPLICATION_SPEED );          
          glueArmIsMoving = true;
        } else if ( glueArmIsMoving && glueArmStepper.distanceToGo() == 0 ) {
          glueArmIsMoving = false;
          glueServoDriver.write( GLUE_RAISED_POSITION );
          isApplyingGlue = false;
          fsmState = STATE_MOVE_TO_3;
        } else {
          glueArmStepper.run();
        }
     }
     break; 

     case STATE_MOVE_TO_3:
       if ( !carriageIsMoving && !glueArmIsMoving ) {
         carriageStepper.move( 1575 );
         carriageIsMoving = true;
         glueArmStepper.move( 1754 );
         glueArmStepper.setMaxSpeed( GLUE_ARM_MAX_SPEED );         
         glueArmIsMoving = true;
       } else if ( carriageIsMoving && carriageStepper.distanceToGo() == 0 && glueArmIsMoving && glueArmStepper.distanceToGo() == 0 ) {
         carriageIsMoving = false;
         glueArmIsMoving = false;
         fsmState = STATE_APPLY_GLUE_3; 
       } else {
         carriageStepper.run();
         glueArmStepper.run();
       }
       break;

   case STATE_APPLY_GLUE_3:
     if ( !isApplyingGlue ) {
       glueServoDriver.write( GLUE_APPLICATION_POSITION );
       delay( 300 );
       isApplyingGlue = true;
     } else if ( isApplyingGlue ) {
        if ( !glueArmIsMoving ) {
          glueArmStepper.moveTo( 0);
          glueArmStepper.setMaxSpeed( GLUE_ARM_APPLICATION_SPEED );
          glueArmIsMoving = true;
        } else if ( glueArmIsMoving && glueArmStepper.distanceToGo() == 0 ) {
          glueArmIsMoving = false;
          glueServoDriver.write( GLUE_RAISED_POSITION );
          isApplyingGlue = false;
          fsmState = STATE_MOVE_TO_4;
        } else {
          glueArmStepper.run();
        }
     }
     break; 

     case STATE_MOVE_TO_4:
       if ( !carriageIsMoving && !glueArmIsMoving ) {
         carriageStepper.move( 1575 );
         carriageIsMoving = true;
         glueArmStepper.move( 1754 );
         glueArmStepper.setMaxSpeed( GLUE_ARM_MAX_SPEED );       
         glueArmIsMoving = true;
       } else if ( carriageIsMoving && carriageStepper.distanceToGo() == 0 && glueArmIsMoving && glueArmStepper.distanceToGo() == 0 ) {
         carriageIsMoving = false;
         glueArmIsMoving = false;
         fsmState = STATE_APPLY_GLUE_4;
       } else {
         carriageStepper.run();
         glueArmStepper.run();
       }
       break;

   case STATE_APPLY_GLUE_4:
     if ( !isApplyingGlue ) {
       glueServoDriver.write( GLUE_APPLICATION_POSITION );
       delay( 300 );
       isApplyingGlue = true;
     } else if ( isApplyingGlue ) {
        if ( !glueArmIsMoving ) {
          glueArmStepper.moveTo( 0);
          glueArmStepper.setMaxSpeed( GLUE_ARM_APPLICATION_SPEED );          
          glueArmIsMoving = true;
        } else if ( glueArmIsMoving && glueArmStepper.distanceToGo() == 0 ) {
          glueArmIsMoving = false;
          glueServoDriver.write( GLUE_RAISED_POSITION );
          isApplyingGlue = false;
          fsmState = STATE_MOVE_TO_5;
        } else {
          glueArmStepper.run();
        }
     }
     break; 

     case STATE_MOVE_TO_5:
       if ( !carriageIsMoving && !glueArmIsMoving ) {
         carriageStepper.move( 1575 );
         carriageIsMoving = true;
         glueArmStepper.move( 1754 );
         glueArmStepper.setMaxSpeed( GLUE_ARM_MAX_SPEED );         
         glueArmIsMoving = true;
       } else if ( carriageIsMoving && carriageStepper.distanceToGo() == 0 && glueArmIsMoving && glueArmStepper.distanceToGo() == 0 ) {
         carriageIsMoving = false;
         glueArmIsMoving = false;
         fsmState = STATE_APPLY_GLUE_5; 
       } else {
         carriageStepper.run();
         glueArmStepper.run();
       }
       break;

   case STATE_APPLY_GLUE_5:
     if ( !isApplyingGlue ) {
       glueServoDriver.write( GLUE_APPLICATION_POSITION );
       delay( 300 );
       isApplyingGlue = true;
     } else if ( isApplyingGlue ) {
        if ( !glueArmIsMoving ) {
          glueArmStepper.moveTo( 0);
          glueArmStepper.setMaxSpeed( GLUE_ARM_APPLICATION_SPEED );          
          glueArmIsMoving = true;
        } else if ( glueArmIsMoving && glueArmStepper.distanceToGo() == 0 ) {
          glueArmIsMoving = false;
          glueServoDriver.write( GLUE_RAISED_POSITION );
          isApplyingGlue = false;
          fsmState = STATE_MOVE_TO_READY;
        } else {
          glueArmStepper.run();
        }
     }
     break;
     
    case STATE_MOVE_TO_READY:
      if ( !carriageIsMoving ) {
        carriageStepper.moveTo( 0 );
        carriageIsMoving = true;
      } else if ( carriageIsMoving && carriageStepper.distanceToGo() == 0 ) {
        carriageIsMoving = false;
        fsmState = STATE_OFF;
      } else {
        carriageStepper.run();
      }
      break;
    
   default: 
     Serial.println( "INVALID STATE" );
     break;
  }

 
}
