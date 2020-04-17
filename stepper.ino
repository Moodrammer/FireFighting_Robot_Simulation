
#define StepsPerRev 64
#define RotSpeed 30

#define leftWheelBm 8
#define leftWheelBp 10
#define leftWheelAm 9
#define leftWheelAp 11

#define rightWheelBm A0
#define rightWheelBp A2
#define rightWheelAm A1
#define rightWheelAp A3

#define fanBm 4
#define fanBp 6
#define fanAm 5
#define fanAp 7

#define ledPin 12
#define PowerSwitch A4
#define fireSensor 2

#define triggerPin A5
#define frontecho 1
#define leftecho 3
#define rightecho 0

#define numStepsToRotate 128  
//-----------------------------------------------------------------------------------------------------------------------
#include <Stepper.h>
//------------------------------------------------------------------------------------------------------------------------
//Steppers
Stepper leftWheel(StepsPerRev, leftWheelBm, leftWheelBp, leftWheelAm, leftWheelAp);
Stepper rightWheel(StepsPerRev, rightWheelBm, rightWheelBp, rightWheelAm, rightWheelAp);
Stepper fan(StepsPerRev, fanBm, fanBp, fanAm, fanAp);
//variables
unsigned long prevtime = 0;
unsigned long currenttime = 0;
unsigned long alerttime = 0;
bool ledState = 0;
volatile bool fireAlert = 0;
volatile bool alertConfirm = 0; 
volatile bool exitSignal = 0;
bool alerttimeset = 0;
bool stopMotors = 0;
int motionStatus = 0; //0: moveforward 1: turn right 2: turn left 3: movebackwards
int rotationSteps = numStepsToRotate; //two revolutions
bool rotating = 0; //0: the robot is not rotating 1: the robot is currently applying a rotation

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(PowerSwitch, INPUT);
  
  pinMode(frontecho, INPUT);
  pinMode(leftecho, INPUT);
  pinMode(rightecho, INPUT);
  pinMode(triggerPin, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(fireSensor), triggerStatus, CHANGE);
}

void loop() {
  //if the power is on operate the robot
  if(digitalRead(PowerSwitch)){
    //Calculate the time at the begining of the loop
    currenttime = millis();
    //---------------------------------------------------------------------------------------------------
    //Check for the distance using the front ultrasonic
    //calculate the front distance if the robot is neither performing a rotation nor vanquishing fire
    if(!rotating && !alertConfirm){
      if(calcDist(frontecho) < 10){
        if(calcDist(rightecho) < 10){
          //check left
          if(calcDist(leftecho) < 10){
            //turn backwards
            motionStatus = 3;  
          }
          else{
            //turn left
            motionStatus = 2;
          }
        }
        else{
          //turn right
          motionStatus = 1; 
        }
      }
    }
    //------------------------------------------------------------------------------------------------------
    
    if(!stopMotors){
      leftWheel.setSpeed(RotSpeed);
      rightWheel.setSpeed(RotSpeed);
      
      switch(motionStatus){
      //move forwards
      case 0:
        //Ordinary rotation of the motor
        rightWheel.step(StepsPerRev/ 32);
        leftWheel.step(StepsPerRev/ 32);
        break;
      //turn right  
      case 1:
        if(rotationSteps){
          leftWheel.step(StepsPerRev/ 32);
          rotationSteps -= StepsPerRev/ 32;
          rotating = 1;
         }
        else{
          //reset the motion to moving forward
          motionStatus = 0;
          rotationSteps = numStepsToRotate;
          rotating = 0; 
        }
        break;
      //turn left  
      case 2:        
        if(rotationSteps){
          rightWheel.step(StepsPerRev/ 32);
          rotationSteps -= StepsPerRev/ 32;
          rotating = 1;
         }
        else{
          //reset the motion to moving forward
          motionStatus = 0;
          rotationSteps = numStepsToRotate;
          rotating = 0; 
        }
        break;
      case 3:
        if(rotationSteps){
          rightWheel.step(-(StepsPerRev/ 32));
          leftWheel.step(-(StepsPerRev/ 32));
          rotationSteps -= StepsPerRev / 32;
          rotating = 1;  
        }
        else {
          motionStatus = 0;
          rotationSteps = numStepsToRotate;
          rotating = 0;  
        }
        break;  
      }
    }
    
    //blinking the LED every one second
    if(currenttime - prevtime >= 1000){
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
      prevtime = currenttime;
    }
    
    //If a fire alert occurs
    if(fireAlert && !alerttimeset) {
        alerttime = currenttime;
        alerttimeset = 1;
    }
    if(alertConfirm){
      if(currenttime - alerttime >= 500){
       //stop the motors from moving
        stopMotors = 1; 
        putoff();
      }
      //else it is a false call
      else {
        //reset the state of all alerts
        fireAlert = 0;
        alerttimeset = 0; 
        alertConfirm = 0;  
      }
    }
  }
  else{
    //force switching off the LED if the switch becomes off
    digitalWrite(ledPin, LOW);
  }

}
//ISR to trigger the status of flags when the sensor input is triggered
void triggerStatus(){
  if(!exitSignal){
    //If there is no fireAlert set the fireAlert
    if(!fireAlert) fireAlert = 1;
   //If there is already a fire alert set the status of the alert  confirm inorder to check if the time has passed
    else if(!alertConfirm) alertConfirm = 1;
    //If there is already a fireAlert and it is confirmed then we set the exit signal to exit the fire extinguising loop 
    else exitSignal = 1;  
  }
  else{
    exitSignal = 0;
  }
}

//Loop that handles the fan motion and putting off the fire 
void putoff(){
  //continue as long as no exit signal has been triggered
  if(!exitSignal){
    fan.setSpeed(RotSpeed);
    fan.step(StepsPerRev/ 32);
  }
  else{
     //reset all the status variables
    fireAlert = 0;
    alertConfirm = 0;
    alerttimeset = 0;
    stopMotors = 0;
   }
}

//Calculates the distance of the triggered ultrasonic
float calcDist(int echopin){
  digitalWrite( triggerPin, LOW ); 
  delayMicroseconds( 2 );
 
  digitalWrite( triggerPin, HIGH );
  delayMicroseconds( 10 );
  digitalWrite( triggerPin, LOW );
  
  unsigned long triggertime;
  float distance = 0;
  float duration = 0;
  int counter = 0;
  
  switch(echopin) {
//-----------------------------------front sensor-------------------------------------------------      
      case frontecho:
        while(--counter != 0){  
          if(digitalRead(frontecho)){
            triggertime = micros();
            break;
          }
        } 
        
        while(--counter != 0){
          if(digitalRead(frontecho) == 0){
            duration = micros() - triggertime;
            break;
          }
        }
        break;
//-----------------------------------------left sensor---------------------------------------------  
      case leftecho:
        while(--counter != 0){  
          if(digitalRead(leftecho)){
            triggertime = micros();
            break;
          }
        }
        while(--counter != 0){
          if(digitalRead(leftecho) == 0){
            duration = micros() - triggertime;
            break;
          }
        }
        break;

//---------------------------------------right sensor ---------------------------------------------
      case rightecho:
        while(--counter != 0){  
          if(digitalRead(rightecho)){
            triggertime = micros();
            break;
          }
        }
        while(--counter != 0){
          if(digitalRead(rightecho) == 0){
            duration = micros() - triggertime;
            break;
          }
        }
         
        break;
      }
//------------------------------------calculate the distance --------------------------------------      
  //0.0344 is the speed of ultrasonic waves in air in cm/ microsecond
  distance = (duration / 2) * 0.0344;
  Serial.println(distance);
  return distance;
}
