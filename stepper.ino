// Notes:
//. In my solution, I assume starting with the switch simulating the fire sensor being off(The fire sensor starts with a Low signal till it senses fire)
//. During the rotation of the robot in 90 degrees in its place it doesnot check the ultrasonic sensors till rotating completely
//. When reaching a dead end the robot moves two steps backward , checks its surroundings for any escape , if it found a free track it will turn , else it moves backward again
//. To turn right, the rightwheel stepper motor stops while the left wheel motor rotates clockwise and the same idea goes for turning left
//. assume the robot checks another track to move to when it is at a distance of 10 cm or less from the obstacle
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
#define StepsPerRev 64
#define RotSpeed 30
#define forward 1
#define backward 0

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

#define numStepsToRotate 128  //Number of steps for two revolutions
//variables & flags
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

int rightcurrStep = 0;
int leftcurrStep = 0;
int fancurrStep = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(PowerSwitch, INPUT);
  
  pinMode(frontecho, INPUT);
  pinMode(leftecho, INPUT);
  pinMode(rightecho, INPUT);
  pinMode(triggerPin, OUTPUT);
  pinMode(leftWheelAp, OUTPUT);
  pinMode(leftWheelAm, OUTPUT);
  pinMode(leftWheelBp, OUTPUT);
  pinMode(leftWheelBm, OUTPUT);
  pinMode(rightWheelAp, OUTPUT);
  pinMode(rightWheelAm, OUTPUT);
  pinMode(rightWheelBp, OUTPUT);
  pinMode(rightWheelBm, OUTPUT);
  pinMode(fanAp, OUTPUT);
  pinMode(fanAm, OUTPUT);
  pinMode(fanBp, OUTPUT);
  pinMode(fanBm, OUTPUT);  
  attachInterrupt(digitalPinToInterrupt(fireSensor), triggerStatus, CHANGE);
}

void loop() {
  //if the power is on operate the robot
  if(digitalRead(PowerSwitch)){
    //Calculate the number of milliseconds at the begining of the loop
    currenttime = millis();
    //---------------------------------------------------------------------------------------------------
    //Check for the distance using the front ultrasonic
    //Calculate the front distance if the robot is neither performing a rotation nor vanquishing fire
    //In these cases the robot is not moving, so the ultrasonic reading is useless
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
      //The robot moves according to the motionStatus specified without stopping the loop
      switch(motionStatus){
      //move forwards-------------------------------------------
      case 0:
        //Ordinary rotation of the motor
        rightcurrStep++;
        leftcurrStep++;
        moveMotor(rightWheelAp, rightWheelAm, rightWheelBp, rightWheelBm, forward, rightcurrStep, 10000);
        moveMotor(leftWheelAp, leftWheelAm, leftWheelBp, leftWheelBm, forward, leftcurrStep, 10000);
        break;
      //turn right 90 degrees----------------------------------------------  
      case 1:
        if(rotationSteps){
          leftcurrStep ++;
          moveMotor(leftWheelAp, leftWheelAm, leftWheelBp, leftWheelBm, forward, leftcurrStep, 30000);
          rotationSteps -= 1;
          rotating = 1;
         }
        else{
          //reset the motion to moving forward
          motionStatus = 0;
          rotationSteps = numStepsToRotate;
          rotating = 0; 
        }
        break;
      //turn left 90 degrees------------------------------------------------  
      case 2:        
        if(rotationSteps){
          rightcurrStep ++;
          moveMotor(rightWheelAp, rightWheelAm, rightWheelBp, rightWheelBm, forward, rightcurrStep, 30000);
          rotationSteps -= 1;
          rotating = 1;
         }
        else{
          //reset the motion to moving forward
          motionStatus = 0;
          rotationSteps = numStepsToRotate;
          rotating = 0; 
        }
        break;
       //move backwards two wheel revolutions-------------------------------------------
      case 3:
        if(rotationSteps){
          (rightcurrStep >= 1)? rightcurrStep -- : rightcurrStep = 3;
          (leftcurrStep >= 1)? leftcurrStep -- : leftcurrStep = 3;
          moveMotor(rightWheelAp, rightWheelAm, rightWheelBp, rightWheelBm, backward, rightcurrStep, 10000);
          moveMotor(leftWheelAp, leftWheelAm, leftWheelBp, leftWheelBm, backward, leftcurrStep, 10000);
          rotationSteps -= 1;
          rotating = 1;  
        }
        else {
          motionStatus = 0;
          rotationSteps = numStepsToRotate;
          rotating = 0;  
        }  
      }
    }
    
    //blinking the LED every one second
    if(currenttime - prevtime >= 1000){
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
      prevtime = currenttime;
    }
    
    //If a fire alert occurs set the alertStarting time with the current time
    if(fireAlert && !alerttimeset) {
        alerttime = currenttime;
        alerttimeset = 1;
    }
    //If an alertConfirm signal is asserted this means the sensor returned a low after a high signal so the duration of high signal should be checked
    if(alertConfirm){
      //If the duration is more than half a second
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
    //force switching off the LED if power the switch becomes off
    digitalWrite(ledPin, LOW);
  }

}
//--------------------------------------------------------------------------------------------------------------------------
//ISR to trigger the status of flags when the sensor input is triggered
void triggerStatus(){
  //If the exit signal is high then reset its state to low when the sensor changes from high to low
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
      fancurrStep++;
      moveMotor(fanAp, fanAm, fanBp, fanBm, forward, fancurrStep, 30000);
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
  //Invoke the ultrasonic sensor to generate ultrasonic waves(LOW- HIGH - LOW) Signal on trigger pin 
  unsigned long triggertime;
  triggertime = micros();
  while(micros() - triggertime < 20){
    digitalWrite(triggerPin, LOW);
  }
  triggertime = micros();
  while(micros() - triggertime < 40){
    digitalWrite(triggerPin, HIGH); 
  }
  digitalWrite(triggerPin, LOW);
  
  float distance = 0;
  float duration = 0;
  int counter = 0;
  // the duration till the waves bounce and return to the reciever are twice the time through which the trigger pin goes to high then returns to low on recieving the wave
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
      }
//------------------------------------calculate the distance --------------------------------------      
  //0.0344 is the speed of ultrasonic waves in air in cm/ microsecond
  //we divide by 2 to get the duration between the emission of the wave and the instance it hits a surface 
  distance = (duration / 2) * 0.0344;
  return distance;
}

//Move the stepper motor
void moveMotor(int Ap, int Am, int Bp, int Bm, int direction, int currentStep, int stepWait){
  currentStep %= 4;
  unsigned long waitStart = micros();
  while(micros() - waitStart < stepWait) {} 
  switch(currentStep){
    case 0:
    digitalWrite(Ap, 1);
    digitalWrite(Bp, 1);
    digitalWrite(Am, 0);
    digitalWrite(Bm, 0);
    break;
  
    case 1:
    digitalWrite(Ap, 1);
    digitalWrite(Bp, 0);
    digitalWrite(Am, 0);
    digitalWrite(Bm, 1);
    break;
  
    case 2:
    digitalWrite(Ap, 0);
    digitalWrite(Bp, 0);
    digitalWrite(Am, 1);
    digitalWrite(Bm, 1);
    break;
  
    case 3:
    digitalWrite(Ap, 0);
    digitalWrite(Bp, 1);
    digitalWrite(Am, 1);
    digitalWrite(Bm, 0);
   }
}
