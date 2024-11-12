#include <ezButton.h>
#include <Stepper.h>

const int ARRAY_SIZE = 6;
int receivedData[3];
int receivedChecksum;
int numIntegers;
int index = 0;
int dataIndex = 0;
const int stepsPerRevolution = 200;
int currentYaw = 100;
int currentPitch = 100;

int yawMax = 200;
int yawMin = 0;

int pitchMax = 50;
int pitchMin = 0;

int yawSetpointDegrees = 180;
int pitchSetpointDegrees = 180;

bool previousLaserState;
bool previousTurretState;

//int yawSlew = 0;
//int pitchSlew = 0;
bool laserState = false;

bool turretState = false;

bool isCalibrated = false;

int in1Yaw = 11;
int in2Yaw = 10;
int in3Yaw = 9;
int in4Yaw = 8;

int in1Pitch = 14;
int in2Pitch = 15;
int in3Pitch = 16;
int in4Pitch = 17;

int yawLimitPin = 7;
int pitchLimitPin = 4;

int laserPin = 5;
int turretPin = 6;

ezButton yawSwitch(yawLimitPin);
ezButton pitchSwitch(pitchLimitPin);

Stepper yawStepper(stepsPerRevolution, in4Yaw,in3Yaw,in2Yaw,in1Yaw);
Stepper pitchStepper(stepsPerRevolution, in4Pitch,in3Pitch,in2Pitch,in1Pitch);

int degreesToSteps(int degrees){
  return degrees / 360 * 200;
}

void goToSetpoint(int currentStep, int setpoint, char stepperID){
  if (currentStep < setpoint){
    

    if(stepperID == 'y' && currentStep < yawMax){
      yawStepper.step(1);
      currentYaw = currentYaw + 1;
    }
    else if(stepperID == 'p' && currentStep < pitchMax){
      pitchStepper.step(1);
      currentPitch = currentPitch + 1;
    }
  }
  else if (currentStep > setpoint){
    

    if(stepperID == 'y' && currentStep > yawMin){
      yawStepper.step(-1);
      currentYaw = currentYaw - 1;
    }
    else if(stepperID == 'p' && currentStep > pitchMin){
      pitchStepper.step(-1);
      currentPitch = currentPitch - 1;
    }
  }
}

void calibrateTurret(){
  yawStepper.setSpeed(100);
  pitchStepper.setSpeed(100);
  while(!yawSwitch.isPressed()){
    yawSwitch.loop();
    yawStepper.step(-1);
  
  }
  currentYaw = 0;
  yawStepper.step(100);
  currentYaw = 100;
 // while(!pitchSwitch.isPressed()){
   // pitchSwitch.loop();
  //  pitchStepper.step(-1);
  }
  currentPitch = 0;
  //pitchStepper.step(20);
  currentPitch = 20;
  isCalibrated = true;

  
}

void readSerial(){
  bool completed = false;
  while(!completed){
    if (Serial.available() >= 1) {
    char receivedChar = (char) Serial.read();
    int dataHolderTurretState;
    int dataHolderLaser;
    int dataHolderYawSetPoint;
    int dataHolderPitchSetPoint;

    if (receivedChar == 's') {
      dataIndex = 0;
      receivedData[0] = 0;
      receivedData[1] = 0;
      receivedData[2] = 0;
      

    }
    else if(receivedChar == 'd'){
      dataIndex = 1;
    }
    else if(receivedChar == 'e'){
      dataIndex = 2;
    }
    
    
    else if(receivedChar == 'z'){
      
      if (receivedData[0] == 0){
        laserState = false;
        turretState = false;
      }
      else if (receivedData[0] == 1){
        laserState = false;
        turretState = true;
      }
      else if (receivedData[0] == 2){
        laserState = true;
        turretState = false;
      }
      else if(receivedData[0] == 3){
        laserState = true;
        turretState = true;
      }
      yawSetpointDegrees = receivedData[1];
      pitchSetpointDegrees = receivedData[2];
      completed = true;
    }
   
    
    else if(isdigit(receivedChar)){
      
      receivedData[dataIndex] = (receivedData[dataIndex] * 10) + (receivedChar - '0');
      

    }
      if (receivedData[0] == 0){
        laserState = false;
        turretState = false;
      }
      else if (receivedData[0] == 1){
        laserState = true;
        turretState = false;
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else if (receivedData[0] == 2){
        laserState = false;
        turretState = true;
      }
      else if(receivedData[0] == 3){
        laserState = true;
        turretState = true;
        digitalWrite(LED_BUILTIN, HIGH);
      }
  }
  
      

      


      //yawSetpointDegrees = receivedData[1];
      //pitchSetpointDegrees = receivedData[2];
  }
    
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(in1Yaw, OUTPUT);
  pinMode(in2Yaw, OUTPUT);
  pinMode(in3Yaw, OUTPUT);
  pinMode(in4Yaw, OUTPUT);
  
  pinMode(in1Pitch, OUTPUT);
  pinMode(in2Pitch, OUTPUT);
  pinMode(in3Pitch, OUTPUT);
  pinMode(in4Pitch, OUTPUT);

  pinMode(laserPin, OUTPUT);
  pinMode(turretPin, OUTPUT);
  yawSwitch.setDebounceTime(50);
  pitchSwitch.setDebounceTime(50);
  while (!Serial) {
    // Wait for the serial port to connect
  }

  
  

}

void loop() {
  // put your main code here, to run repeatedly:
  readSerial();
  yawSwitch.loop();
  pitchSwitch.loop();

  if (laserState){
    digitalWrite(laserPin, HIGH);
  }
  else{
    digitalWrite(laserPin, LOW);
  }

  if(turretState){
    digitalWrite(turretPin, HIGH);
  }
  else{
    digitalWrite(turretPin, LOW);
  }

  if(yawSwitch.isPressed() && currentYaw < 100){
    currentYaw = 0;
  }
  else if(yawSwitch.isPressed() && currentYaw > 100){
    currentYaw = 200;
  }

  if(pitchSwitch.isPressed()){
    currentPitch = 0;
  }

  if (!isCalibrated && turretState){
    calibrateTurret();
  }

  

  goToSetpoint(currentYaw, degreesToSteps(yawSetpointDegrees), 'y');
  goToSetpoint(currentPitch, degreesToSteps(pitchSetpointDegrees), 'p');
  
  //previousTurretState = turretState;

  

  

}
