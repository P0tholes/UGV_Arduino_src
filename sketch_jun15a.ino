
#include <ezButton.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <AccelStepper.h>
//ON TTYUSB0
#define EN_PIN_YAW           4 // Enable
#define DIR_PIN_YAW          3 // Direction
#define STEP_PIN_YAW         12 // Step
#define EN_PIN_PITCH          2 // Enable
#define DIR_PIN_PITCH          10 // Direction
#define STEP_PIN_PITCH 11

#define SW_SCK           5 // Software Slave Clock (SCK)
#define SW_RX_PITCH            6 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX_PITCH           7 // Software Slave Clock (SCK)
#define SW_RX_YAW            9 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX_YAW 8// TMC2208/TMC2224 SoftwareSerial transmit pin
 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS_PITCH 0b00
#define DRIVER_ADDRESS_YAW 0b00
// TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f

#define LIMIT_DETECT_PIN_PITCH 19
#define LIMIT_DETECT_PIN_YAW 18
#define LASER_PIN 17
SoftwareSerial SoftSerialPitch(SW_RX_PITCH, SW_TX_PITCH);
SoftwareSerial SoftSerialYaw(SW_RX_YAW, SW_TX_YAW);

TMC2209Stepper yaw_driver(&SoftSerialYaw, R_SENSE, DRIVER_ADDRESS_YAW);
TMC2209Stepper pitch_driver(&SoftSerialPitch, R_SENSE, DRIVER_ADDRESS_PITCH);

AccelStepper pitch_stepper = AccelStepper(pitch_stepper.DRIVER, STEP_PIN_PITCH, DIR_PIN_PITCH);
AccelStepper yaw_stepper = AccelStepper(yaw_stepper.DRIVER, STEP_PIN_YAW, DIR_PIN_YAW);

const int ARRAY_SIZE = 6;
int receivedData[3];
int receivedChecksum;
int numIntegers;
//int index = 0;
int dataIndex = 0;

const int yawStepsPerRevolution = 200;
const int pitchStepsPerRevolution = 200;
int currentYaw = 0;
int currentPitch = 0;
int yawMaxSteps = 200;
int yawMinSteps = 0;

int pitchMaxSteps = 50;
int pitchMinSteps = 0;

int yawMaxDegrees = 180;
int yawMinDegrees = -180;

int pitchMaxDegrees = 60;
int pitchMinDegrees = -30;

int yawSetpointDegrees = 0;
int pitchSetpointDegrees = 0;

bool previousLaserState;
bool previousTurretState;

bool laserState = false;

bool turretState = false;

bool isCalibrated = false;
bool isPitchCalibrated = false;
bool isYawCalibrated = false;

//int yawLimitPin = 7;
//int pitchLimitPin = 4;

//int laserPin = 18;

bool atPitchSetpoint = false;
bool atYawSetpoint = false;

ezButton yawSwitch(LIMIT_DETECT_PIN_YAW);
ezButton pitchSwitch(LIMIT_DETECT_PIN_PITCH);



int degreesToSteps(int degrees){
  return degrees / 360 * 200 * 16;
}
int byteToDegrees(int inputByte){
  return (inputByte*360L/255)-180;
}
void lightFromInput(int input){
  if (input >128){
    digitalWrite(LED_BUILTIN, HIGH);
  
  }
  else{
   digitalWrite(LED_BUILTIN, LOW);
  }
}



int calibratePitch(){
  Serial.print("BEGINNING PITCH CALIBRATION");
  //digitalWrite(EN_PIN_PITCH, LOW);
   pitchSwitch.loop();
   pitch_stepper.setMaxSpeed(500);
    pitch_stepper.setAcceleration(9000);
    pitch_stepper.moveTo(180*32L);
  while(pitchSwitch.getState() == true){
    Serial.print("movin");
    
    pitch_stepper.run();
    pitchSwitch.loop();
    
    //Serial.print("AT REST");
  }
  pitch_stepper.setCurrentPosition(degreesToStepsPitch(70));
  
  //pitch_driver.shaft(false);
  pitch_stepper.setMaxSpeed(4000);
  pitch_stepper.setAcceleration(0);
  pitch_stepper.moveTo(PitchSetpoint(0));
  while(pitch_stepper.distanceToGo() != 0){
    pitch_stepper.run();
  }
  
  isPitchCalibrated = true;
  
  return 45;
 }

 int calibrateYaw(){
  Serial.print("BEGINNING YAW CALIBRATION");
  //digitalWrite(EN_PIN_YAW, HIGH);
  
   yawSwitch.loop();
   yaw_stepper.setMaxSpeed(1000);
    yaw_stepper.setAcceleration(500);
    yaw_stepper.moveTo(-360*32L);
    
  while(yawSwitch.getState() == true){
    yaw_stepper.run();
    
    
    yawSwitch.loop();
    
    //Serial.print("AT REST");
  }
  yaw_stepper.setCurrentPosition(degreesToStepsYaw(-180));
  yaw_stepper.setMaxSpeed(4000);
  yaw_stepper.setAcceleration(25000);
  yaw_stepper.moveTo(YawSetpoint(0));
  while(yaw_stepper.distanceToGo() != 0){
    yaw_stepper.run();
  }
  //yaw_driver.shaft(false);
  //for (uint16_t i = 45*16; i>0; i--) {
  //  digitalWrite(STEP_PIN_YAW, HIGH);
  //  delayMicroseconds(20);
  //  digitalWrite(STEP_PIN_YAW, LOW);
   // delayMicroseconds(20);
//  }
  
  
  isYawCalibrated = true;
  
  return 45;
 }

 void testSwitch(){
  //digitalWrite(EN_PIN_PITCH, HIGH);
  if(yawSwitch.getState()){
    //Serial.print("PRESSED");
  }
  else{
    //Serial.print("OPEN");
  }
  //Serial.print('\n');
 }

long int degreesToStepsPitch(int angleInDegrees){
  return (angleInDegrees*200*16L)/360;
}
long int degreesToStepsYaw(int angleInDegrees){
  return (((angleInDegrees*200*32L)/360)*352)/225;
  
}
String receivedString = "           ";
String pitchExtracted = "0";
String yawExtracted = "0";


long int PitchSetpoint(int angleInDegrees){
  if (angleInDegrees > pitchMaxDegrees) {
    return degreesToStepsPitch(pitchMaxDegrees);
  }
  else if (angleInDegrees < pitchMinDegrees) {
    return degreesToStepsPitch(pitchMinDegrees);
  }
  else{
    return degreesToStepsPitch(angleInDegrees);
  }
  
}

long int YawSetpoint(int angleInDegrees){
  if (angleInDegrees > yawMaxDegrees) {
    return degreesToStepsYaw(yawMaxDegrees);
  }
  else if (angleInDegrees < yawMinDegrees) {
    return degreesToStepsYaw(yawMinDegrees);
  }
  else{
    return degreesToStepsYaw(angleInDegrees);
  }
  
}

/*void setTurretInput(const char* cmd){
  if (inputString[1] == '0'){
           turretState = false;      
      }
        else if (inputString[1] == '1'){           
            turretState = false;
      }
         else if (inputString[1] == '2'){
           turretState = true;
      }
         else if(inputString[1] == '3'){           
            turretState = true;      
      }
}
*/
char pitchValueStr[4];
char yawValueStr[4];
void setGimbalInput(){
  if(receivedString[0] == 's' && receivedString[2] == 'd'&& receivedString[6] == 'e'&& receivedString[10] == 'z'
        )
        {
          pitchExtracted = receivedString.substring(3, 6);
          pitchSetpointDegrees = (100 - ((pitchExtracted.toInt()*100)/120))-30;
          yawExtracted = receivedString.substring(7, 10);
          yawSetpointDegrees = yawExtracted.toInt()-180;
        }
        

        
  /*if(cmd[2]=='d'){
    strncpy(pitchValueStr, cmd + 3, 3);
    pitchValueStr[3] = '\0';
    pitchSetpointDegrees = (100 - ((atoi(pitchValueStr)*100)/120))-30;
  }
  if(cmd[6]=='e'){
    strncpy(yawValueStr, cmd + 7, 3);
    yawValueStr[3] = '\0';
    yawSetpointDegrees = atoi(yawValueStr)-180;
  }
  Serial.println(cmd);
  */
}
void setStateInput(const char* cmd){
  /*if (inputString[1] == '0'){
           laserState = false;           
       }
        else if (inputString[1] == '1'){
            laserState = true;            
      }
         else if (inputString[1] == '2'){
            laserState = false;
      }
         else if(inputString[1] == '3'){
           laserState = true;       
      }
      */
   if (cmd[0]=='s'){
    switch(cmd[1]){
      case '0':
        laserState = false; 
        turretState = false;
        break;
      case '1':
        laserState = true; 
        turretState = false;
        break;
      case '2':
        laserState = false; 
        turretState = true;
        break;
      case '3':
        laserState = true; 
        turretState = true;
        break;
    }
   }
}

char inputBuffer[16]= "s0d000e000z";
byte index = 0;
void readSerialString(){
  if (Serial.available() > 0) {
        receivedString = Serial.readStringUntil('\n');  // Read data until newline    
    }
   /*while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputBuffer[index] = '\0'; // Null-terminate
      index = 0;
      // process inputBuffer here
      index = 0; // Reset for next input
    } else if (index < sizeof(inputBuffer) - 1) {
      inputBuffer[index++] = c;
    }
  }
  */
}
bool escaped = false;
void readSerialBytes(){
  static uint8_t byteBuffer[3];
  if (Serial.available()>=4){
    if(Serial.read() == 255){
      while (Serial.available() < 3);
      int state = Serial.read();
      
      pitchSetpointDegrees = (100 - ((Serial.read()*100)/120))-30;
      yawSetpointDegrees = byteToDegrees(Serial.read());
      
      
      //Serial.println(state);
      
      //Serial.println(pitchSetpointDegrees);
      
      //Serial.println(yawSetpointDegrees);
    }
  

  
  }
 //Serial.println(byteBuffer[0]);
 //Serial.println(byteBuffer[1]);
 //Serial.println(byteBuffer[2]);

  
}


bool atSP1 = false;
bool atSP2 = false;
int sp1 = 10;
int sp2 = 80;

void setup() {
  Serial.begin(250000);
  SoftSerialPitch.begin(11520); 
  SoftSerialYaw.begin(11520); 
  yaw_driver.beginSerial(11520);
  pitch_driver.beginSerial(11520);
  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LASER_PIN, OUTPUT);
  yaw_stepper.setMaxSpeed(8000);
  yaw_stepper.setAcceleration(25000);
  yaw_stepper.setEnablePin(EN_PIN_YAW);

  pitch_stepper.setMaxSpeed(3800);
  pitch_stepper.setAcceleration(18000);
  pitch_stepper.setEnablePin(EN_PIN_PITCH);
  

  yawSwitch.setDebounceTime(50);
  pitchSwitch.setDebounceTime(50);
  // put your setup code here, to run once:
  pinMode(EN_PIN_YAW, OUTPUT);
  pinMode(EN_PIN_PITCH, OUTPUT);
  
  pinMode(STEP_PIN_YAW, OUTPUT);
  pinMode(STEP_PIN_PITCH, OUTPUT);
  
  pinMode(DIR_PIN_YAW, OUTPUT);
  pinMode(DIR_PIN_PITCH, OUTPUT);
  
  digitalWrite(EN_PIN_YAW, LOW);
  digitalWrite(EN_PIN_PITCH, LOW);

  

  yaw_driver.begin();
  pitch_driver.begin();

  yaw_driver.toff(5);
  pitch_driver.toff(5);// Enables driver in software
  
  yaw_driver.rms_current(800);
  pitch_driver.rms_current(600);// Set motor RMS current
  yaw_driver.en_spreadCycle(false);
  pitch_driver.en_spreadCycle(false);
  yaw_driver.pwm_autoscale(true);
  pitch_driver.pwm_autoscale(true);
  yaw_driver.microsteps(16);
  pitch_driver.microsteps(16); 

  

 currentPitch = calibratePitch();
 currentYaw = calibrateYaw();
  //Serial.println("CLBR COMPLETE");
  digitalWrite(EN_PIN_YAW, LOW);
  digitalWrite(EN_PIN_PITCH, LOW);
  yaw_stepper.setAcceleration(25000);
  pitch_stepper.setAcceleration(0);
  while (!Serial) {
    // Wait for the serial port to connect
  }
 Serial.println("SP LINK SUCCESS");
 digitalWrite(LED_BUILTIN, HIGH);

}
unsigned long lastSerialCheck = 0;
unsigned long serialInterval = 100;


void loop() {
  //if(millis() - lastSerialCheck>= serialInterval){
    //lastSerialCheck = millis();
   readSerialBytes();
   pitch_stepper.moveTo(PitchSetpoint(pitchSetpointDegrees));
   yaw_stepper.moveTo(YawSetpoint(yawSetpointDegrees));
    
 // }
  //Serial.println(inputBuffer);
  
  
  if (laserState){
    //digitalWrite(LASER_PIN, HIGH);
  }
  else{
    //digitalWrite(LASER_PIN, LOW);
  }
  
  yaw_stepper.run();
  pitch_stepper.run();
 
  
  lightFromInput(yawSetpointDegrees);
  

}
