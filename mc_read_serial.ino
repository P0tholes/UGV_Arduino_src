const int ARRAY_SIZE = 6;
int receivedData[5];
int receivedChecksum;
int numIntegers;
int index = 0;
int dataIndex = 0;

int throttle = 0;
int twist = 0;

int R_ENR = 2;
int R_ENL = 4;
int L_ENR = 7;
int L_ENL = 8;
int R_PWMR = 3;
int R_PWML = 5;
int L_PWMR = 9;
int L_PWML = 10;



int ledBrightness = 0;
int testLedBright = 0;

void differentialDrive(int throttle, int twist){
  int y = 128;
  int x = 128;
  y = throttle;
  x = twist;

  int speeds[2];
  y = y - 128;
  x = x - 128;
  speeds[0] = y - x;
  speeds[1] = y + x;
  speeds[0] = constrain(speeds[0],-128,128);
  speeds[1] = constrain(speeds[1],-128,128);
  sendMotorSpeeds(speeds[0],speeds[1]);


  
  
}


void sendMotorSpeeds(int left, int right){
  if(left >= 0){
    
    
    analogWrite(L_PWML, 0);
    analogWrite(L_PWMR, left);
    
    
  }
  else{
    

    analogWrite(L_PWMR, 0);
    analogWrite(L_PWML, -left);

  }

  if(right >= 0){
    

    
    analogWrite(R_PWML, 0);
    analogWrite(R_PWMR, right);
    

    

  }
  else{
    

    analogWrite(R_PWMR, 0);
    analogWrite(R_PWML, -right);
    


  }
}





void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(R_ENR, OUTPUT);
  pinMode(R_ENL, OUTPUT);
  pinMode(L_ENR, OUTPUT);
  pinMode(L_ENL, OUTPUT);
  pinMode(R_PWMR, OUTPUT);
  pinMode(R_PWML, OUTPUT);
  pinMode(L_PWMR, OUTPUT);
  pinMode(L_PWML, OUTPUT);
  digitalWrite(R_ENR, HIGH);
  digitalWrite(R_ENL, HIGH);
  digitalWrite(L_ENR, HIGH);
  digitalWrite(L_ENL, HIGH);
  
  while (!Serial) {
    // Wait for the serial port to connect
  }

}

void loop() {
  

  if (Serial.available() >= 1) {
    char receivedChar = (char) Serial.read();
    int dataHolderThrottle;
    int dataHolderTwist;

    if (receivedChar == 's') {
      dataIndex = 0;
      receivedData[0] = 0;
      receivedData[1] = 0;

    }
    else if(receivedChar == 'd'){
      dataIndex = 1;
    }
    else if(receivedChar == 'z'){
      
      throttle = receivedData[0];
      twist = receivedData[1];
      
      
      
      
      

    }
    else if(isdigit(receivedChar)){
      
      receivedData[dataIndex] = (receivedData[dataIndex] * 10) + (receivedChar - '0');

    }
    differentialDrive(throttle,twist);
    
      

    
    

    

    



    

      
  
      
    
            
      
    
        
        

      
    
    

  



    
    
    
    
    
    
    
    
     
    
    
    delay(1);
  }
  
  
}
