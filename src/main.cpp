#include "Servo.h"
#include <Wire.h> 


//---------------------------------------------------------------------------
//Servo Motor Definitions
//---------------------------------------------------------------------------
int servo_pin = D7;  // for ESP8266 microcontroller
//int servo_pin = 4;  // for ESP32 microcontroller
Servo myservo;
int currentTiltAngle = 0;  //for tilt angle

void setTiltAngle(int angle){
  myservo.write(angle);
  currentTiltAngle = angle;
}


//---------------------------------------------------------------------------
//Magnetic Positioner Definitions
//---------------------------------------------------------------------------
int magnetStatus = 0; //value of the status register (MD, ML, MH)
int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle 
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])

int ReadRawAngle()
{ 
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
  
  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625;   
  return rawAngle;
}
void checkMagnetPresence()
{  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

    Serial.print("Magnet status: ");
    Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }      
  
  //Status register output: 0 0 MD ML MH 0 0 0  
  //MH: Too strong magnet - 100111 - DEC: 39 
  //ML: Too weak magnet - 10111 - DEC: 23     
  //MD: OK magnet - 110111 - DEC: 55

  Serial.println("Magnet found!");
  //delay(1000);  
}



//---------------------------------------------------------------------------
//Pan Motors Definitions
//---------------------------------------------------------------------------
int ENA = 14;
int IN1 = 0;
int IN2 = 2;

void panCW(){
digitalWrite(ENA, HIGH); // set speed to 200 out of possible range 0~255
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
}
void panCCW(){
  digitalWrite(ENA, HIGH); // set speed to 200 out of possible range 0~255
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}
void panStop(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENA, LOW); // set speed to 200 out of possible range 0~255
}
void setPanAngle(int targetAngle){
  int currentPanAngle = ReadRawAngle();
  int diffAngle = currentPanAngle - targetAngle;
  while(currentPanAngle != targetAngle){
    currentPanAngle = ReadRawAngle();
    diffAngle = currentPanAngle - targetAngle;
    if(abs(diffAngle) < 2048){
      if(diffAngle > 0){
        panCW();
      }else{
        panCCW();
      }
    }else{
      if(diffAngle > 0){
        panCCW();
      }else{
        panCW();
      }
    }    
  }   
}

//---------------------------------------------------------------------------
// Methods
//---------------------------------------------------------------------------
void moveNozzle(int tiltAngle, int panAngle){
  setTiltAngle(tiltAngle);
  setPanAngle(panAngle);
}
void logPosition(){
  Serial.print("Pan Angle: ");
  Serial.println(rawAngle);
}



void setup() { 
//checkMagnetPresence(); //check the magnet (blocks until magnet is found)
  myservo.attach(servo_pin);  //Set Tilt Servo
  pinMode(ENA, OUTPUT); // set all the pan motor control pins to outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.begin(9600); 
  while(!Serial){} // Waiting for serial connection
 
  Serial.println(); // Scan for devices and report found devices
  Serial.println("Start I2C scanner ...");
  Serial.print("\r\n");
  byte count = 0;
  
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
      {
      Serial.print("Found I2C Device: ");
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println(")");
      count++;
      delay(1);
      }
  }
  Serial.print("\r\n");
  Serial.println("Finish I2C scanner");
  Serial.print("Found ");
  Serial.print(count, HEX);
  Serial.println(" Device(s).");
  
} 



void loop() { // move from 0 to 180 degrees with a positive angle of 1
  ReadRawAngle();
  logPosition();
  delay(500);
}