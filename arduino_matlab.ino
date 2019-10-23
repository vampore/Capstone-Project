#include <Stepper.h>

int ledPin = 13;
int matlabData;
const int stepsPerRevolution = 200;
Stepper Stepper1(stepsPerRevolution, 4, 5, 6, 7);
Stepper Stepper2(stepsPerRevolution, 8, 9, 10, 11);
 
void setup() 
{
  pinMode(ledPin,OUTPUT);
  Stepper1.setSpeed(60);
  Stepper2.setSpeed(10);
  Serial.begin(9600);
}
 
void loop() 
{   
   if(Serial.available()>0) // if there is data to read
   {
    matlabData=Serial.read(); // read data
    if (matlabData == 1){
      Stepper1.step(stepsPerRevolution);    
    }
    if (matlabData == 2){
      Stepper1.step(-stepsPerRevolution);
    }
    if (matlabData == 3){
      Stepper2.step(stepsPerRevolution);    
    }
    if (matlabData == 4){
      Stepper2.step(-stepsPerRevolution);    
    }
  }
}
