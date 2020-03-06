#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Arduino.h>
#include "BasicStepperDriver.h"
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

int dumb_index;

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120
// RPM 600 from last year 

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// All the wires needed for full functionality
#define DIR 0
#define STEP 1
//Uncomment line to use enable/disable functionality
#define SLEEP 7

int motorMultiplier = 27;

// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

//boolean to flag when rocket has launched 
bool isLaunch = true;
bool isLand = true;  

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  dumb_index = 0;
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

  stepper.begin(RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);
}

void loop(void) 
{
  
  
  int historyShit[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int dumb_len = sizeof(historyShit) / sizeof(historyShit[0]);

  int _sum = 0;

  for(int i = 0; i < dumb_len; i ++) {
    _sum += historyShit[i];
  }

  float avg = _sum / dumb_len;  

  /* CHECK HISOTRY HERE */
  if(avg >= 30) {
    isLaunch = true;
  }
  if (avg < 1 & isLaunch) {
    isLand = true;
  }
  
  /* Get a new sensor event */
  sensors_event_t event; 
  bno.getEvent(&event);
  int x_or = event.orientation.x;
  int y_or = event.orientation.y;
  int z_or = event.orientation.z;
  imu::Vector<3> linear_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  int x_lacc = linear_acc.x();
  int y_lacc = linear_acc.y();
  int z_lacc = linear_acc.z(); 

  historyShit[dumb_index] = y_lacc;

  dumb_index = (dumb_index + 1) % dumb_len;
  
  /* check if flag is set to true, if is not set to true, check to set flag true?*/
  /* using y linear acceleration to determine if rocket has launched or not */
  
  if (isLaunch && isLand && (abs(y_or) < 5) && (abs(abs(z_or) - 180) < 5) ) {
    stepper.enable();
    //stepper.move(100*MICROSTEPS);
    //NEXT LINE IS TESTING ONLY
    stepper.move(MOTOR_STEPS*MICROSTEPS*motorMultiplier); 
  }
}
