#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <Adafruit_seesaw.h>
#include <seesaw_neopixel.h>

class IRProximity {
  Adafruit_seesaw ss;
  int sensor;
  int wait;
  unsigned long previousMillis;

  public:
  IRProximity(int waitTime) {
    wait = waitTime;
    previousMillis = 0;
  }

  void attachPin(int pin, Adafruit_seesaw seesaw) {
    ss = seesaw;
    sensor = pin;
    ss.pinMode(sensor, INPUT_PULLUP);
  }

  boolean getBoolean() {
    if (!ss.digitalRead(sensor)) {
      if (millis()-previousMillis >= wait) return true;
    } else {
      previousMillis = millis();
    }
    return false;
  }
};

Adafruit_seesaw ss;

#define LED 13

#define BAT A1
 
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  400 // microseconds
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     300 // 0 is stopped, 400 is full speed
#define TURN_SPEED        400
#define FORWARD_SPEED     200
#define FULL_SPEED        400
#define REVERSE_DURATION  150 // ms
#define TURN_DURATION     200 // ms
 
ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
 
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];
 
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

#define FRONT_DURATION 10 //ms
#define SIDE_DURATION 25 //ms

IRProximity frontLeft(FRONT_DURATION);
IRProximity frontRight(FRONT_DURATION);
IRProximity left(SIDE_DURATION);
IRProximity right(SIDE_DURATION);

#define TURN_TIME 180 //ms
boolean rTurn = false;
boolean lTurn = false;
unsigned long previousMillis = 0;

void waitForButtonAndCountDown()
{
  digitalWrite(LED, HIGH);
  button.waitForButton();
  digitalWrite(LED, LOW);
   
  // play audible countdown
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(4), 500, 15);  
  delay(1000);
}
 
void setup()
{
  // uncomment if necessary to correct motor directions
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  if(!ss.begin()){
    while(1);
  }

  frontLeft.attachPin(11, ss);
  frontRight.attachPin(10, ss);
  left.attachPin(14, ss);
  right.attachPin(15, ss);
   
  pinMode(LED, OUTPUT);
   
  waitForButtonAndCountDown();
}

void loop()
{
  unsigned int batteryVoltage = analogRead(BAT) * 5000L * 3/2 / 1023;
  digitalWrite(LED, batteryVoltage < 4500);
  
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown();
  }
   

  sensors.read(sensor_values);
  
  if (sensor_values[0] < QTR_THRESHOLD)
  {
    // if leftmost sensor detects line, reverse and turn to the right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else if (sensor_values[5] < QTR_THRESHOLD)
  {
    // if rightmost sensor detects line, reverse and turn to the left
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else
  {
    // otherwise, track and move

    if (rTurn) {
      motors.setSpeeds(FULL_SPEED, -FULL_SPEED);
      if (millis() - previousMillis > TURN_TIME) {
        rTurn = false;
      }
    } else if (lTurn) {
      motors.setSpeeds(-FULL_SPEED, FULL_SPEED);
      if (millis() - previousMillis > TURN_TIME) {
        lTurn = false;
      }
    } else {
      rTurn = right.getBoolean();
      lTurn = left.getBoolean();
      previousMillis = millis();
      
      if (frontLeft.getBoolean() && frontRight.getBoolean()) {
        motors.setSpeeds(FULL_SPEED, FULL_SPEED);
      } else if (frontLeft.getBoolean()) {
        motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
      } else if (frontRight.getBoolean()) {
        motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      } else {
        motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      }
    }
    
  }
}
