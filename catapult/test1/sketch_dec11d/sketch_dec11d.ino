#include <math.h>
#include <Servo.h>
#include "NewPing.h"

#define RPM 200 //3750 old value
#define TRIG_PIN 2
#define ECHO_PIN 3
#define INTIALPOINT 16.2
#define PI 3.14159265
#define MOTOR_MS 0.047
/* this is with r=0.0015 and 200rpm
 * ORIGINAL DC MOTOR -> 0.39267 WITH 0.001m radius and 3750 RPM. CHANGE when needed
 */

bool mState = true; //GOT CHANGED TO FALSE, SET BACK TO TRUE
bool spinState = true; //GOT CHANGED TO TRUE, SET BACK TO FALSE
bool launchState = false; //GOT CHNAGED TO TRUE, SET BACK TO FALSE
bool spinBackState = false;

double measuredDistance; //change to 750 for tests
double measuredDistanceM;
double duration;

double tSpin;

//pins for buttons
int mDistBtn = 53;
int launchBtn = 51;
int resetBtn = 48;

//pins for LEDs
int blueLED = 45;
int greenLED = 43;
int redLED = 41;

//motor spin time global varible
double motorSpinTime;

//helps control DC motor
class Motor {
  public:
    int dir1;
    int dir2;
    int speedPin;
    int motorSpeed;  
    
    Motor(int d1, int d2, int sPin, int mSpeed) {
      dir1 = d1;
      dir2 = d2;
      speedPin = sPin;
      motorSpeed = mSpeed;
    };

    void spinForward() {   //this is supposed to be clockwise
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
      analogWrite(speedPin, motorSpeed);
    }

    void spinBack() {    //this is supposed to be counter-clockwise
      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
      analogWrite(speedPin, motorSpeed);
    }

    void off() {
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, LOW);
    }

    double getMotorPBT(double d) {
      double t = d / MOTOR_MS;
      //Serial.print("Pullback time --> ");
      //Serial.print(t);
      return t;
    }
};

double getPBD(double distance, double sine_degrees)
{
    double m, g, k, val, top, bottom, inside, final, finalCm, finalPoint;
    
    m = 0.0101;
    g = 9.8;
    k = 26.3752318;
    val = PI / 180.0; //USED TO CONVERT sine_degrees TO DEGREES

    top = distance * g * m;
    
    sine_degrees = sin((2 * sine_degrees) * val);

    bottom = k * sine_degrees;
    inside = top / bottom;

    final = sqrt(inside); //THIS IS DISTANCE IN METERS
    finalCm = final * 100;                   //THIS IS DISTANCE IN CENTIMETERS
    finalPoint = finalCm + INTIALPOINT;      //THIS IS DISTANCE + INTIAL POINT

    //printf("pullback cm final point @ [ %lf cm ]", finalPoint);
    return final; //can return this, finalCm or finalPoint
}

//OBJECT DECLARATIONS
NewPing sensor(TRIG_PIN, ECHO_PIN);
Motor dcMotor(10,9,11,255);
Servo servo; 

int servoMin = 0;
int servoMax = 180;

void setup() {
  
  servo.attach(5);
  servo.write(servoMax);
  pinMode(mDistBtn, INPUT_PULLUP);
  pinMode(launchBtn, INPUT_PULLUP);
  pinMode(resetBtn, INPUT_PULLUP);
  pinMode(blueLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  Serial.begin(9600);
  
}

void loop() {
  
  //gets distance in cm (with decimals) and converts to m, finds pullback distance
  if (digitalRead(mDistBtn) == LOW && mState == true) {
    duration = sensor.ping();
    measuredDistance = (duration / 2) * 0.0343;
    //measuredDistance = 5000; //TEST AND DEBUDDING PURPOSES
    measuredDistanceM = measuredDistance / 100;    //converts cm --> m
    double pbd = getPBD(measuredDistanceM, 40.00); //sends to distance calc with angle of 40.00 degrees
    double pb_time = dcMotor.getMotorPBT(pbd);
    motorSpinTime = pb_time;
    
    digitalWrite(blueLED, HIGH);
    Serial.print("Distance cm --> ");
    Serial.print(measuredDistance);
    Serial.print("  ||  Distance m --> ");
    Serial.print(measuredDistanceM);
    Serial.print(" || PBD --> [ ");
    Serial.print(pbd);
    Serial.print(" ]");
    Serial.print('\n');
    Serial.print("Motor pullback time --> ");
    Serial.print(pb_time);
    Serial.print('\n');
    delay(500);
    digitalWrite(blueLED, LOW);
    mState = false;
  }

  if (digitalRead(launchBtn) == LOW && launchState == false) {
    Serial.println("Beginning to spin..");
    Serial.println(motorSpinTime);
    tSpin = motorSpinTime * 1000;
    
    Serial.print("tSpin: ");
    Serial.print(tSpin);    //debugging
    Serial.println(" ms");
    
    digitalWrite(greenLED, HIGH);
    
    while (spinState != false) {
      dcMotor.spinForward();
      delay(tSpin);
      dcMotor.off();
      spinState = false;
    }
    
    Serial.println("Done spinning");
    Serial.println("Going to launch servo");
    delay(2000);
    servo.write(servoMin);
    Serial.print("Launched servo");
    delay(500);
    digitalWrite(greenLED, LOW);
    launchState = true;
  }

  //resets all changed variables
  if (digitalRead(resetBtn) == LOW && launchState == true) {
    digitalWrite(redLED, HIGH);
  
    Serial.println("Spinning motor back: ");
    Serial.print(tSpin);
    Serial.print(" s\n");
    while (spinBackState == false) {
      dcMotor.spinBack();
      delay(tSpin);
      dcMotor.off();
      spinBackState = true;
    }
    
    delay(2000);
    Serial.println("Going to reset servo");
    servo.write(servoMax);
    Serial.println("Reset Servo");
    
    mState = true;
    spinState = true;
    launchState = false;
    spinBackState = false;
    motorSpinTime = 0;
    tSpin = 0;
    delay(500);
    digitalWrite(redLED, LOW);
    Serial.println("RESET..");
  }
}
