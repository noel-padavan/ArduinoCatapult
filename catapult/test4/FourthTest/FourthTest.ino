#include <DFRobot_TFmini.h>
#include <math.h>
#include <Servo.h>

//motor constants
#define RPM 200
#define MOTOR_RAD 0.0015
#define MOTOR_MS 0.03142 //with radius of 0.0015m and RPM of 200

//For distance sensor
#define TX 52
#define RX 50

//For pullback calculations
#define INTIALPOINT 16.2
#define PI 3.14159265

//Btn Pin #'s
#define MD_BTN 13
#define LAUNCH_BTN 12
#define RESET_BTN 11

//LED Pin #'s
#define RED_LED 8
#define GREEN_LED 9
#define BLUE_LED 10

//Constants for servo rotation in degrees
#define SERVO_MIN 0
#define SERVO_MAX 90

//boolean values for launch security checks
bool mState = true; 
bool spinState = true; 
bool launchState = false; 
bool spinBackState = false;

//Distance measurement variables
double measuredDistance; //change to 750 for tests
double measuredDistanceM;
double duration;

//time for motor spin
double tSpin;
double motorSpinTime;


//Pullback distance calculation. Result in meters
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


//helps control DC motor
class Motor {
  public:
    int in1;
    int in2;

    Motor(int i1, int i2) {
      in1 = i1;
      in2 = i2;
    }

    void spinForward() {   //clockwise
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }

    void spinBack() {     //counter clockwise spin
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }

    void off() {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }

    double getMotorPBT(double d) {
      double t = d / MOTOR_MS;
      //Serial.print("Pullback time --> ");
      //Serial.print(t);
      return t;
    }
};


//Obejct declarations
Motor dcMotor(53, 51);
Servo servo; 

//For TFMini
SoftwareSerial mySerial(TX, RX); // TX, RX

DFRobot_TFmini  TFmini;
uint16_t distance_cm;

//measure distance, launch and reset
class Catapult {
  public:
    static void MEASURE_DISTANCE(double d) {
      measuredDistanceM = d;                          //distance in meters
      measuredDistance = d * 100;                     //distance in cm
      double pbd = getPBD(measuredDistanceM, 40.00);  //sends to distance calc with angle of 40.00 degrees
      double pb_time = dcMotor.getMotorPBT(pbd);
      motorSpinTime = pb_time;
    
      digitalWrite(BLUE_LED, HIGH);
      Serial.println('\n');
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
      digitalWrite(BLUE_LED, LOW);
      mState = false;
    }

    static void LAUNCH() {
      Serial.println("Beginning to spin..");
      Serial.println(motorSpinTime);
      tSpin = motorSpinTime * 1000;
    
      Serial.print("tSpin: ");
      Serial.print(tSpin);    //debugging
      Serial.println(" ms");
    
      digitalWrite(GREEN_LED, HIGH);
    
      while (spinState != false) {
        dcMotor.spinForward();
        delay(tSpin);
        dcMotor.off();
        spinState = false;
      }
    
      Serial.println("Done spinning");
      Serial.println("Going to launch servo");
      delay(1500); //or tSpin + 1000
      servo.write(SERVO_MAX);
      Serial.println("Launched servo");
      delay(200);                             //orginal was 500ms
      digitalWrite(GREEN_LED, LOW);
      launchState = true;
    }

    static void RESET() {
  
      digitalWrite(RED_LED, HIGH);
  
      Serial.println("Spinning motor back: ");
      Serial.print(tSpin);
      Serial.print(" s\n");
      while (spinBackState == false) {
        dcMotor.spinBack();
        delay(tSpin);
        dcMotor.off();
        spinBackState = true;
      }
    
      delay(1000); //or tSpin + 1000
      Serial.println("Going to reset servo");
      servo.write(SERVO_MIN);
      Serial.println("Reset Servo");
    
      mState = true;
      spinState = true;
      launchState = false;
      spinBackState = false;
      motorSpinTime = 0;
      tSpin = 0;
      delay(500);
      digitalWrite(RED_LED, LOW);
      Serial.println("RESET COMPLETE");
      Serial.print('\n');
    }
};


void setup() {
  // put your setup code here, to run once:
  
  servo.attach(6);
  servo.write(SERVO_MIN);
  
  pinMode(MD_BTN, INPUT_PULLUP);
  pinMode(LAUNCH_BTN, INPUT_PULLUP);
  pinMode(RESET_BTN, INPUT_PULLUP);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  
  Serial.begin(115200);
  TFmini.begin(mySerial);
}

void loop() {
  // put your main code here, to run repeatedly:
  if((TFmini.measure() && digitalRead(MD_BTN) != LOW) && mState == true){                     
     distance_cm = TFmini.getDistance();       
     double distance_meters = distance_cm / 100.00;    
     Serial.print("Distance = ");
     Serial.print(distance_cm);
     Serial.println("cm");
     delay(100);
        
     if (digitalRead(MD_BTN) == LOW) {
      Catapult::MEASURE_DISTANCE(distance_meters);
      mState = false;
    }
  } 

  if (digitalRead(LAUNCH_BTN) == LOW && launchState == false) {
    Catapult::LAUNCH();
  }

  if (digitalRead(RESET_BTN) == LOW && launchState == true) {
    Catapult::RESET();
  }
}