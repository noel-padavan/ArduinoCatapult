#include <SR04.h>
#include <Servo.h>

#define ECHOPIN 4
#define TRIGPIN 3

int onPin = 12;
int offPin = 13;
int greenLED = 7;
int redLED = 6;

int servoDelay = 1000;
int servoMax = 0;
int servoMin = 180;

Servo myServo;
SR04 sensor = SR04(ECHOPIN, TRIGPIN);

double distance;

class LED {
  public:
    static void ON(int pinNumber) {
      digitalWrite(pinNumber, HIGH);
    }

    static void OFF(int pinNumber) {
      digitalWrite(pinNumber, LOW);
    }

    static void timedON(int pinNumber, int duration) {
      digitalWrite(pinNumber, HIGH);
      delay(duration);
      digitalWrite(pinNumber, LOW);
    }
};

class Catapult {
  
  public:
    static void launch() {
      myServo.write(servoMax);
      LED::OFF(redLED);
      LED::timedON(greenLED, 500);
    }
    
    static void reset() {
      myServo.write(servoMin);
      LED::OFF(greenLED);
      LED::timedON(redLED, 500);
    }
};


void setup() {
    myServo.attach(9);
    pinMode(onPin, INPUT_PULLUP);
    pinMode(offPin, INPUT_PULLUP);
    pinMode(greenLED, OUTPUT);
    pinMode(redLED, OUTPUT);
    Serial.begin(9600);
    delay(1000);
    pinMode(2, OUTPUT);
}

void loop() {
    /*
    if (digitalRead(onPin) == LOW) {
      Catapult::launch();
    }  

    if (digitalRead(offPin) == LOW) {
      Catapult::reset();
    }
    */

    //this prints out the distance readings to the serial monitor
    digitalWrite(2, HIGH);
    distance = sensor.Distance();
    Serial.println(distance);
    delay(50);

    //this is the control version that relies on hand distance to sesnor
    if (distance == 15.00) {
      Catapult::launch();
    }

    if (distance == 10.00) {
      Catapult::reset();
    }
    
    
}
