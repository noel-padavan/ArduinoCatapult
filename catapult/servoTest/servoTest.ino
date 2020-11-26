#include <Servo.h>

int onPin = 12;
int offPin = 13;
int greenLED = 7;
int redLED = 6;

int servoDelay = 1000;
int servoMax = 0;
int servoMin = 180;

Servo myServo;

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
}

void loop() {
    if (digitalRead(onPin) == LOW) {
      Catapult::launch();
    }  

    if (digitalRead(offPin) == LOW) {
      Catapult::reset();
    }
}
