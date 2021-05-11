/*
int speedPin = 11;
int dir1 = 10;
int dir2 = 9;
int mSpeed = 255;
*/
int onBtn = 53;
int offBtn = 51;

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

    void spinForward(int t) {   //this is supposed to be clockwise
      t = t * 1000;             //converts seconds to milliseconds
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
      analogWrite(speedPin, motorSpeed);
    }

    void spinBack(int t) {    //this is supposed to be counter-clockwise
      t = t * 1000;           //converts seconds to milliseconds
      digitalWrite(dir1, HIGH);
       digitalWrite(dir2, LOW);
      analogWrite(speedPin, motorSpeed);
    }
};

Motor dcMotor(10,9,11,255);
int i = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT); 
  pinMode(onBtn, INPUT_PULLUP);
  pinMode(offBtn, INPUT_PULLUP);

  Serial.begin(9600);  
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH );
  analogWrite(speedPin, mSpeed);
  */

  if (digitalRead(onBtn) == LOW) {
    dcMotor.spinForward(5000);
  }

  if (digitalRead(offBtn) == LOW) {
    dcMotor.spinBack(5); 
  }
}
