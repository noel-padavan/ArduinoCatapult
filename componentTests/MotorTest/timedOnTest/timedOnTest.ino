int speedPin = 11;
int dir1 = 10;
int dir2 = 9; 
int mSpeed = 255;
bool state = true;

void turnOn(int t) {
  t = t * 1000;
  while (state != false) {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    analogWrite(speedPin, mSpeed);
    delay(t);
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
    state = false;
  }
};

void setup() {
  // put your setup code here, to run once:
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT); 
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  turnOn(5);
}
