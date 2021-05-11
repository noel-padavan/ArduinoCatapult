int speedPin = 5;
int dir1 = 4;
int dir2 = 3;
int mSpeed = 255;

void setup() {
  // put your setup code here, to run once:
  int(speedPin, OUTPUT);
  int(dir1, OUTPUT);
  int(dir2, OUTPUT); 
  Serial.begin(9600);  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH );
  analogWrite(speedPin, mSpeed);

}
