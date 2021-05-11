#define DIR1 2
#define DIR2 3

void setup() {
  // put your setup code here, to run once:
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //CLOCKWISE SPIN
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, HIGH);
  delay(2000);
  //COUNTER-CLOCKWISE SPIN
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);
  delay(2000);
}
