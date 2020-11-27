#include "SR04.h"
#define ECHOPIN 4
#define TRIGPIN 3

SR04 sensor = SR04(ECHOPIN, TRIGPIN);
double distance;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  pinMode(2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(2, HIGH);
  distance = sensor.Distance();
  Serial.println(distance);
  delay(50);
}
