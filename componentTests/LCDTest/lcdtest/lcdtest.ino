#include <LiquidCrystal.h>
#include "SR04.h"
#define TRIG 6
#define ECHO 5

int rs = 7;
int en = 8;
int d4 = 9;
int d5 = 10;
int d6 = 11;
int d7 = 12;


LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
SR04 distanceSensor = SR04(ECHO,TRIG);
long a;

/*
int rs = 7;
int en = 8;
int d4 = 9;
int d5 = 10;
int d6 = 11;
int d7 = 12;

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16,2);
  Serial.begin(9600);
  delay(1000);
   
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  lcd.setCursor(0, 0);
  
  //lcd.print("Hello");
  //lcd.setCursor(0,1);
  //lcd.print("There");
  
  //lcd.print("PBD: "); //XAMPLE OF PRINTING NUMBERS
  //lcd.print(8);
  //lcd.print(" D: ");
  //lcd.print(17.8);
    a=sr04.Distance();
    Serial.print(a);
    Serial.println("cm");
    delay(200);

  
}
*/

//THIS IS FOR READING A DISTANCE FROM THE ULTRASONIC SENSOR AND THEN PRINTING IT OUT TO LCD
void setup() 
{  
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  lcd.begin(16,2); //Tell Arduino to start your 16 column 2 row LCD
  lcd.setCursor(0,0);  //Set LCD cursor to upper left corner, column 0, row 0
  lcd.print("Target Distance:");  //Print Message on First Row
}

void loop() {
  /*
  long duration, distance;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = (duration/2) / 29.1;
  */
  long distance;
  distance = distanceSensor.Distance();

  lcd.setCursor(0,1);  //Set cursor to first column of second row
  lcd.print("                "); //Print blanks to clear the row
  lcd.setCursor(0,1);   //Set Cursor again to first column of second row
  lcd.print(distance); //Print measured distance
  lcd.print(" cm");  //Print your units.
  delay(500); //pause to let things settle
}
