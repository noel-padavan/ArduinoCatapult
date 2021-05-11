#include <LiquidCrystal.h>
#include "SR04.h"
#include <math.h>

#define TRIG 6
#define ECHO 5
#define INTIALPOINT 16.2
#define PI 3.14159265


int rs = 7;
int en = 8;
int d4 = 9;
int d5 = 10;
int d6 = 11;
int d7 = 12;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
SR04 distanceSensor = SR04(ECHO,TRIG);

double mainCalc(double distance, double sine_degrees)
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


void setup() {
  // put your setup code here, to run once:
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(9600);
  
  lcd.begin(16,2); //Tell Arduino to start your 16 column 2 row LCD
  lcd.setCursor(0,0);  //Set LCD cursor to upper left corner, column 0, row 0
  //lcd.print("Target Distance:");  //Print Message on First Row
}

void loop() {
  // put your main code here, to run repeatedly:
  double distance;
  distance = distanceSensor.Distance();

  distance = distance / 100; //cm to m
  double pbd = mainCalc(distance, 40.00);
  pbd = pbd * 100; //m to cm

  //ORIGINAL
  //lcd.setCursor(0,1);  //Set cursor to first column of second row
  //lcd.print("                "); //Print blanks to clear the row
  //lcd.setCursor(0,1);   //Set Cursor again to first column of second row
  //lcd.print(distance); //Print measured distance
  //lcd.print(" cm");  //Print your units.
  //delay(500); //pause to let things settle

  lcd.print("DIST: ");
  lcd.print(distance);
  lcd.print(" m");
  lcd.setCursor(0,1);
  lcd.print("PBD: ");
  lcd.print(pbd);
  lcd.print(" cm");
  delay(1000);
  lcd.clear();
}
