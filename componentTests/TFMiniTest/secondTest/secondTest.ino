#include <DFRobot_TFmini.h>
#define MD_BTN 6
#define RESET_BTN 5

SoftwareSerial mySerial(8, 7); // RX, TX

DFRobot_TFmini  TFmini;
uint16_t distance_cm,strength;

//my variables
bool resetState = false;
bool dState = false;

//my functions
void useDistance(double d) {
  Serial.print("Distance --> ");
  Serial.print(d);
  Serial.print('\n');
  resetState = true;
}

void setup(){
    Serial.begin(115200);
    TFmini.begin(mySerial);
    pinMode(MD_BTN, INPUT_PULLUP);
    pinMode(RESET_BTN, INPUT_PULLUP);
}

void loop(){
    if((TFmini.measure() && digitalRead(MD_BTN) != LOW) && dState == false){                     
        distance_cm = TFmini.getDistance();       
        double distance_meters = distance_cm / 100.00;    
        //Serial.print("Distance = ");
        //Serial.print(distance);
        //Serial.println("cm");
        delay(100);
        
        if (digitalRead(MD_BTN) == LOW) {
          useDistance(distance_meters);
          dState = true;
        }
    } 

    if (digitalRead(RESET_BTN) == LOW && resetState == true) {
      resetState = false;
      dState = false;
      Serial.println("RESET COMPLETE");
      
    }
}
