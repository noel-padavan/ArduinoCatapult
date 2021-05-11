#include <DFRobot_TFmini.h>

#define SIZE 10
#define MD_BTN 5
#define RESET_BTN 4

SoftwareSerial mySerial(8, 7); // TX GOES TO PIN 8 and RX (WHITE) GOES TO PIN 7 

DFRobot_TFmini  TFmini;
uint16_t distance,strength;

//variables I added
bool state = false;
bool resetState = false;

int i = 0;
double data[SIZE] = {};

//function for printing values stored in data array
void printArr() {
  int i;
  for (i = 0; i < SIZE; i++) {
    Serial.print("@ index: ");
    Serial.print(i);
    Serial.print(" --> ");
    Serial.print(data[i]);
    Serial.print('\n');
  }
}


void setup(){
    Serial.begin(115200);
    TFmini.begin(mySerial);
    pinMode(MD_BTN, INPUT_PULLUP);
    pinMode(RESET_BTN, INPUT_PULLUP);
}

void loop(){

    /*
    if(TFmini.measure()){                      //Measure Distance and get signal strength
        distance = TFmini.getDistance();       //Get distance data
        strength = TFmini.getStrength();       //Get signal strength data
        Serial.print("Distance = ");
        Serial.print(distance);
        Serial.println("cm");
        //Serial.print("Strength = ");
        //Serial.println(strength);
        delay(10);
    }
    delay(10);
    */



    /*
     * -My Version-
     * 
     */
     
    if ((digitalRead(MD_BTN) == LOW) && (state == false && TFmini.measure())) {
      while(i < SIZE) {
        distance = TFmini.getDistance();
        double distance_float = distance;
        double distanceFloat_meters = distance / 100.00;     //converts the uint16_t to a double type and to meters
        Serial.print("distance #: ");
        Serial.print(i);
        Serial.print(" --> ");
        Serial.print(distance_float);
        Serial.print(" cm");
        Serial.print('\n');
        data[i] = distanceFloat_meters;

        delay(200);
        i++; 
      }

      printArr();
      state = true;
      resetState = true;
    }

   if (digitalRead(RESET_BTN) == LOW && resetState == true) {
      delay(100);
      state = false;
      Serial.println("\nRESET COMPLETE\n");
      resetState = false;
   }
}
