#define RED_LED 46
#define GREEN_LED 50
#define BLUE_LED 52

#define MD_BTN 53
#define LAUNCH_BTN 51
#define RESET_BTN 49

bool md_state = false;
bool launch_state = false;
bool reset_state = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(MD_BTN, INPUT_PULLUP);
  pinMode(LAUNCH_BTN, INPUT_PULLUP);
  pinMode(RESET_BTN, INPUT_PULLUP);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(MD_BTN) == LOW && md_state == false) {
    digitalWrite(BLUE_LED, HIGH);
    delay(500);
    digitalWrite(BLUE_LED, LOW);
    Serial.println("PRESSED M.D BTN");
    md_state = true;
  }

  if (digitalRead(LAUNCH_BTN) == LOW && launch_state == false) {
    digitalWrite(GREEN_LED, HIGH);
    delay(500);
    digitalWrite(GREEN_LED, LOW);
    Serial.println("PRESSED LAUNCH BTN");
    launch_state = true;
    reset_state = true;
  }

  if (digitalRead(RESET_BTN) == LOW && reset_state == true) {
    digitalWrite(RED_LED, HIGH);
    delay(500);
    digitalWrite(RED_LED, LOW);
    Serial.println("PRESSED RESET BTN");
    md_state = false;
    launch_state = false;
    reset_state = false;
  }

}
