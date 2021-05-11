#define MOTOR_MS 0.047

class Motor {
  public:
    int in1;
    int in2;

    Motor(int i1, int i2) {
      in1 = i1;
      in2 = i2;
    }

    void spinForward() {   //clockwise
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }

    void spinBack() {     //counter clockwise spin
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }

    void off() {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }

    double getMotorPBT(double d) {
      double t = d / MOTOR_MS;
      //Serial.print("Pullback time --> ");
      //Serial.print(t);
      return t;
    }
};

Motor motor(53, 51);

void setup() {
  // put your setup code here, to run once:

}
void loop() {
  // put your main code here, to run repeatedly:
  motor.spinForward();
}
