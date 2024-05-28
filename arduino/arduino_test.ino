#include <Servo.h>

Servo myServo;

int E2 = 11;
int M2 = 13;
int servo = 7;

void setup()
{
  pinMode(M2, OUTPUT);
  pinMode(servo, OUTPUT);
  
  myServo.attach(servo);
  
  Serial.begin(9600);
}

void loop()
{
  int value;
  int receiveValue = 0;

  if (Serial.available() > 0) {
    receiveValue = Serial.read();
    Serial.println(receiveValue);

    digitalWrite(M2, LOW);
    analogWrite(E2, 160);
    myServo.write(receiveValue);

    if (receiveValue == 0) {
      analogWrite(E2, 0);
    }
  }
}