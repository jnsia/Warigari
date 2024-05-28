#include <Servo.h>

Servo myServo;

int E2 = 11;
int M2 = 13;
int servo = 7;
int target_servo=90;
int now_servo=90;
int Step = 15;
char Buffer[12];

int value = 0;
String cmd;


void setup()
{
  pinMode(M2, OUTPUT);
  pinMode(servo, OUTPUT);
  
  myServo.attach(servo);
  digitalWrite(M2, HIGH);
  analogWrite(E2, 50);

  delay(2000);
  
  Serial.begin(115200);

  // memset(Buffer, 0, sizeof(Buffer));
}


void loop()
{

  if (Serial.available() > 0) {
    Serial.readBytesUntil('\n', Buffer, sizeof(Buffer));
    String receiveString = String(Buffer);
    memset(Buffer, 0, sizeof(Buffer));
    // Serial.println(receiveString);

    int spaceIdx = receiveString.indexOf(' ');
    cmd = receiveString.substring(0, spaceIdx);
    value = receiveString.substring(spaceIdx+1).toInt();    
    
    switch(cmd[0]){
      case 's':
        target_servo = value;
        break;
      case 'd':
        digitalWrite(M2, HIGH);
        analogWrite(E2, value);
        break;
      //default:
        //Serial.println(cmd[0]);
    }

  }

  if(now_servo > target_servo){
    now_servo = max(now_servo-Step, target_servo);
  }
  else if(now_servo < target_servo){
    now_servo = min(now_servo+Step, target_servo);
  }
  
  myServo.write(now_servo);
}