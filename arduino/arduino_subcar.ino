#include <SoftwareSerial.h> 
#include <AFMotor.h>
#include <Servo.h>
 
AF_DCMotor motor_1(3);
Servo myServo;

int target_servo=80;
int now_servo=80;
char Buffer[12];

int value = 0;
String cmd;

void setup() {
  myServo.attach(10);
  motor_1.setSpeed(160);
  motor_1.run(FORWARD);
  Serial.begin(115200);
}
 
void loop() {
  if (Serial.available() > 0) {
    Serial.readBytesUntil('\n', Buffer, sizeof(Buffer));
    String receiveString = String(Buffer);
    memset(Buffer, 0, sizeof(Buffer));
//    Serial.println(receiveString);
    
    int spaceIdx = receiveString.indexOf(' ');
    cmd = receiveString.substring(0, spaceIdx);
    value = receiveString.substring(spaceIdx+1).toInt();    
    
    switch(cmd[0]){
      case 's':
        target_servo = value;
        break;
      case 'd':
        motor_1.run(FORWARD);
        motor_1.setSpeed(value);
        break;
      default:
        Serial.println(cmd[0]);
    }

   }
    if(now_servo > target_servo){
      now_servo = max(target_servo, now_servo-10);
    }
    else if(now_servo < target_servo){
      now_servo = min(target_servo, now_servo+10);
    }
    myServo.write(now_servo);
}