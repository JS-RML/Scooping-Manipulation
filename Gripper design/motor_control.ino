#include <Servo.h> 
Servo servo1;
Servo servo2;

void setup(){
  Serial.begin(57600);
  pinMode(8, OUTPUT);
  pinMode(6, OUTPUT);
  servo1.attach(8); 
  servo2.attach(6);
}
char var;
void loop(){
  if (Serial.available() > 0)
  {
    char str0 = Serial.read();
    delay(0.5);
    if(str0=='1')
      {
        delay(2);
        String comdata = "";
        while (Serial.available() > 0)
        {
          comdata += char(Serial.read());
          delay(2);
        }
        int servo1_value = comdata.toInt();
        servo1.write(servo1_value);
        delay(100);
      }
    if(str0=='2')
      {
        delay(2);
        String comdata = "";
        while (Serial.available() > 0)
        {
          comdata += char(Serial.read());
          delay(2);
        }
        int servo2_value = comdata.toInt();
        servo2.write(servo2_value);
        delay(100);
      }
  }
}
