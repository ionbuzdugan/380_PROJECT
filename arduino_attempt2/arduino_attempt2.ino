#include <Servo.h>

//servo declarations
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;
Servo servo_6;
Servo esc;
int s,com,curMotor,k, outOfRange;

//initial servo positions
const int init1 = 0;
const int init2 = 179;
const int init3 = 0;
const int init4 = 179;
const int init5 = 0;
const int init6 = 179;

//define the initial postions of the servos
int pos1 = init1;
int pos2 = init2;
int pos3 = init3;
int pos4 = init4;
int pos5 = init5;
int pos6 = init6;
int angles[6]={0,0,0,0,0,0};

void setup() {
  Serial.begin(9600);
  servo_1.attach(3);
  servo_2.attach(5);
  servo_3.attach(6);
  servo_4.attach(9);
  servo_5.attach(10);
  servo_6.attach(11);
  esc.attach(2,1000,2000);

  esc.write(0);
  servo_1.write(pos1);
  servo_2.write(pos2);
  servo_3.write(pos3);
  servo_4.write(pos4);
  servo_5.write(pos5);
  servo_6.write(pos6);
  

  delay(2000);
}

void loop() {
  k=0;
  esc.write(80);
  while(k<6){
    while(k<6 && Serial.available()){
      angles[k]=Serial.read();
      Serial.println(angles[k]);
      k++;
    }
  }

  outOfRange=0;
  for(int i=0;i<6;i++){
    if(angles[i]>180 || angles[i]<0){
      outOfRange++;
    }
  }

  if(!outOfRange){
    servo_1.write(angles[0]);
    servo_2.write(angles[1]);
    servo_3.write(angles[2]);
    servo_4.write(angles[3]);
    servo_5.write(angles[4]);
    servo_6.write(angles[5]);
  }
}
