//including libraries used in the code
#include <NewPing.h>
#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 gyro;
Servo myservo;
//setting sensors pins
#define Fe A0
#define Ft 5
#define R1e A1
#define R1t 6
#define R2e A2
#define R2t 7
#define L1e A3
#define L1t 8
#define L2e A4
#define L2t 9
#define M 3
#define M_1 2
#define M_2 4
#define Be A5
#define Bt 10

//setting variables 
unsigned long start = 0, endd = 0 , start1 = 0;
float F, R1, R2, L1, L2, B;   //ultrasonic distance 
float Ang_r = 0, Ang_l = 0, P_Ang_r = 0, P_Ang_l = 0;
float kp = 1.5, kpr = 1.45;
float dis_m = 14., dis = 14.;     //the border deadline 
int it = 0;
float R, L;                                                                         //distance from the wall
const float MAX = 200., d = 14., ang = 32., ang_1 = 30., ang_2 = 20., ang_3 = 45, ang_m = 17.; //distance between sensors 
const float I_pos = 80, L_min, pos = 80;      //servo middle angle
bool Clock = true;
NewPing sonar_f(Ft, Fe, MAX);
NewPing sonar_r1(R1t, R1e, MAX);
NewPing sonar_r2(R2t, R2e, MAX);
NewPing sonar_l1(L1t, L1e, MAX);
NewPing sonar_l2(L2t, L2e, MAX);
NewPing sonar_b(Bt, Be, MAX);
float angle = 0.0, pangle = 0.0;

//calculate ultrasonic distances
void Sensor() {
  F = sonar_f.ping() * 0.0343 / 2.;
  delay(5);
  if (F == 0) F = 200;
  R1 = sonar_r1.ping() * 0.0343 / 2. ;
  delay(5);
  if (R1 == 0) R1 = 200;
  L1 = (sonar_l1.ping() * 0.0343 / 2.);
  delay(5);
  if (L1 == 0) L1 = 200;
  R2 = (sonar_r2.ping() * 0.0343 / 2.);
  delay(5);
  if (R2 == 0) R2 = 200;
  L2 = (sonar_l2.ping() * 0.0343 / 2.) - 1.7;
  delay(5);
  if (L2 == 0) L2 = 200;
  B = (sonar_b.ping() * 0.0343 / 2.) ;
  delay(5);
  if (B == 0) B = 200;
}
//calculating the robot angle

void Angle_l(int L1, int L2) {
  float a = (L1 - L2) / d;
  Ang_l = -1 * atan(a) * 180 / 3.14;
  //if (abs(Ang_l - P_Ang_l) > 10) Ang_l = P_Ang_l;
  L = abs(((L1 + L2) / 2) * cos(atan((L1 - L2) / d)));
  if (Ang_l > ang_m) Ang_l = ang_m;
  else if (Ang_l < -ang_m) Ang_l = -ang_m;
  //P_Ang_l = Ang_l;
}
void Angle_r(int R1, int R2) {
  float a = ((R1 - R2) / d);
  Ang_r = atan(a) * 180 / 3.14;
  //if (abs(Ang_r - P_Ang_r) > 10) Ang_r = P_Ang_r;
  R = abs(((R1 + R2) / 2) * cos(atan((R1 - R2) / d)));
  if (Ang_r > ang_m) Ang_r = ang_m;
  else if (Ang_r < -ang_m) Ang_r = -ang_m;
  //P_Ang_r = Ang_r;
}
//movement functions 
void Distl(float a) {
  myservo.write((a - dis) * kp + I_pos );
}
void Distr(float a) {
  myservo.write(-1 * (a - dis) * kp + I_pos );
}
void forward(int a) {
  digitalWrite(M_1, LOW);
  digitalWrite(M_2, HIGH);
  analogWrite(M, a);
  delay(5);
}
void back(int a, int b) {
  digitalWrite(M_1, HIGH);
  digitalWrite(M_2, LOW);
  analogWrite(M, a);
  delay(b);
}
void stopp(int a) {
  digitalWrite(M_1, LOW);
  digitalWrite(M_2, LOW);
  analogWrite(M, 0);
  delay(a);
}


void setup() {
  // setup pins, to run once:
  pinMode(M, OUTPUT);
  pinMode(M_1, OUTPUT);
  pinMode(M_2, OUTPUT);
  pinMode(Fe, INPUT);
  pinMode(2, INPUT);
  myservo.attach(11);
  myservo.write(pos);
  delay(10);
  Serial.begin(9600);
  Sensor();
  //direction detection 
  while (R1 < 100 && L1 < 100) {
    Sensor();
    if (F < 110) forward(150);
    else
      forward(250);
    Angle_l(L2, L1);
    Angle_r(R1, R2);
    
    //turn away of side wall if the vehicle is in the danger zone
    if (L < dis ) {
      Distl(L);
    }
    else if (R < dis) {
      Distr(R);
    }
    else {
      //angle follow
      myservo.write(I_pos + Ang_l);
    }

  } endd = millis();
  //turn right
  if ((R1 >= 80) && L1 < 80|| ((F < 50) && L1 < 80)) {

    //decrease speed when approaching to the wall
    if (F < 110) forward(150);
    else
      forward(250);
    myservo.write(I_pos - 38);
    delay(400);
    start = millis();
    Clock = true;
  }
  //turn left
  if ((L1 >= 80) && R1 < 80 || ((F < 50) && R1 < 80))  {
    if (F < 110) forward(150);
    else
      forward(250);
    myservo.write(I_pos + 38);
    delay(550);
    start = millis();
    Clock = false;
  }

}
void loop() {
  Sensor();
  if (F < 110) forward(150);
  else
    forward(250);

  //clockwise round
  while (Clock) {
    while (it < 11) {
      endd = millis();
      //turn mode
      if ((R1 >= 80) && abs(endd - start) > 1200 || ((F < 50) && abs(endd - start) > 1200)) {
        it++;
        if (F < 110) forward(150);
        else
          forward(250);
        myservo.write(I_pos - 38);
        delay(480);
        start = millis();
      }

      if (F < 110) forward(150);
      else
        forward(250);
      Sensor();
      Angle_l(L2, L1);
      Angle_r(R1, R2);

      if (L < dis ) {
        Distl(L);
      }
      else if (R < dis) {
        Distr(R);
      }
      else {
        myservo.write(I_pos + Ang_l);
        ///
      }
    }
    { forward(250);
      start1 = millis();
      Serial.println(start);
      Serial.println(endd);
      endd = millis();
      Sensor();
      Angle_l(L2, L1);
      Angle_r(R1, R2);

      if (L < dis) {
        Distl(L);
      }
      else if (R < dis) {
        Distr(R);
      }
      else {
        myservo.write(I_pos + Ang_l);
      }
      if (B > 110 && abs(Ang_l < 5) && F < 180) {
        //end of the round
        stopp(5);
        back(255, 50);
        myservo.write(I_pos);
        stopp(1000000);
      }

    }
  }
  //unclockwise round
  while (!Clock) {
    while (it < 11) {
      endd = millis();
      if ((L1 >= 80) && abs(endd - start) > 1200 || ((F < 50) && abs(endd - start) > 1200)) {
        it++;
        if (F < 110) forward(150);
        else
          forward(250);
        myservo.write(I_pos + 38);
        delay(650);
        start = millis();
      }

      if (F < 110) forward(150);
      else
        forward(250);
      Sensor();
      Angle_l(L2, L1);
      Angle_r(R1, R2);

      if (L < dis ) {
        Distl(L);
      }
      else if (R < dis) {
        Distr(R);
      }
      else {
        myservo.write(I_pos - Ang_r);
      }
    }
    { forward(250);
      start1 = millis();
      Serial.println(start);
      Serial.println(endd);
      endd = millis();
      Sensor();
      Angle_l(L2, L1);
      Angle_r(R1, R2);

      if (L < dis) {
        Distl(L);
      }
      else if (R < dis) {
        Distr(R);
      }
      else {
        myservo.write(I_pos - Ang_r);
      }
      if (B > 110 && abs(Ang_r < 5) && F < 180) {
        //end of the round
        stopp(5);
        back(255, 50);
        myservo.write(I_pos);
        stopp(1000000);
      }

    }
  }
}
// The End Of The First Day Code//