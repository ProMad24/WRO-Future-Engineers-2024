#include <NewPing.h>
#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 gyro;

Servo myservo;
Servo fservo;
#define Fe A6
#define Ft 22
#define R1e A3
#define R1t 4
#define R2e A2
#define R2t 30
#define L1e A0
#define L1t 12
#define L2e A1
#define L2t 31
#define Ml 11
#define M_2 8
#define M_1 10
#define Mr 9
int gy =2;
float psa = 0;
float F, R1, R2, L1, L2;
float Ang_r = 0, Ang_l = 0, P_Ang_r = 0, P_Ang_l = 0;
float kp = 1.7, kpr = 1.7;
float dis = 17.;
unsigned long start = 0, endd = 0, start1 = 0, start2 = 0,dlys=0,dlye=0;
int it = 1;
float R, L;
char u = 'W';
//هدةل هني مسافة الروبوت غن الحيط
const float MAX = 200., d = 15., ang = 32., ang_1 = 40., ang_2 = 20., ang_3 = 45, ang_m = 17.;  //هاد مشان البعد بين الحساسين
const float I_pos = 70, I_posf = 90., I_pos_1 = 70, pos = 70;
bool Clock = true, set=false;
NewPing sonar_f(Ft, Fe, MAX);
NewPing sonar_r1(R1t, R1e, MAX);
NewPing sonar_r2(R2t, R2e, MAX);
NewPing sonar_l1(L1t, L1e, MAX);
NewPing sonar_l2(L2t, L2e, MAX);
float angle = 0.0, pangle = 0.0;
char Data = 'a', pData = ' ', t = 'F', sData = 'W', xr = 'a', xg = 'a';
const float gyroScale = 131.0;
unsigned long prevTime;
void Gyro() {
  int16_t gyroX, gyroY, gyroZ;

  gyro.getRotation(&gyroX, &gyroY, &gyroZ);


  unsigned long currentTime = micros();
  float elapsedTime = (currentTime - prevTime) / 1000000.0;  // Convert to seconds
  prevTime = currentTime;

  float gyroRateZ = gyroZ / gyroScale;

  angle += gyroRateZ * elapsedTime + elapsedTime * 1.513;
  pangle=angle;
//  Serial.println(angle);
  if(gy==it){
  if (Clock) {
    angle= angle + 90;
  }
  

  else angle = angle - 90;
  gy++;}
  //else          angle = -fmod(-angle,90.);
  //Serial.println(pangle);
}

void Sensor() {
  Gyro();
  F = sonar_f.ping() * 0.0343 / 2.;
  delay(5);
  if (F == 0) F = 200;
  R1 = sonar_r1.ping() * 0.0343 / 2.;
  delay(5);
  if (R1 == 0) R1 = 200;
  L1 = sonar_l1.ping() * 0.0343 / 2.;
  delay(5);
  if (L1 == 0) L1 = 200;
  R2 = (sonar_r2.ping() * 0.0343 / 2.);  //1.2
  delay(5);
  if (R2 == 0) R2 = 200;
  L2 = (sonar_l2.ping() * 0.0343 / 2.);  //-2.7
  delay(5);
  if (L2 == 0) L2 = 200;
}

void Angle_r(int R1, int R2) {
  Gyro();
  float a = ((R1 - R2) / d);
  Ang_r = atan(a) * 180 / 3.14;
  Ang_r = Ang_r * 0.9;
  R = ((R1 + R2) / 2) * cos(atan((R1 - R2) / d));
  if (Ang_r > ang) Ang_r = ang;
  else if (Ang_r < -ang) Ang_r = -ang;
}
void Angle_l(int L1, int L2) {
  Gyro();
  float a = (L1 - L2) / d;
  Ang_l = -1 * atan(a) * 180 / 3.14;
  L = ((L1 + L2) / 2) * cos(atan((L1 - L2) / d));
  if (Ang_l > ang) Ang_l = ang;
  else if (Ang_l < -ang) Ang_l = -ang;
}

void Distl(int a) {
  Gyro();
  myservo.write((a - dis) * -1 * kp + I_pos);
}
void Distr(int a) {
  Gyro();
  myservo.write((a - dis) * kpr + I_pos);
}
void forward(int a, int b) {
  Gyro();
  digitalWrite(Ml, HIGH);
  digitalWrite(Mr, HIGH);
  analogWrite(M_1, a);
  analogWrite(M_2, 0);
  dly(b);
}
void back(int a, int b) {
  Gyro();
  digitalWrite(Ml, HIGH);
  digitalWrite(Mr, HIGH);
  analogWrite(M_1, 0);
  analogWrite(M_2, a);
  dly(b);
}
void stopp(int a) {
  Gyro();
  digitalWrite(Ml, HIGH);
  digitalWrite(Mr, HIGH);
  analogWrite(M_1, 0);
  analogWrite(M_2, 0);
  dly(a);
}

void pass(float a) {
  Gyro();
  psa = atan((a - 10.) / 40.) * 180 / 3.14;
  if (psa > 50)
    psa = 50;
    else if(psa<0)
    psa=50;
}
void ser() {
  Gyro();
  Data = Serial.read();
  if (Data != 'W' && Data != 'G' && Data != 'R')
    Data = 'W';
}

void whitel() {

  forward(165, 5);
  Sensor();
  Angle_l(L1, L2);
  Gyro();
  if (L1 >= dis && R1 >= dis) {
    //
    myservo.write(I_pos_1 + Ang_l);

    //
    if (abs(Ang_l) < 3) angle = 0.;

  }

  else if (R1 < dis) {

    Distr(R1);
  } else if (L1 < dis) {
    Distl(L1);
  }
}

void whiter() {
  forward(165, 5);
  Sensor();
  Angle_r(R1, R2);
  Gyro();
  if (L1 >= dis && R1 >= dis) {

    myservo.write(I_pos + Ang_r);
    if (abs(Ang_r) < 3) angle = 0.;
  } else if (R1 < dis) {

    Distr(R1);
  } else if (L1 < dis) {
    Distl(L1);
  }
}
void whitelclock() {
  forward(165, 5);
  Sensor();
  Angle_l(L1, L2);
  Gyro();

  if (it < 13) {

    if (L1 >= dis && R1 >= dis) {
      //
      myservo.write(I_pos_1 + Ang_l);

      //
      if (abs(Ang_l) < 3) angle = 0.;

    } else if (R1 < dis) {

      Distr(R1);
    } else if (L1 < dis) {
      Distl(L1);
    }
    if (R1 > 100) {
      Serial.print('e');
      //      stopp()
      //      ser();
      //      if(Data=='R')
      //      Red();
      //      else if(Data=='G')
      //      Green();
      pData = 'W';
      xr = 'l';
      xg='m';


      endd = millis();
      if ((endd - start) > 5000||set) {
        it++;
        Gyro();
        //stopp(1000);
        forward(165, 5);
        Sensor();

        while (F > 95) {

          Sensor();
          whitel();
        }
        while(F<80){
          back(165,5);
          Sensor();
          Angle_l(L1,L2);
          myservo.write(I_pos-Ang_l);
          }
       // angle = 90.;
        
        Serial.print('n');

        forward(165, 5);
       
        Sensor();
        Angle_l(L1, L2);
        
         Gyro();
        while (angle >7 ) {
          myservo.write(I_pos_1 + ang_1);
          Gyro();
        }
        Sensor();
        Angle_l(L1,L2);
        while(abs(Ang_l)>10){
          Sensor();
        Angle_l(L1,L2);
        myservo.write(I_pos+ang_1);
        }
        //angle = 0.;
        myservo.write(I_pos);
        Serial.print('e');
        stopp(200);
        forward(165, 5);
        myservo.write(I_pos);
        ser();
        xg='m';
        
        if(set){
          Sensor();
          Angle_l(L1,L2);
          endd=millis();
          start2=millis();
          stopp(200);
          back(165,5);
        while(endd-start2<3000){
          ser();
          if(Data=='R' || Data=='G') break; //here
          Sensor();
          Angle_l(L1,L2);
          myservo.write(I_pos-Ang_l);
          endd=millis();
          }  
          while(endd-start2<3000){
          Sensor();
          Angle_l(L1,L2);
          myservo.write(I_pos-Ang_l);
          endd=millis();
          }  
          stopp(200);
          angle = 0.;
          set=false;}
        if (Data == 'R') {
          /////////////////////////////////////////////////////////////////////sdsdsds
          stopp(1000);
          forward(165,5);
          pData='R';
          Sensor();
          start2=millis();
          endd=millis();
          while(R1>30|| R2>30 && (endd-start2)<200){
              Gyro();
              Sensor();
              Angle_l(L1,L2);
              myservo.write(I_pos+Ang_l);
              endd=millis();
            
          forward(165, 10);
          u = 'R';
        }
        }
        else if (Data == 'G')
        {   Sensor();
          Angle_l(L1,L2);
          endd=millis();
          start2=millis();
          stopp(200);
          back(165,5);
        while(endd-start2<900){
          Sensor();
          Angle_l(L1,L2);
          myservo.write(I_pos-Ang_l);
          endd=millis();
          }  
          stopp(200);
        forward(165,5);
          Green();
          u = 'G';
        }
        start = millis();
        Sensor();
        Angle_l(L1, L2);
      }

      forward(165, 5);

    }
    start1 = millis();

  }
else {
    while (1) {
      ser();
      endd = millis();
      Sensor();
      Angle_r(R1, R2);
      Angle_l(L1, L2);
      while (L1 > 100 && R1 > 100) {  //here
        Sensor();
        ser();
        if (Data == 'R' || u == 'R') {//here
          Red();
          break;
        }
        else if (Data == 'G' ||  u == 'G') {//here
          Green();
          break;
        }
        else whitelclock();
      }
      {
        start = millis();
        endd = millis();
        while ((endd - start) < 4000) {
          whitel();
          ser();
          if(Data=='R'){
          Red();
          break;}
          else if(Data=='G'){
          Green();
          break;}
          endd = millis();
          Sensor();
          if (L2 < 100 && R2 < 100 && (endd - start) > 2000 && L1 < 100 && R1 < 100)
            break;
        }
        myservo.write(I_pos_1);
        stopp(100);
        back(255, 100);
        stopp(10000);
        dly(10000000);
      }
    }
  }
}

void whiterclock() {
  //stopp(2000);
  forward(165, 5);
  Sensor();
  Angle_r(R1, R2);
  Gyro();

  if (it < 13) {
    if (L1 >= dis && R1 >= dis) {
    
      myservo.write(I_pos_1 + Ang_r);
      if (abs(Ang_r) < 3) angle = 0.;
    }

    else if (R1 < dis) {

      Distr(R1);
    } else if (L1 < dis) {
      Distl(L1);
    }
    endd = millis();
    if (R1 > 100 && (endd - start) > 3000||set) {
      it++;
      Gyro();
      pData = 'W';
      stopp(50);
      Serial.print('n');
      myservo.write(I_pos - 15);
      back(165, 600);
      stopp(50);
      Serial.print('e');
      start2 = millis();
      endd = millis();
      while (endd - start2 < 500) {
        ser();
        if (Data == 'G' || Data == 'R') break; //here
        endd = millis();
      }

      forward(165, 5);
      Sensor();
      Angle_l(L1, L2);
      /*while(Ang_l<3){
        myservo.write(I_pos+10);
        Sensor();
        Angle_l(L1,L2);
        }*/
      /*myservo.write(I_pos);
        Serial.print('e');
        stopp(300);
        Sensor();
        ser();
        //stopp(300);
        forward(165, 5);*/

      if (Data == 'R') {
        myservo.write(I_pos - 15);
        dly(400);
        Sensor();
        while (R1 < 100) {
          Sensor();
          Angle_r(R1, R2);
          myservo.write(I_pos + Ang_r);
        }

        if ((endd - start) > 1500) {
        

          forward(165, 5);
          myservo.write(I_pos_1 + ang_1 + 7);

          dly(700);
        
          Sensor();
          Angle_l(L1, L2);
          Gyro();
          while (angle > 7) {

            myservo.write(I_pos_1 + ang_2 + 7);

            Gyro();
          }
          stopp(500);
          forward(165,5);
          Sensor();
          Angle_l(L1, L2);
          /*while (abs(Ang_l) > 5) {
            Sensor();
            Angle_l(L1, L2);
            myservo.write(I_pos_1 + ang_2);
          
          }*/

          forward(165, 5);
          start = millis();

          Sensor();
          Angle_l(L1, L2);
        }
        pData = 'R';
        loop();
      } else if (Data == 'G')

      {
        myservo.write(I_pos - 15);
        dly(500);
        //stopp(1000);
        //stopp(10000);
        Sensor();
        Angle_l(L1, L2);
        forward(165, 10);

        //myservo.write(I_pos-17);
        //dly(100);
        Sensor();
        Angle_l(L1, L2);

        while (F > 35) {

          myservo.write(I_pos + Ang_l);
          Sensor();
          Angle_l(L1, L2);
        }

        if ((endd - start) > 1500) {
          //it++;
        
          forward(165, 5);
          myservo.write(I_pos_1 + ang_1);
          dly(700);
          forward(165, 5);
          Gyro();
          Sensor();
          P_Ang_l = 0.;
          Angle_l(L1, L2);
          forward(165, 5);
          while (angle > -10) {
            myservo.write(I_pos_1 + ang_2);

            Gyro();
          }
          start = millis();
          whitel();
          forward(165, 5);
          Sensor();
          Angle_l(L1, L2);
        }
        pData = 'G';
      }
else if (Data == 'W') {
        if ((endd - start) > 1500) {
          Serial.print('n');
          myservo.write(I_pos - 10);
          dly(500);
          while (F > 65) {
            whitel();
          }
        stopp(50);

          forward(165, 5);
          
          myservo.write(I_pos_1 + ang_1);
          dly(250);
          Gyro();
          Sensor();
          Angle_l(L1, L2);
          
          while (angle > 15) {
          
            Gyro();
            myservo.write(I_pos_1 + ang_1);
          }
          stopp(50);
          forward(165,5);
  Sensor();
  Angle_l(L1, L2);
          while (abs(Ang_l) > 3) {
            Sensor();
            Angle_l(L1, L2);
            myservo.write(I_pos_1 + ang_2);
          }
          stopp(50);
          back(165, 5);
          start2 = millis();
          endd = millis();
          Sensor();
          Angle_l(L1, L2);
          stopp(10000);
          while (endd - start2 < 700) {
            back(165, 5);
            myservo.write(I_pos - Ang_l);
            Sensor();
            Angle_l(L1, L2);

            endd = millis();
          }

          stopp(50);
          Serial.print('e');
          start2 = millis();
          endd = millis();
          
          while (endd - start2 < 500) {
            ser();
            if (Data == 'G' || Data == 'R') //here
              break;
            endd = millis();
          }

          forward(165, 5);

          start = millis();
          Sensor();
          Angle_l(L1, L2);
          if (Data == 'R')
            Red();
          else if (Data == 'G')
            Green();
          else whiterclock();
        }
      }
    }

    start1 = millis();

  } else {
    while (1) {
      endd = millis();
      Sensor();
      Angle_r(R1, R2);
      Angle_l(L1, L2);

      if (((endd - start1) < 1200) && ((endd - start) > 1000 && (endd - start) < 1400 && F < 180)) {//here
        if (R1 < dis) {

          Distr(R1);


        } else if (L1 < dis) {

          Distl(L1);

        } else if (L1 >= dis && R1 >= dis) {  //
          myservo.write(I_pos_1 + Ang_l);

          //
        }
      } else {
        myservo.write(I_pos_1);
        stopp(100);
        back(255, 100);
        stopp(10000);
        dly(100000);
      }
    }
  }
}

void whiternclock() {
  forward(165, 5);
  Sensor();
  Angle_r(R1, R2);
  Gyro();

  if (it < 13) {

    if (L1 >= dis && R1 >= dis) {
      //
      myservo.write(I_pos_1 + Ang_r);

      //
      if (abs(Ang_r) < 3) angle = 0.;

    } else if (R1 < dis) {

      Distr(R1);
    } else if (L1 < dis) {
      Distl(L1);
    }
    if (L1 > 100) {
     
      Serial.print('e');
      //      stopp()
      //      ser();
      //      if(Data=='R')
      //      Red();
      //      else if(Data=='G')
      //      Green();
      pData = 'W';
      xg = 'r';
      xr='m';

      endd = millis();
      if ((endd - start) > 2000) {
         it++;
        forward(165, 5);
        Sensor();

        while (F > 65) {

          Sensor();
          whitel();
        }
        
        Serial.print('n');

        forward(165, 5);
        Gyro();
        Sensor();
        Angle_r(R1, R2);

        while (angle < -15) {

          myservo.write(I_pos_1 - ang_1);
          // ser();
          // if(Data=='R'){
          //   Red();
          //   break;}              ///////////////////////////////////////////////////////////////////////
          // else if(Data=='G'){
          //   Green();
          //   break;

          // }

          Gyro();
        }
        //angle = 0.;
        myservo.write(I_pos);
        Serial.print('e');
        stopp(200);
        forward(165, 5);
        myservo.write(I_pos);
        ser();
        if (Data == 'G') {
          /////////////////////////////////////////////////////////////////////sdsdsds
          forward(165, 10);
          u = 'G';
Green();
        }
        else if (Data == 'R')
        { stopp(200);
          back(165, 500);
          stopp(200);
          Red();
          u = 'R';
        }
        start = millis();
        Sensor();
        Angle_r(R1, R2);
      }

      forward(165, 5);

      loop();
      
    }
    start1 = millis();
  }

  else {
    while (1) {
      ser();
      endd = millis();
      Sensor();
      Angle_r(R1, R2);
      Angle_l(L1, L2);
      while (L1 > 100 && R1 > 100) {//here
        Sensor();
        ser();
        if (Data == 'R' && u == 'R') {//here
          Red();
          break;
        }
        else if (Data == 'G' && u == 'G') {//here
          Green();
          break;
        }
        else whitelclock();
      }
      {
        start = millis();
        endd = millis();
        while ((endd - start) < 4000) {
          whitel();
          ser();
          if(Data=='R'){
          Red();
          break;}
          else if(Data=='G'){
          Green();
          break;}
          endd = millis();
          Sensor();
          if (L2 < 100 && R2 < 100 && (endd - start) > 2000 && L1 < 100 && R1 < 100)
            break;
        }
        myservo.write(I_pos_1);
        stopp(100);
        back(255, 100);
        stopp(10000);
        dly(10000000);
      }
    }
  }
}

void whitelnclock() {
  //stopp(2000);
  forward(165, 5);
  Sensor();
  Angle_l(L1, L2);
  Gyro();

  if (it < 13) {
    if (L1 >= dis && R1 >= dis) {
    

      myservo.write(I_pos_1 + Ang_l);
      if (abs(Ang_l) < 3) angle = 0.;
    }

    else if (R1 < dis) {

      Distr(R1);
    } else if (L1 < dis) {
      Distl(L1);
    }
    endd = millis();
    if (L1 > 100 && (endd - start) > 1500) {
      pData = 'W';
      stopp(50);
      Serial.print('n');
      myservo.write(I_pos + 15);
      back(165, 600);
      stopp(50);
      Serial.print('e');
      start2 = millis();
      endd = millis();
      while (endd - start2 < 500) {
        ser();
        if (Data == 'G' && Data == 'R') break;
        endd = millis();
      }

      forward(165, 5);
      Sensor();
      Angle_r(R1, R2);
      /*while(Ang_l<3){
        myservo.write(I_pos+10);
        Sensor();
        Angle_l(L1,L2);
        }*/
      /*myservo.write(I_pos);
        Serial.print('e');
        stopp(300);
        Sensor();
        ser();
        //stopp(300);
        forward(165, 5);*/

      if (Data == 'G') {
        myservo.write(I_pos + 15);
        dly(400);
        Sensor();
        while (L1 < 100) {
          Sensor();
          Angle_l(L1, L2);
          myservo.write(I_pos + Ang_l);
        }

        if ((endd - start) > 1500) {
          it++;   

          forward(165, 5);
          myservo.write(I_pos_1 - ang_1 - 7);

          dly(700);
          Gyro();
          Sensor();
          Angle_r(R1, R2);

          while (angle < -10) {

            myservo.write(I_pos_1 - ang_2 - 7);

            Gyro();
          }
          while (abs(Ang_r) > 5) {
            Sensor();
            Angle_r(R1, R2);
            myservo.write(I_pos_1 - ang_2); 

          }
       
          //angle -=60.;

          forward(165, 5);
          start = millis();

          Sensor();
          Angle_r(R1, R2);
        }
        pData = 'G';
        loop();
      } else if (Data == 'R')

      {
        
        myservo.write(I_pos + 15);

        dly(500);
        //stopp(1000);
        //stopp(10000);
        Sensor();
        Angle_r(R1, R2);
        forward(165, 10);
        //myservo.write(I_pos-17);
        //dly(100);
        Sensor();
        Angle_r(R1, R2);

        while (F > 35) {
          //Serial.println(Ang_r);
        
          myservo.write(I_pos + Ang_r);
          Sensor();
          Angle_r(R1, R2);
        }

        if ((endd - start) > 1500) {
          it++;
         
          forward(165, 5);
          myservo.write(I_pos_1 - ang_1);
          dly(700);
          forward(165, 5);
          Gyro();
          Sensor();
          P_Ang_l = 0.;
          Angle_r(R1, R2);
          forward(165, 5);
          while (angle < 10) {
            myservo.write(I_pos_1 - ang_2);

            Gyro();
          }
          start = millis();
          whiter();
          forward(165, 5);
          Sensor();
          Angle_r(R1, R2);
        }
        pData = 'R';
      }

      else if (Data == 'W') {
        if ((endd - start) > 1500) {
          Serial.print('n');
          myservo.write(I_pos + 10);

          dly(500);
          while (F > 65) {
            whiter();
          }
          angle = -90;
          it++;

          forward(165, 5);
          myservo.write(I_pos_1 - ang_1);

          dly(250);
          Gyro();
          Sensor();
          Angle_r(R1, R2);

          while (angle < -15) {
            Gyro();
            myservo.write(I_pos_1 - ang_1);
          }

          while (abs(Ang_r) > 3) {
            Sensor();
            Angle_l(R1, R2);
            myservo.write(I_pos_1 - ang_2);
          }
          angle = 0.;
          stopp(50);
          back(165, 5);
          start2 = millis();
          endd = millis();
          Sensor();
          Angle_l(L1, L2);
          while (endd - start2 < 700) {
            back(165, 5);
            myservo.write(I_pos - Ang_l);
            Sensor();
            Angle_r(R1, R2);

            endd = millis();
          }

          stopp(50);
          Serial.print('e');
          start2 = millis();
          endd = millis();
          while (endd - start2 < 500) {
            ser();
            if (Data == 'G' && Data == 'R')//here
              break;
            endd = millis();
          }

          forward(165, 5);

          start = millis();
          Sensor();
          Angle_l(L1, L2);
          if (Data == 'R')
            Red();
          else if (Data == 'G')
            Green();
          else whiternclock();
        }
      }
    }

    start1 = millis();

  } else {
    while (1) {
      endd = millis();
      Sensor();
      Angle_r(R1, R2);
      Angle_l(L1, L2);

      if (((endd - start1) < 1200) && ((endd - start) > 1000 && (endd - start) < 1400 && F < 180)) {//here
        if (R1 < dis) {

          Distr(R1);


        } else if (L1 < dis) {

          Distl(L1);

        } else if (L1 >= dis && R1 >= dis) {  //
          myservo.write(I_pos_1 + Ang_l);

        }
      } else {
        myservo.write(I_pos_1);
        stopp(100);
        back(255, 100);
        stopp(10000);
        dly(100000);
      }
    }
  }
}

void Red() {

  if (pData != 'R') {
    //stopp(9000);
    forward(165, 5);
    Sensor();
    Gyro();
    if(pData=='G') pass(103 - L1);
    else if (xr == 'l') {
      //stopp(2000);
      //forward(165,5);
      pass(30);
      psa = 20.;
      forward(165, 5);
    } 
    else if(xr=='m') pass(R1+10);
    else pass(90 - L1);
    ser();
    while (angle > -psa) {
      Gyro();
      //Serial.println(angle);

      ser();
      myservo.write(I_pos + 15);
      Frservo(90 - psa);
    }
    Sensor();
    while (F > 25) {
      Sensor();

      ser();
      if (xr == 'l')  myservo.write(I_pos - 1);
      else myservo.write(I_pos);
    }
    Frservo(0);

    Gyro();
    while (abs(angle) > 10) {

      Gyro();

      ser();
      myservo.write(I_pos - 35);
    }
    Sensor();
    Angle_r(R1, R2);
    //    while (Ang_r < 3) {
    //      Sensor();
    //      Angle_r(R1, R2);
    //
    //      ser();
    //      myservo.write(I_pos - 10);
    //      Frservo(0);
    //    }}
  }

  Frservo(0);
  //Data = Serial.read();
  Data = 'W';
  pData = 'R';
  if (xr == 's') {
    Sensor();
    while (R1 < 100 && L1 < 100) {
      whiter();
      Sensor();
    }
    if (R1 > 100 && L1 < 100) {
      Clock = true;
      xr = 'a';
      whiterclock();
    } else if (R1 < 100 && L1 > 100) {
      Clock = false;
      xr = 'a';
      whiternclock();
    }
  }
  else {
    xr = 'a';
    if(Clock)
    whiterclock();
    else whiternclock();
  }
}
void Green() {
  //stopp(1000);
  forward(165,5);
  if (pData != 'G') {

    Sensor();
    Gyro();
    if (xg == 'r') {pass(80 - R1);
    psa=20.;
    }

    else if (xg == 'm') pass(L1 + 24);
    else{
     
      pass(L1+10);
    }
    //Serial.println(psa);
    forward(165, 5);


    ser();
Gyro();
    //Serial.print('n');
    while (abs(angle) < psa) {
      Gyro();

      ser();
      myservo.write(I_pos - 15);
      Frservo(-90 + psa);
    }
    Sensor();
    while (F > 25) {
      Sensor();

      ser();
      if(xg=='r') myservo.write(I_pos+1);
      else
      myservo.write(I_pos);
      // Serial.println(F);
    }
    Gyro();
    //stopp(1000);
    forward(165,5);
    //Serial.print('e');
    while (angle > 7) {
      Gyro();
      ser();
      myservo.write(I_pos + 30);
      /*if (Data == 'R') {
        Red();
        break;
      }*/
    }
    forward(165,5);
    //stopp(3000);
    Sensor();
    Angle_l(L1, L2);
    /*while (Ang_l > 3) {
      Sensor();
      Angle_l(L1, L2);

      ser();
      myservo.write(I_pos + 20);
      Frservo(0);
      if (Data == 'R') {
        Red();
        break;
      }
    }*/
  }
  Frservo(0);
  Data = 'W';
  pData = 'G';

  forward(165,5);
  if (xg == 's') {
    Sensor();
    while (R1 < 100 && L1 < 100) {
      whitel();
      Sensor();
    }
    if (R1 > 100 && L1 < 100) {
      Clock = true;
      xg='a';
      whitelclock();
    } else if (R1 < 100 && L1 > 100) {
      Clock = false;
      xg='a';
      whitelnclock();
    }
  } else{
    xg='a';
    start2=millis();
    endd=millis();
    while(endd-start2<50){
      whitel();
      endd=millis();
    }
    forward(165,5);
    if(Clock) whitelclock();
    else loop();
  }
}

void Frservo(float a) {
  Gyro();
  if (a > 40) a = 40;
  else if (a < -40) a = -40;
  fservo.write(I_posf + a);
}
void dly(int a){
  dlys=millis();
  dlye=millis();
  while(dlye-dlys<=a){
    Gyro();
    dlye=millis();
  }
}

void setup() {

  Wire.begin();
  gyro.initialize();
  pinMode(Mr, OUTPUT);
  pinMode(Ml, OUTPUT);
  pinMode(M_1, OUTPUT);
  pinMode(M_2, OUTPUT);
  pinMode(Fe, INPUT);
  pinMode(2, INPUT);
  Serial.begin(9600);
  myservo.attach(13);
  dly(10);
  myservo.write(I_pos);
  fservo.attach(3);
  fservo.write(I_posf);
  dly(10);
  Serial.print('a');
  angle=0.;
  while (digitalRead(2) == 0) {
    dly(10);
  } 
  Serial.print('b');
  while(1){
   Data=Serial.read();
  if(Data=='R'||Data=='W'||Data=='G') break;
  }

  Sensor();
  Gyro();
  forward(165, 5);
  xr = 's';
  xg = 's';
  if (Data == 'G') {
    Green();
  } else if (Data == 'R') {
    Red();
  } else {
    Sensor();
    
    while (R1 < 100 && L1 < 100) {
      whiter();
      /*ser();
      if (Data == 'R') {
        Red();
      } else if (Data == 'G') {
        Green();
      }*/
      Sensor();
    }
    forward(165,5);
    if (R1 > 100 && L1 < 100) {
      Clock = true;
      stopp(1000);
      set=true;
      forward(165,5);
      whitelclock();
    } else if (R1 < 100 && L1 > 100) {
      Clock = false;
      whiternclock();
    }
  }
  /*
     while (R1 < 100 && L1 < 100) {
    Gyro();
    Sensor();
    Angle_l(L1, L2);
    myservo.write(I_pos + Ang_l);

    forward(165, 5);
    if (R1 < dis) {

      Distr(R1);
    } else if (L1 < dis) {
      Distl(L1);
    }
    //    myservo.write(I_pos + Ang_l);
    }

    if (L1 > 100 && R1 < 100) {
    Clock = false;
    }
    digitalWrite(M_1, LOW);
    digitalWrite(M_2, HIGH);
    analogWrite(M, 255);
    }

  */

  //Clock = true;
  loop();
}

void loop() {
  // put your main code , to run repeatedly:
  while (Serial.available() > 0) {
    //myservo.write(100);
    forward(165, 5);
    //Serial.write('F');
    while (Clock) {

      ser();

      if (Data == 'R') {
        //Serial.write('T');
        Red();
      }

      else if (Data == 'G') {
        Green();
      }
      else {
if (pData == 'R') {
          //stopp(2000);
          whiterclock();
        }
        else
          whitelclock();

      }
    }
    while (!Clock) {
      ser();


      if (Data == 'R') {
        //Serial.write('T');
        Red();
      }

      else if (Data == 'G') {
        Green();
      }
      else {

        if (pData == 'R') {
          //stopp(2000);
          whiternclock();
        }
        else
          whitelnclock();

      }
    }
  }
}
//End Of Second Day Code//