#include <Arduino.h>
#include <Encoder.h>

Encoder LEnc(18, 19); 
Encoder REnc(2, 3);

#define LM 8
#define RM 9

#define leftF 47
#define leftB 49

#define rightF 51
#define rightB 53

#define start_but 51 //start button

int max_A8 = 34;//L5
int max_A0 = 1008;//L4
int max_A1= 1008;//L3
int max_A2= 1008;//L2
int max_A3= 1008;//L1
int max_A4= 1008;//R1
int max_A5= 1008;//R2
int max_A6= 1008;//R3
int max_A7= 1008;//R4
int max_A9= 34;//R5

//B_DL A10
//B_DR A11
int min_A8 = 17;
int min_A0 = 847;
int min_A1 = 890;
int min_A2 = 880;
int min_A3 = 887;
int min_A4 = 886;
int min_A5 = 862;
int min_A6 = 864;
int min_A7 = 827;
int min_A9 = 17;

int a = 0; //black line>>+1,white line>>0

int DL5;
int DL4;
int DL3;
int DL2;
int DL1;
int DR1;
int DR2;
int DR3;
int DR4;
int DR5;

int cross_count;
int count_at_c;
int lf_err;
int lf_prverr;
int lf_bwd_err;
int lf_bwd_prverr;

int stt_time;
int elp_time;

long LnewPosition;
long RnewPosition;

long ev1;
long ev2;
long enc_err;

int flag = 0;


//Game Parameters
int gems=0;
char wall_color = 'N';
float kp = 0.1;

//---------Color Sensor1 U---------
#define S01 44
#define S11 36
#define S21 42
#define S31 46

#define OUT1 38
#define CS1_LED 40
int delaytime = 10;

int arr1[] = {70, 120, 90, 150, 30, 100};

int red_min1 = arr1[0];
int red_max1 = arr1[1];
int green_min1 = arr1[2];
int green_max1 = arr1[3];
int blue_min1 = arr1[4];
int blue_max1 = arr1[5];



float red1 = 0; float redColor1 = 0; float rednew1;
float green1 = 0; float greenColor1 = 0; float greennew1;
float blue1 = 0; float blueColor1 = 0; float bluenew1;

float total1;
String color1;
String color;
int countR, countG, countB;

////---------Color Sensor2 Front---------
#define S02 32
#define S12 26
#define S22 22
#define S32 24
#define CS2_LED 30

#define OUT2 28

int arr2[] = {70, 120, 90, 150, 30, 100};

int red_min2 = arr2[0];
int red_max2 = arr2[1];
int green_min2 = arr2[2];
int green_max2 = arr2[3];
int blue_min2 = arr2[4];
int blue_max2 = arr2[5];



float red2 = 0; float redColor2 = 0; float rednew2;
float green2 = 0; float greenColor2 = 0; float greennew2;
float blue2 = 0; float blueColor2 = 0; float bluenew2;

float total2;
String color2;

//----------colour sensor end---------------


//Declare Functions
void line_flw();
void invt_line_flw();
void line_flw_bwd();
void line_flw_dur(int time_);
void mtr_cmd(int lm, int rm);
void brk();
void cal();
void trn_180();
void trn_lft();
void trn_rgt();
void trn_180_at_c();
void fwd_enc(int duration);
void bwd_enc(int duration);
void fwd_enc_spd(int duration,int spd);
void dgtl();
char colorDetect1();
char colorDetect2();
void buzz();
void line_flw_fwd();





void setup() {

  Serial.begin(9600);
  Serial3.begin(9600);
  // put your setup code here, to run once:
  // for (int pin = A0; pin <= A15; pin++) {
  //   pinMode(pin, INPUT);
  // }
 
  
  

  pinMode(17,OUTPUT);//Buzzer

  pinMode(LM, OUTPUT);
  pinMode(RM, OUTPUT);
  pinMode(leftF, OUTPUT);
  pinMode(leftB, OUTPUT);
  pinMode(rightF, OUTPUT);
  pinMode(rightB, OUTPUT);

  //------------colour sensor----------
  pinMode(S01, OUTPUT);
  pinMode(S11, OUTPUT);
  pinMode(S21, OUTPUT);
  pinMode(S31, OUTPUT);
  pinMode(OUT1, INPUT);

  pinMode(CS1_LED, OUTPUT);
  pinMode(CS2_LED, OUTPUT);

  digitalWrite(S01, HIGH);
  digitalWrite(S11, HIGH);
  digitalWrite(CS1_LED, HIGH);
  digitalWrite(CS2_LED, HIGH);

  pinMode(S02, OUTPUT);
  pinMode(S12, OUTPUT);
  pinMode(S22, OUTPUT);
  pinMode(S32, OUTPUT);
  pinMode(OUT2, INPUT);

  digitalWrite(S02, HIGH);
  digitalWrite(S12, HIGH);

  //-----------colour sensor end------------
   buzz();
   cal();
   buzz();

  // while (digitalRead(start_but) != HIGH) {
  //   delay(1);
  // }
  fwd_enc(400);

}

void loop() {
  

  //   if (flag == 0) {
  //   dgtl();
   

  //   while (true) {
  //     if (DR4 == a * 1 && DL4 == a * 1) {
  //       break;
  //     }
  //     if(DL5 == a*1){
  //      fwd_enc(100);
  //     }

  //     line_flw();
  //     dgtl();
  //   }
  //   if (DR4 == a * 1 && DL4 == a * 1) {
  //     brk();
  //     delay(500);
  //     fwd_enc(100);
  //     brk();
  //     delay(500);
  //     //color detection
  //     wall_color = colorDetect1();

  //     bwd_enc(650);
  //     trn_180();
  //     delay(500);
  //     dgtl();

  //     while (true) {
  //       if (DR5 == a * 1 ) {
  //         break;
  //       }
  //       line_flw();
  //       dgtl();
  //     }

  //     if (DR5 == a * 1) {
  //       brk();
  //       delay(500);
  //       fwd_enc(100);
  //       trn_rgt();
  //       dgtl();

  //       while (true) {
  //         if (DR5 == a * 1 ) {
  //           break;
  //         }
  //         line_flw();
  //         dgtl();
  //       }
  //       brk();
  //       delay(1000);
  //       fwd_enc(175);
  //       trn_rgt();

  //       while (!(DR5 == a * 1 && DL5 == a * 1)) {
  //         line_flw();
  //         dgtl();
  //       }
  //       brk();
  //       delay(500);
  //       fwd_enc(100);
  //       trn_lft();
  //       dgtl();
        
  //       flag = 1;
  //       brk();
        
  //     }
  //   }
  // }
  

  //line_flw_fwd();
  line_flw();

}


//////////////////////////////////////////////////////////////////////LINE FOLLOWER///////////////////////////////////////////////////////////////////////
void line_flw() {//White line


  float kp = 2;
  float kd = 8;
  float ki = 1;

  
  int R1 = map(analogRead(A4), min_A4,max_A4/*45 , 770*/ , 50, 0);
  int L1 = map(analogRead(A3), min_A3,max_A3/*40 , 660*/ , 50, 0);
  int R2 = map(analogRead(A5), min_A5,max_A5/*45 , 770*/ , 50, 0);
  int L2 = map(analogRead(A2), min_A2,max_A2/*40 , 660*/ , 50, 0);
  int L3 = map(analogRead(A1), min_A1,max_A1/*45 , 700*/ , 50, 0);
  int R3 = map(analogRead(A6), min_A6,max_A6/*40 , 700*/ , 50, 0);
  int L4 = map(analogRead(A0), min_A0,max_A0/*45 , 700*/ , 50, 0);
  int R4 = map(analogRead(A7), min_A7,max_A7/*40 , 700*/ , 50, 0);
  

  lf_err=0.125*(L1-R1)+0.25*(L2-R2)+0.5*(L3-R3)+1*(L4-R4);

  int lf_dif = lf_err * kp + (lf_err - lf_prverr) * kd + (lf_err + lf_prverr)* ki;
  Serial.print("ERR ");
  Serial.print(lf_dif);
  Serial.print(" ");
  Serial.print("Kp ");
  Serial.println(kp);

  Serial3.print("ERR ");
  Serial3.print(lf_dif);
  Serial3.print(" ");
  Serial3.print("Kp ");
  Serial3.println(kp);
  
  mtr_cmd((90 - lf_dif/3),(90 + lf_dif/3));

  lf_prverr = lf_err;

  
 
}

void buzz(){

 digitalWrite(17, HIGH);
 delay(100);
 digitalWrite(17, LOW);

}


void invt_line_flw() {//Black Line

  float kp = 0.2;
  float kd = 0;
  float ki = 0.0002;
 
  int R2 = map(analogRead(A5), min_A5,max_A5/*45 , 770*/ , 0, 50);
  int L2 = map(analogRead(A2), min_A2,max_A2/*40 , 660*/ , 0, 50);
  int L4 = map(analogRead(A0), min_A0,max_A0/*45 , 700*/ , 0, 50);
  int R4 = map(analogRead(A7), min_A4,max_A4/*40 , 700*/ , 0, 50);

  lf_err=(L2-R2)+0.5*(L4-R4);

  int lf_dif = lf_err * kp + (lf_err - lf_prverr) * kd + (lf_err + lf_prverr)* ki;
  Serial.println(lf_dif);
  Serial3.println(lf_dif);
  
   mtr_cmd(90 - lf_dif, 90 + lf_dif);

  lf_prverr = lf_err;

 

 

}



void line_flw_bwd() {
  float kp = 0.1;
  float kd = 0.0001;

  int R4 = map(analogRead(A7), min_A7 , max_A7 , 0, -400);
  int R3 = map(analogRead(A6), min_A6 , max_A6 , 0, -300);
  int R2 = map(analogRead(A5), min_A5 , max_A5 , 0, -200);
  int R1 = map(analogRead(A4), min_A4 , max_A4 , 0, -100);
  int L1 = map(analogRead(A3), min_A3 , max_A3 , 0, 100);
  int L2 = map(analogRead(A2), min_A2 , max_A2 , 0, 200);
  int L3 = map(analogRead(A1), min_A1 , max_A1 , 0, 300);
  int L4 = map(analogRead(A0), min_A0 , max_A0 , 0, 400);

  lf_bwd_err = L4 + L3 + L2 + L1 + R1 + R2 + R3 + R4;

  int lf_dif = lf_bwd_err * kp + (lf_bwd_err - lf_bwd_prverr) * kd;

  mtr_cmd(-1*(100 - lf_dif), -1*(100 + lf_dif));

  lf_bwd_prverr = lf_bwd_err;

}

void line_flw_fwd() {
  float kp = 0.01;
  float kd = 0.0001;

  int R4 = map(analogRead(A7), min_A7 , max_A7 , 0, -400);
  int R3 = map(analogRead(A6), min_A6 , max_A6 , 0, -300);
  int R2 = map(analogRead(A5), min_A5 , max_A5 , 0, -200);
  int R1 = map(analogRead(A4), min_A4 , max_A4 , 0, -100);
  int L1 = map(analogRead(A3), min_A3 , max_A3 , 0, 100);
  int L2 = map(analogRead(A2), min_A2 , max_A2 , 0, 200);
  int L3 = map(analogRead(A1), min_A1 , max_A1 , 0, 300);
  int L4 = map(analogRead(A0), min_A0 , max_A0 , 0, 400);

  lf_bwd_err = L4 + L3 + L2 + L1 + R1 + R2 + R3 + R4;

  int lf_dif = lf_bwd_err * kp + (lf_bwd_err - lf_bwd_prverr) * kd;


  mtr_cmd((80 - lf_dif), (80 + lf_dif));

  lf_bwd_prverr = lf_bwd_err;

}


void line_flw_dur(int time_){
  stt_time=millis();
  elp_time=0;
  while(time_>elp_time){
    line_flw();
    elp_time=millis()-stt_time;
  }
}
//////////////////////////////////////////////////////////////////////LINE FOLLOWER///////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////MORTOR///////////////////////////////////////////////////////////////////////////////
void mtr_cmd(int lm, int rm) {
  /////////lm///////////////////////
  if (0 < lm) {
    if (lm >= 255) {
      lm = 255;
    }
    else {
      lm = lm;
    }

  }


  else {
    if (lm <= -255) {
      lm = -255;
    }
    else {
      lm = lm;
    }
  }

  ///////////rm/////////////////////
  if (0 < rm) {
    if (rm >= 255) {
      rm = 255;
    }
    else {
      rm = rm;
    }
  }


  else {
    if (rm <= -255) {
      rm = -255;
    }
    else {
      rm = rm;
    }
  }

  ///////////////////Lmtr//////////////

  if (lm > 0) {

    digitalWrite(leftF, HIGH);
    digitalWrite(leftB, LOW);
    analogWrite(LM, lm);
  }

  else {
    digitalWrite(leftF, LOW);
    digitalWrite(leftB, HIGH);
    analogWrite(LM, (-1)*lm);
  }

  /////////////////////Rmtr////////////////
  if (rm > 0) {
    digitalWrite(rightF, HIGH);
    digitalWrite(rightB, LOW);
    analogWrite(RM, rm);
  }

  else {
    digitalWrite(rightF, LOW);
    digitalWrite(rightB, HIGH);
    analogWrite(RM, (-1)*rm);
  }
}

void brk(){
  digitalWrite(leftF,HIGH);
  digitalWrite(leftB,HIGH);
  digitalWrite(rightF,HIGH);
  digitalWrite(rightB,HIGH);
  
}
///////////////////////////////////////////////////////////////////////////////////////MORTOR////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////Calibrate///////////////////////////////////////////////////////////////////////

void cal() {
  min_A8 = 1024;
  min_A0 = 1024;
  min_A1 = 1024;
  min_A2 = 1024;
  min_A3 = 1024;
  min_A4 = 1024;
  min_A5 = 1024;
  min_A6 = 1024;
  min_A7 = 1024;
  min_A9 = 1024;

 
  for (int i = 0; i < 5000; i++) {
    //Serial.println("calibrating");
    int A8_Reading = analogRead(A8);
    int A7_Reading = analogRead(A7);
    int A6_Reading = analogRead(A6);
    int A5_Reading = analogRead(A5);
    int A4_Reading = analogRead(A4);
    int A3_Reading = analogRead(A3);
    int A2_Reading = analogRead(A2);
    int A1_Reading = analogRead(A1);
    int A0_Reading = analogRead(A0);
    int A9_Reading = analogRead(A9);
    
    if (A8_Reading > max_A8) {
      max_A8 = A8_Reading;
    }
    else if (A8_Reading < min_A8) {
      min_A8 = A8_Reading;
    }

    /////////////////////////////////////////////
    if (A0_Reading > max_A0) {
      max_A0 = A0_Reading;
    }
    else if (A0_Reading < min_A0) {
      min_A0 = A0_Reading;
    }

    /////////////////////////////////////////////
    if (A1_Reading > max_A1) {
      max_A1 = A1_Reading;
    }
    else if (A1_Reading < min_A1) {
      min_A1 = A1_Reading;
    }
    /////////////////////////////////////////////

    
    if (A2_Reading > max_A2) {
      max_A2 = A2_Reading;
    }
    else if (A2_Reading < min_A2) {
      min_A2 = A2_Reading;
    }


    /////////////////////////////////////////////

    if (A3_Reading > max_A3) {
      max_A3 = A3_Reading;
    }
    else if (A3_Reading < min_A3) {
      min_A3 = A3_Reading;
    }

    /////////////////////////////////////////////

    if (A4_Reading > max_A4) {
      max_A4 = A4_Reading;
    }
    else if (A4_Reading < min_A4) {
      min_A4 = A4_Reading;
    }

    ///////////////////////////////////////////


    if (A5_Reading > max_A5) {
      max_A5 = A5_Reading;
    }
    else if (A5_Reading < min_A5) {
      min_A5 = A5_Reading;
    }


    ///////////////////////////////////////////

    if (A6_Reading > max_A6) {
      max_A6 = A6_Reading;
    }
    else if (A6_Reading < min_A6) {
      min_A6 = A6_Reading;
    }

    ///////////////////////////////////////////

    if (A7_Reading > max_A7) {
      max_A7 = A7_Reading;
    }
    else if (A7_Reading < min_A7) {
      min_A7 = A7_Reading;
    }

    /////////////////////////////////////////////
    if (A9_Reading > max_A9) {
      max_A9 = A9_Reading;
    }
    else if (A9_Reading < min_A9) {
      min_A9 = A9_Reading;
    }


  }

 
  /*Serial.print(min_A7);
    Serial.print("    ");
    Serial.print(min_A6);
    Serial.print("    ");
    Serial.print(min_A5);
    Serial.print("    ");
    Serial.print(min_A4);
    Serial.print("    ");
    Serial.print(min_A3);
    Serial.print("    ");
    Serial.print(min_A2);
    Serial.print("    ");
    Serial.print(min_A1);
    Serial.print("    ");
    Serial.print(min_A0);
    Serial.println("    ");

    Serial.print(max_A7);
    Serial.print("    ");
    Serial.print(max_A6);
    Serial.print("    ");
    Serial.print(max_A5);
    Serial.print("    ");
    Serial.print(max_A4);
    Serial.print("    ");
    Serial.print(max_A3);
    Serial.print("    ");
    Serial.print(max_A2);
    Serial.print("    ");
    Serial.print(max_A1);
    Serial.print("    ");
    Serial.print(max_A0);
    Serial.print("    ");*/
}
///////////////////////////////////////////////////////////////////////////Calibrate///////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////Turn///////////////////////////////////////////////////////////////////////////
void trn_180(){
  int enc_val=600;
  mtr_cmd(0, 0);
  delay(1000);

  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;

  while (RnewPosition > -enc_val*2 && LnewPosition < enc_val*2) {

    mtr_cmd(150, -150);
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
  }


  mtr_cmd(0, 0);
  delay(1000);
}


void trn_lft() {
  int enc_val=600;
  

  mtr_cmd(0, 0);
  delay(1000);

  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;


  while (LnewPosition > -enc_val && RnewPosition < enc_val) {

    mtr_cmd(-150, 150);
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
  }

  mtr_cmd(0, 0);
  delay(1000);

   fwd_enc(175);


}


void trn_rgt() {

  int enc_val = 750;
  

  mtr_cmd(0, 0);
  delay(1000);

  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;

  while (RnewPosition > -enc_val && LnewPosition < enc_val) {

    mtr_cmd(120, -120);
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
  }

  
  mtr_cmd(0, 0);
  delay(1000);

  fwd_enc(175);  
  
}

void trn_180_at_c(){
  int enc_val=650;
  mtr_cmd(0, 0);
  delay(1000);

  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;

  while (RnewPosition > -enc_val*2 && LnewPosition < enc_val*2) {

    mtr_cmd(150, -150);
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
  }


  mtr_cmd(0, 0);
  delay(1000);
}

void trn_180_at_maze(){
  int enc_val=650;
  mtr_cmd(0, 0);
  delay(1000);

  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;

  while (LnewPosition > -enc_val*2 && RnewPosition < enc_val*2) {

    mtr_cmd(-150, +150);
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
  }


  mtr_cmd(0, 0);
  delay(1000);
}
///////////////////////////////////////////////////////////////////////////Turn///////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////Forward PID///////////////////////////////////////////////////////////////////////////
void fwd_enc(int duration){
  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;
  elp_time=0;
  stt_time=millis();

  while (elp_time<=duration) {
    enc_err=(-1)*LnewPosition+RnewPosition;

    mtr_cmd(150+enc_err,150-enc_err);
    
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
    elp_time=millis()-stt_time;
  }
}


void bwd_enc(int duration){
  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;
  elp_time=0;
  stt_time=millis();

  while (elp_time<=duration) {
    enc_err=(-1)*LnewPosition+RnewPosition;

    mtr_cmd(-1*(120-enc_err),-1*(120+enc_err));
    
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
    elp_time=millis()-stt_time;
  }
}


void fwd_enc_spd(int duration,int spd){
  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;
  elp_time=0;
  stt_time=millis();

  while (elp_time<=duration) {
    enc_err=(-1)*LnewPosition+RnewPosition;

    mtr_cmd(spd+enc_err,spd-enc_err);
    
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
    elp_time=millis()-stt_time;
  }
}
//////////////////////////////////////////////////////////////////////Forward PID///////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////IR Digitalize///////////////////////////////////////////////////////////////////////////
void dgtl() {

 
 int refA0 = (max_A0 + min_A0) / 2;
 int refA1 = (max_A1 + min_A1) / 2;
 int refA2 = (max_A2 + min_A2) / 2;
 int refA3 = (max_A3 + min_A3) / 2;
 int refA4 = (max_A4 + min_A4) / 2;
 int refA5 = (max_A5 + min_A5) / 2;
  int refA6 = (max_A6 + min_A6) / 2;
 int refA7 = (max_A7 + min_A7) / 2;
 int refA8 = (max_A8 + min_A8) / 2;
 int refA9 = (max_A9 + min_A9) / 2;

  if (analogRead(A8) > refA8) {
    DL5 = 1; //1>>BLACK
  }

  else {
    DL5 = 0;
  }

  if (analogRead(A0) > refA0) {
    DL4 = 1; //1>>BLACK
  }

  else {
    DL4 = 0;
  }

  if (analogRead(A1) > refA1) {
    DL3 = 1; //1>>BLACK
  }
  else {
    DL3 = 0;
  }

  if (analogRead(A2) > refA2) {
    DL2 = 1; //1>>BLACK
  }
  else {
    DL2 = 0;
  }

  if (analogRead(A3) > refA3) {
    DL1 = 1; //1>>BLACK
  }
  else {
    DL1 = 0;
  }

  if (analogRead(A4) > refA4) {
    DR1 = 1; //1>>BLACK
  }
  else {
    DR1 = 0;
  }

  if (analogRead(A5) > refA5) {
    DR2 = 1; //1>>BLACK
  }
  else {
    DR2 = 0;
  }

  if (analogRead(A6) > refA6) {
    DR3 = 1; //1>>BLACK
  }

  else {
    DR3 = 0;
  }

  if (analogRead(A7) > refA7) {
    DR4 = 1; //1>>BLACK
  }

  else {
    DR4 = 0;
  }

  if (analogRead(A9) > refA9) {
    DR5 = 1; //1>>BLACK
  }

  else {
    DR5 = 0;
  }
  /*Serial.print(DL5);
  Serial.print("    ");
  Serial.print(DL4);
  Serial.print("    ");
  Serial.print(DL3);
  Serial.print("    ");
  Serial.print(DL2);
  Serial.print("    ");
  Serial.print(DL1);
  Serial.print("    ");
  Serial.print(DR1);
  Serial.print("    ");
  Serial.print(DR2);
  Serial.print("    ");
  Serial.print(DR3);
  Serial.print("    ");
  Serial.print(DR4);
  Serial.print("    ");
  Serial.print(DR5);
  Serial.println("    ")*/;


}
/////////////////////////////////////////////////////////////////IR Digitalize///////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////Color Detection /////////////////////////////////////////////////////////////////////////

char colorDetect1() {

  digitalWrite(S21, LOW);
  digitalWrite(S31, LOW);
  red1 = pulseIn(OUT1, LOW);
  redColor1 = map(red1, red_min1, red_max1, 255, 0);

  digitalWrite(S21, HIGH);
  digitalWrite(S31, HIGH);
  green1 = pulseIn(OUT1, LOW);
  greenColor1 = map(green1, green_min1, green_max1, 255, 0);

  digitalWrite(S21, LOW);
  digitalWrite(S31, HIGH);
  blue1 = pulseIn(OUT1, LOW);
  blueColor1 = map(blue1, blue_min1, blue_max1, 255, 0);

  total1 = red1 + green1 + blue1;


  rednew1 = (red1 / total1) * 100;
  greennew1 = (green1 / total1) * 100;
  bluenew1 = (blue1 / total1) * 100;

  if (rednew1 < greennew1 && rednew1 < bluenew1) { //RED
    return 'R';
  }
  else if (rednew1 > greennew1 && greennew1 < bluenew1) { //GREEN
    return 'G';
  }
  else if (bluenew1 < greennew1 && rednew1 > bluenew1) { //BLUE
    return 'B';
  }else{
    return 'N';
  }
  //    Serial.print(R1);
  //    Serial.print(" ");
  //
  //    Serial.print(G);
  //    Serial.print(" ");
  //
  //
  //    Serial.print(B);
  //    Serial.print(" ");
  //
  //    Serial.println();
  //    delay(100);

}

char colorDetect2() {

  digitalWrite(S22, LOW);
  digitalWrite(S32, LOW);
  red2 = pulseIn(OUT2, LOW);
  redColor2 = map(red2, red_min2, red_max2, 255, 0);

  digitalWrite(S22, HIGH);
  digitalWrite(S32, HIGH);
  green2 = pulseIn(OUT2, LOW);
  greenColor2 = map(green2, green_min2, green_max2, 255, 0);

  digitalWrite(S22, LOW);
  digitalWrite(S32, HIGH);
  blue2 = pulseIn(OUT2, LOW);
  blueColor2 = map(blue2, blue_min2, blue_max2, 255, 0);

  total2 = red2 + green2 + blue2;


  rednew2 = (red2 / total2) * 100;
  greennew2 = (green2 / total2) * 100;
  bluenew2 = (blue2 / total2) * 100;

  if (rednew1 < greennew1 && rednew1 < bluenew1) { //RED
    return 'R';
  }
  else if (rednew1 > greennew1 && greennew1 < bluenew1) { //GREEN
    return 'G';
  }
  else if (bluenew1 < greennew1 && rednew1 > bluenew1) { //BLUE
    return 'B';

  }else{
    return 'N';
  }
  //    Serial.print(R1);
  //    Serial.print(" ");
  //
  //    Serial.print(G);
  //    Serial.print(" ");
  //
  //
  //    Serial.print(B);
  //    Serial.print(" ");
  //
  //    Serial.println();
  //    delay(100);
}
