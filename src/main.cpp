#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>
#include <NewPing.h>

Encoder LEnc(18, 19); 
Encoder REnc(2, 3);

#define LM 8
#define RM 9

#define leftF 47
#define leftB 49

#define rightF 51
#define rightB 53

#define start_but 51 //start button

//Arm 
#define armY_Pin 10
#define armX_Pin 11

Servo armY;
Servo armX;

//Front Ultra Sonic Sensor
#define trigPinFront 4
#define echoPinFront 5

//Object Detector
#define objDetectorReset 7
#define objType 6 //0>>Cylinder 1>> cuboid


// NewPing sonar(trigPinFront, echoPinFront, 100);


int max_A8 = 37;//L5
int max_A0 = 987;//L4
int max_A1= 988;//L3
int max_A2= 988;//L2
int max_A3= 988;//L1
int max_A4= 988;//R1
int max_A5= 991;//R2
int max_A6= 991;//R3
int max_A7= 992;//R4
int max_A9= 37;//R5

//B_DL A10
//B_DR A11
int min_A8 = 14;
int min_A0 = 140;
int min_A1 = 67;
int min_A2 = 57;
int min_A3 = 163;
int min_A4 = 165;
int min_A5 = 149;
int min_A6 = 240;
int min_A7 = 322;
int min_A9 = 14;

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

int flag = 9;

//Run Switch
#define run_sw 23

//Metral Detector
#define metalDetector A15


//Game Parameters
int gems=0;
int wall_color = 0;
float kp = 0.1;
int colorCircleTurn = 0; // 0>>left, 1>>right

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
void fwd_enc_spd(float target_distance,int spd);
void bwd_enc_spd(float target_distance,int spd);
void bwd_enc_givenSpd(int target_distance,int spd);  
void fwd_enc_givenSpd(int target_distance,int spd);

void dgtl();
int colorDetect1(); // red>>1 , green>>2 , blue>>3
int colorDetect2();
void buzz();
void line_flw_fwd();
void catch_obj();
void drop_obj();
void arm_up();
void arm_down();
void grab_obj();
void release_obj();
int isMetal();
int getDistance();  
void trn_Precide_180();
void trnToWhiteLine180();
void trnToWhiteLinergt();
void trnToWhiteLinelft();
void line_flw_circle();
int trnSlightlylft(int time);
int trnSlightlyrgt(int time);
int captureCube();
void bwd_line_flw();
void line_flw_given_spd(int speed);
void close_arm();


void setup() {

  Serial.begin(9600);
  Serial3.begin(9600);
  // put your setup code here, to run once:
  // for (int pin = A0; pin <= A15; pin++) {
  //   pinMode(pin, INPUT);
  // }
 
 //Metal detector
 pinMode(metalDetector, INPUT);
  
  //Servo (Arms)
  pinMode(armY_Pin, OUTPUT);
  pinMode(armX_Pin, OUTPUT);

  armY.attach(armY_Pin);
  armX.attach(armX_Pin);

  armX.write(180);
  armY.write(20);


  pinMode(17,OUTPUT);//Buzzer
  pinMode(run_sw, INPUT);//Run Switch
  pinMode(objDetectorReset, OUTPUT);//Object Detector Reset
  pinMode(objType, INPUT);//Object Type

  digitalWrite(objDetectorReset, LOW);



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

  pinMode(trigPinFront,OUTPUT);
  pinMode(echoPinFront,  INPUT);

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

  // while (digitalRead(run_sw) != HIGH) {
  //   delay(500);
  // }
  fwd_enc(400);
  //fwd_enc_spd(6,80);
  
  //ARM test
  // catch_obj();
  // delay(2000);
  // drop_obj();

  

}

void loop() {

  if(flag==0){
    dgtl();
    while(true){
      if(DL5==a && DL4==a && DL3==a && DL2==a){
          break; 
      }
      line_flw();
      dgtl();
    }
    
      brk();
      delay(500);
      fwd_enc(100);
      delay(200);
      dgtl();
      while(true){//Wall detection
          if(getDistance()<7){  
              break;
          }
          line_flw();
          dgtl();
          delay(10);
      }
      brk();
      delay(500);
      while(true){
          if(getDistance()<5){
              break;
          }
          fwd_enc(50); 
          delay(100);   
          buzz();
          delay(100);
          buzz();
          delay (100);
          buzz();
               
      }
      delay(500);
      while(true){
        wall_color = colorDetect2();//Wall color detection
        if(wall_color==3){
          buzz();
          break;
        }else if(wall_color==2){
          buzz();
          delay(100);
          buzz();
          break;
      }
      }
      
      delay(500);

      
      delay(500);
      bwd_enc(250);
      trnToWhiteLine180();
      //trnPrecise180();  //Trn 180 at wall detection area
      delay(500);
      fwd_enc(100);
      
      flag = 1;


  }else if(flag==1){//First turn
    dgtl();
    while(true){

        if(DR5==a && DR4==a && DR3==a && DR2==a ){
          break;

        } 
        
        line_flw();
        dgtl();
      }
      brk();
      delay(500);
      fwd_enc(350); // current position
      trnToWhiteLinergt(); // right turn
      dgtl();
      flag=2;
  }

  else if(flag==2){//second turn Rgt
    dgtl();
    while(true){

        if(DR5==a && DR4==a && DR3==a && DR2==a ){
          break;

        } 
        
        line_flw();
        dgtl();
      }
      brk();
      delay(500);
      fwd_enc(320);
      trnToWhiteLinergt(); // right turn
      dgtl();
      flag=3;

  }else if (flag == 3) {//T junction
    dgtl();
    buzz();
    delay(100);
    buzz();

    while(true){
      if(DL5==a && DL4==a && DL3==a && DL2==a){
          break; 
      }
      line_flw();
      dgtl();
    }
    
      brk();
      delay(500);
      fwd_enc(250);
      trnToWhiteLinelft();
      delay(200);
      line_flw_dur(1000);
      flag = 4;
  }else if(flag == 4){//Color circle

    dgtl();
    while(true){
      if(DL5==a && DL4==a && DL3==a && DL2==a && DR5==a && DR4==a && DR3==a && DR2==a){
          break; 
      }
      line_flw();
      dgtl();
    }
    
      brk();
      delay(500);
      fwd_enc_spd(12,100);

      //fwd_enc(500);
      trnToWhiteLinelft();

      delay(200);
      
      int color = 0;
      while(true){
        if(colorDetect1() != 0){
          color = colorDetect1();
          break;
        }
        
        bwd_enc(50);
        delay(50);

      }
      // Detect Floor Color

      // while(true){
      //   wall_color = colorDetect2();//Wall color detection
      //   if(wall_color==3){
      //     buzz();
      //     break;
      //   }else if(wall_color==2){
      //     buzz();
      //     delay(100);
      //     buzz();
      //     break;
      // }
      // }
      if(color != wall_color){
        colorCircleTurn = 1;
        trnToWhiteLine180();
      }

      delay(200);
      fwd_enc(200);
      buzz();
      if(!(DL5==a && DL4==a && DL3==a && DL2==a && DR5==a && DR4==a && DR3==a && DR2==a)){
         bwd_enc(100);
         buzz();
         if(!trnSlightlyrgt(2000)){
            trnSlightlylft(4000);
         }
         fwd_enc(100);
      }
      flag = 5;

  }else if(flag == 5){//Turn After Color circle
    dgtl();
    buzz();
    delay(100);
    buzz();
     
    while(true){
      if((DR5==a && DR4==a && DR3==a && DR2==a) || (DL5==a && DL4==a && DL3==a && DL2==a)){
          break; 
      }
      line_flw();
      dgtl();
    }
      brk();
      delay(500);
      if(DR5==a && DR4==a && DR3==a && DR2==a){
         fwd_enc(320);
         trnToWhiteLinergt();
        delay(200);
        line_flw_dur(800);
      }else{
        fwd_enc(320);
        trnToWhiteLinelft();
        delay(200);
        line_flw_dur(800);
      }
      
      flag = 6;


  }else if(flag == 6){//Object detection Circle
      digitalWrite(objDetectorReset, HIGH);
      while(true){
        if(DR5==a && DR4==a && DL4==a && DL5==a){
          break;
        }
        line_flw_circle();
        dgtl();
      }

      brk();
      delay(500);
      fwd_enc(350);
      trnToWhiteLinelft();
      delay(200);
      

      while(true){
        if(DL5==a && DL4==a && DL3==a && DL2==a){
          break;
        }
        line_flw();
        dgtl();
      }
      brk();
      delay(500);
      fwd_enc(350);
      trnToWhiteLinelft();
      delay(1000);
      if(!digitalRead(objType)){
        gems = gems + 10;//Cylinder
      }else{
        gems = gems +20; //cuboid
      }
      
      
      flag = 7;

  }else if(flag == 7){//Go to tht red circle

    dgtl();
    while(true){
      if((DL5==a && DL4==a && DL3==a && DL2==a) || (DR5==a && DR4==a && DR3==a && DR2==a)){
          break; 
      }
      line_flw();
      dgtl();
    }
        
        brk();
        delay(500);
        fwd_enc(350);
        if(DL5==a && DL4==a && DL3==a && DL2==a){
          trnToWhiteLinelft();
          delay(200);
          line_flw_dur(500);

        }else if(DR5==a && DR4==a && DR3==a && DR2==a){
          trnToWhiteLinergt();
          delay(200);
          line_flw_dur(500);
        }
        while(true)
        {
          if(DL5==a && DL4==a && DL3==a && DL2==a && DR5==a && DR4==a && DR3==a && DR2==a){
            break; 
          }
          line_flw();
          dgtl();
        }

        brk();
        delay(500);
        fwd_enc_spd(12,100);
        if(colorCircleTurn == 1){
          trnToWhiteLinergt();
          delay(200);
        }else{
          trnToWhiteLinelft();
          delay(200);
        }  
        fwd_enc(300);
        if(!(DL5==a && DL4==a && DL3==a && DL2==a && DR5==a && DR4==a && DR3==a && DR2==a)){
         bwd_enc(200);
         if(!trnSlightlyrgt(2000)){
            trnSlightlylft(4000);
         }
         fwd_enc(100);
      }

        flag = 8;

  }else if(flag ==8){ //choose red circle
    dgtl();
    while(true){
      if(DL5==a && DL4==a && DL3==a && DL2==a && DR5==a && DR4==a && DR3==a && DR2==a){
          break; 
      }
      line_flw();
      dgtl();
    }
    
      brk();
      delay(500);
      fwd_enc(250);
      delay(500);
      
      int metal = captureCube();
      if(!metal){//go forward if not go right
        brk();
        delay(500);
        fwd_enc(400);
        delay(500);
        trnToWhiteLinergt();
        metal = captureCube();
          if(!metal){//go right if not go left
            brk();
            delay(500);
            fwd_enc(500);
            delay(500);
            trnToWhiteLine180();
            
            metal = captureCube();
           
            brk(); //if there is a metal in left direct to normaal path
            delay(500);
            fwd_enc(400);
            delay(200);
            trnToWhiteLinelft();
            

          }else{
            brk();
            delay(500);
            fwd_enc(400);
            delay(200);
            trnToWhiteLinergt();
            

          }
      }else{
        brk();
        delay(500);
        fwd_enc(400);
        delay(500);
        trnToWhiteLine180();
        
      }

      brk();
      delay(500);
      line_flw_dur(500);
      delay(200);
      
      

      
      flag = 9;
    

  }else if(flag == 9){//bring cube to the hole
    grab_obj();
    dgtl();
    while(true){
      if(DL5==a && DL4==a && DL3==a && DL2==a && DR5==a && DR4==a && DR3==a && DR2==a){
          break; 
      }
      line_flw();
      dgtl();
    }
    brk();
    delay(500);
    fwd_enc_spd(11,90);
    if(!(DL5==a && DL4==a && DL3==a && DL2==a && DR5==a && DR4==a && DR3==a && DR2==a)){
         bwd_enc(100);
         buzz();
         if(!trnSlightlyrgt(2000)){
            trnSlightlylft(4000);
         }
         fwd_enc(100);
      }
    delay(500);
    dgtl();
    while(true){
      if(DL5==a && DL4==a && DL3==a && DL2==a && DL1 ==a && DR1==a){
          break; 
      }
      line_flw();
      dgtl();
    }
    brk();
    delay(500);
    fwd_enc(400);
    trnToWhiteLinelft();

    
    while (true){//drop and push obj to the cave
      if(getDistance()<15){
          break; 
      }
      line_flw();
      delay(10);
      
    }
    brk();
    delay(500);
    bwd_enc_spd(13,90);
    brk();
    delay(500);
    arm_down();
    delay(500);
    release_obj();
    delay(500);
    bwd_enc_spd(6,90);
    brk();
    close_arm();
    delay(500);
    fwd_enc_spd(13,90);
    brk();
    delay(500);
    bwd_enc(250);
    trnToWhiteLine180();
 
    dgtl();
    while (true)
    {
      if(DL5==a && DL4==a && DL3==a && DL2==a && DR5==a && DR4==a && DR3==a && DR2==a){
          break; 
      }
      line_flw();
      dgtl();
      
    }
    brk();
    delay(500);
    fwd_enc(350);
    delay(200);
    trnToWhiteLinelft();

    flag = 10;


  }else if(flag == 10){//go to the second cube selection area
    dgtl();
    while(true){
      if(DL5==a && DL4==a && DL3==a && DL2==a && DR5==a && DR4==a && DR3==a && DR2==a){
          break; 
      }
      line_flw();
      dgtl();
    }
    
      brk();
      delay(500);
      fwd_enc(500);
      trnToWhiteLinelft();
      delay(200);
      line_flw_dur(100);
      
      dgtl();
      while (true)
      {
        if( DL4==a && DL3==a && DL2==a && DL1==a && DR1==a && DR5==a && DR4==a && DR3==a && DR2==a){
            break; 
        }
        line_flw();
        dgtl();
      }
      brk();
      delay(500);
      fwd_enc_spd(12,90);
      
      dgtl();
      while (true)
      {
        if(DL5==a && DL4==a && DL3==a && DL2==a && DR5==a && DR4==a && DR3==a && DR2==a){
            break; 
        }
        line_flw();
        dgtl();
      }
      brk();
      delay(500);
      fwd_enc(350);
      trnToWhiteLinergt();
      delay(200);

      

      
      flag = 11;


  }else if(flag == 11){ // Second cube selection area functions
      dgtl();
    while(true){
      if(DL5==a && DL4==a && DL3==a && DL2==a && DR5==a && DR4==a && DR3==a && DR2==a){
          break; 
      }
      line_flw();
      dgtl();
    }
    
      brk();
      delay(500);
      fwd_enc(250);
      delay(500);
      
      int metal = captureCube();
      if(!metal){//go forward if not go right
        brk();
        delay(500);
        fwd_enc(400);
        delay(500);
        trnToWhiteLinergt();
        metal = captureCube();
          if(!metal){//go right if not go left
            brk();
            delay(500);
            fwd_enc(500);
            delay(500);
            trnToWhiteLine180();
            
            metal = captureCube();
           
            brk(); //if there is a metal in left direct to normaal path
            delay(500);
            fwd_enc(400);
            delay(200);
            trnToWhiteLinelft();
            

          }else{
            brk();
            delay(500);
            fwd_enc(400);
            delay(200);
            trnToWhiteLinergt();
            

          }
      }else{
        brk();
        delay(500);
        fwd_enc(400);
        delay(500);
        trnToWhiteLine180();
        
      }

      brk();
      delay(500);
      line_flw_dur(500);
      delay(200);
      
      

      
      flag = 12;

  }else if(flag == 12){//carry cube to the hole
    dgtl();
    while(true){
      if(DL5==a && DL4==a && DL3==a && DL2==a && DR5==a && DR4==a && DR3==a && DR2==a){
          break; 
      }
      line_flw();
      dgtl();
    }
    brk();
    delay(500);
    fwd_enc(350);
    brk();

    while (true){//drop and push obj to the cave
      if(getDistance()<15){
          break; 
      }
      line_flw();
      delay(10);
      
    }
    brk();
    delay(500);
    bwd_enc_spd(13,90);
    brk();
    delay(500);
    arm_down();
    delay(500);
    release_obj();
    delay(500);
    bwd_enc_spd(6,90);
    brk();
    close_arm();
    delay(500);
    fwd_enc_spd(13,90);
    brk();
    delay(500);
    bwd_enc(250);
    trnToWhiteLine180();
 
    dgtl();
    while (true)
    {
      if(DL5==a && DL4==a && DL3==a && DL2==a && DR5==a && DR4==a && DR3==a && DR2==a){
          break; 
      }
      line_flw();
      dgtl();
      
    }
    brk();
    delay(500);
    fwd_enc(350);
    delay(200);
    trnToWhiteLinelft();

    flag = 13;


  }


  else if(flag==-1){

    while (true)
    {
      
      bwd_enc_spd(6,80);
     
    }
    
    
    
    //flag =-2;
  }





  //line_flw();
  }

////////////////////////////////////////Cube Detection /////////////////////////////////////////////
int captureCube(){
  dgtl();
  int distanceIn = 0;
  int isThereCube = 1 ;
  dgtl();
  while(true){//Cube detection
          if((DL4==a && DL3==a && DL2==a && DL1==a && DR1 == a && DR4==a && DR3==a && DR2==a ) ){  //getDistance()<2 ||
              isThereCube = 0;
              break;
          }else if(getDistance()<4){
              distanceIn = 1;
              break;
          }
          line_flw_given_spd(80);
          dgtl();
          delay(10);
        
        }
      brk();
      delay(500);
      bwd_enc(450);
      int metal = 0;
      if(isThereCube == 1){//if there is a cube run this
      //   if(distanceIn != 1){//if there is a cube in front of the robot
      //   bwd_enc(310); 
      // }
        
        delay(500);
        arm_down();
        delay(500);
        fwd_enc(250);
        metal = isMetal();
        delay(500);

      }else{
        bwd_enc(300);// if there is no cube backward robot
      }
      

      delay(1000);

      if(metal){
        grab_obj();
        delay(500);
        arm_up();
        delay(500);
        bwd_enc(200);
      }else{
        if(isThereCube ==1){
          bwd_enc(200);
          delay(500);
          arm_up();
        }
        
      }
      dgtl();
      while(true){
      if(DL4==a && DL3==a && DL2==a && DL1==a && DR1==a && DR4==a && DR3==a && DR2==a ){
          break; 
      } 
      line_flw_bwd();
      dgtl();
      
    }

    return metal;

}


////////////////////////////////////////Metal Detect///////////////////////////////////////////////////////////

int isMetal(){
  int count = 0;
  int loop = 50;
  for(int i = 0 ; i<loop ;i++){
    if(analogRead(metalDetector)>800){
      count++;
    }
    delay(10);
    
  }
  if(count>loop/2){
      return 1;
    }
  return 0;


}

//////////////////////////////////////////////////Distance//////////////////////////////////////////////////////
int getDistance() {

  digitalWrite(trigPinFront, LOW);
  delayMicroseconds(2) ;
  digitalWrite(trigPinFront, HIGH);
  delayMicroseconds(30);
  digitalWrite(trigPinFront, LOW);

  long duration  = pulseIn(echoPinFront, HIGH);
  int distance = duration * 0.034 / 2;

  //int distance = sonar.ping_cm();
  return distance;
} 



/////////////////////////////////////////////////////////////////////////////Turn///////////////////////////////////////////////////////////////////////////
void trn_Precise_180(){

  int enc_val=800;
  mtr_cmd(0, 0);
  delay(1000);

  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;
  dgtl();
  while (RnewPosition > -enc_val && LnewPosition < enc_val) {

    mtr_cmd(100, -100);
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
  
  }
}
void trnToWhiteLine180(){

  trn_Precise_180();
  dgtl();
  while(!(DR1==a && DL1==a && DL2==a)){
    mtr_cmd(100, -100);
    delay(100);
    mtr_cmd(0,0);
    delay(100);
    dgtl();
  }


}



void trn_180(){
  int enc_val=550;
  mtr_cmd(0, 0);
  delay(1000);

  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;
  dgtl();
  while (RnewPosition > -enc_val*2 && LnewPosition < enc_val*2 ) {

    mtr_cmd(100, -100);
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
  }


  mtr_cmd(0, 0);
  delay(1000);

  fwd_enc(175);
}


void trn_lft() {
  int enc_val=560;
  

  mtr_cmd(0, 0);
  delay(1000);

  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;


  while (LnewPosition > -enc_val && RnewPosition < enc_val) {

    mtr_cmd(-100, 100);
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
  }

  mtr_cmd(0, 0);
  delay(1000);

  //fwd_enc(175);


}

void trn_rgt() {

  int enc_val = 560;
  

  mtr_cmd(0, 0);
  delay(1000);

  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;

  while (RnewPosition > -enc_val && LnewPosition < enc_val) {

    mtr_cmd(100, -100);
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
  }

  
  mtr_cmd(0, 0);
  delay(1000);

  fwd_enc(175);  
  
}
///////////////////////////Precise Right Turn///////////////////////////////
void trn_Precice_rgt() {

  int enc_val = 350;
  
  

  mtr_cmd(0, 0);
  delay(1000);

  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;

  while (RnewPosition > -enc_val && LnewPosition < enc_val) {

    mtr_cmd(100, -100);
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
  }
  mtr_cmd(0, 0);
  delay(1000); 
  
}
void trnToWhiteLinergt(){

  trn_Precice_rgt();
  dgtl();
  while(!(DR1==a && DL1==a && DL2==a)){
    mtr_cmd(100, -100);
    delay(100);
    mtr_cmd(0,0);
    delay(100);
    dgtl();
  }
  
}


///////////////////////////iwarai///////////////////////////////////////////
///////////////////////Slightly Right Turn///////////////////////////////
int trnSlightlylft(int time){

  stt_time=millis();
  elp_time=0;
  dgtl();
  while(time>elp_time){
    if((DR1==a && DL1==a && DR2==a)){
      return 1;
    }
    mtr_cmd(-100, 100);
    delay(100);
    mtr_cmd(0,0);
    delay(100);
    dgtl();
    elp_time=millis()-stt_time;
  }
  return 0;
}

int trnSlightlyrgt(int time){

  stt_time=millis();
  elp_time=0;
  dgtl();
  while(time>elp_time){
    if((DR1==a && DL1==a && DL2==a)){
      return 1;
    }
    mtr_cmd(100, -100);
    delay(100);
    mtr_cmd(0,0);
    delay(100);
    dgtl();
    elp_time=millis()-stt_time;
  }
  return 0;
  
}

//////////////////////////////////// Precise left Turn /////////////////////////////////
void trn_Precice_lft() {

  int enc_val = 350;
  
  mtr_cmd(0, 0);
  delay(1000);

  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;


  while (LnewPosition > -enc_val && RnewPosition < enc_val) {

    mtr_cmd(-100, 100);
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
  }

  mtr_cmd(0, 0);
  delay(500);
  
}
void trnToWhiteLinelft(){

  trn_Precice_lft();
  dgtl();
  while(!(DR1==a && DL1==a && DR2==a)){
    mtr_cmd(-100, 100);
    delay(100);
    mtr_cmd(0,0);
    delay(100);
    dgtl();
  }
  
}



///////////////////////////////////////////////////////////////////////////Turn///////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////ARM///////////////////////////////////////////////////////////////////////////


void catch_obj() {
  
  armX.write(180);
  delay(500);
  armY.write(112);  
  delay(1000);
  armX.write(70);
  delay(500);
  armY.write(50);
  delay(1000);

  // int arm_pos = 0;
  // while (arm_pos < 180) {
  //   arm_pos += 1;
  //   armX.write(arm_pos);
  //   delay(10);
  // }
}

void drop_obj() {

  armY.write(112);
  delay(500);
  armX.write(180);
  delay(1000);

  // int arm_pos = 0;
  // while (arm_pos < 180) {
  //   arm_pos += 1;
  //   armX.write(arm_pos);
  //   delay(10);
  // }
}

void arm_down(){
  for(int i=50 ; i<=128 ; i++){
    armY.write(i);
    delay(10);
  }
 
}

void grab_obj(){
  armX.write(57);
  delay(300);

}
void close_arm(){
  armX.write(0);
  delay(300);
}

void arm_up(){
  for(int i=128;i>=50;i--){
    armY.write(i);
    delay(10);
  }
}

void release_obj(){
  armX.write(180);
  delay(300);

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
  
  mtr_cmd((90 - lf_dif/2.5),(90 + lf_dif/2.5));

  lf_prverr = lf_err;

  
 
}
void line_flw_given_spd(int speed) {//White line


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
  
  mtr_cmd((speed - lf_dif/2.5),(speed + lf_dif/2.5));

  lf_prverr = lf_err;

  
 
}


void bwd_line_flw() {//White line


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
  
  mtr_cmd(-1*(80 + lf_dif/2),-1*(80 - lf_dif/2));

  lf_prverr = lf_err;

  
 
}

void line_flw_circle() {//White line


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
  
  mtr_cmd((70 - lf_dif/2.5),(70 + lf_dif/2.5));

  lf_prverr = lf_err;

  
 
}

void line_flw_dur(int time_){
  stt_time=millis();
  elp_time=0;
  while(time_>elp_time){
    line_flw();
    elp_time=millis()-stt_time;
  }
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
  
   mtr_cmd(80 - lf_dif, 80 + lf_dif);

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

  mtr_cmd(-1*(80 - lf_dif/2.5), -1*(80 + lf_dif/2.5));

  lf_bwd_prverr = lf_bwd_err;

}

void line_flw_fwd() {
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


  mtr_cmd((80 - lf_dif), (80 + lf_dif));

  lf_bwd_prverr = lf_bwd_err;

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





//////////////////////////////////////////////////////////////////////Forward PID///////////////////////////////////////////////////////////////////////////
void fwd_enc_distance(float target_distance){
  int distance_per_pulse = 0.02;
  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;
  float initial_distance = 0;
  
  while (initial_distance <= target_distance) {
    enc_err = (-1) * LnewPosition + RnewPosition;

    mtr_cmd(110 + enc_err, 110 - enc_err);
    
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
    
    float distance_traveled = ((LnewPosition + RnewPosition) / 2.0) * distance_per_pulse;
    initial_distance += distance_traveled;
  }
  mtr_cmd(0, 0);
}


void fwd_enc(int duration){
  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;
  elp_time=0;
  stt_time=millis();

  while (elp_time<=duration) {
    enc_err=(-1)*LnewPosition+RnewPosition;

    mtr_cmd(110+enc_err,110-enc_err);
    
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
    elp_time=millis()-stt_time;
  }
  mtr_cmd(0, 0);

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

    mtr_cmd(-1*(100-enc_err),-1*(100+enc_err));
    
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
    elp_time=millis()-stt_time;
  }

  mtr_cmd(0, 0);
}
void bwd_enc_givenSpd(int duration , int spd){
  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;
  elp_time=0;
  stt_time=millis();

  while (elp_time<=duration) {
    enc_err=(-1)*LnewPosition+RnewPosition;

    mtr_cmd(-1*(spd-enc_err),-1*(spd+enc_err));
    
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
    elp_time=millis()-stt_time;
  }

  mtr_cmd(0, 0);
}
void fwd_enc_givenSpd(int duration , int spd){
  ev1 = LEnc.read();
  ev2 = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;
  elp_time=0;
  stt_time=millis();

  while (elp_time<=duration) {
    enc_err=(-1)*LnewPosition+RnewPosition;

    mtr_cmd((spd+enc_err),(spd-enc_err));
    
    LnewPosition = LEnc.read() - ev1;
    RnewPosition = REnc.read() - ev2;
    elp_time=millis()-stt_time;
  }

  mtr_cmd(0, 0);
}



void bwd_enc_spd(float target_distance, int spd) {  
  LEnc.readAndReset();
  REnc.readAndReset();
  int ev1bwd = LEnc.read();
  int ev2bwd = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;
  float distance_traveled = 0;
  
  while (distance_traveled <= target_distance) { // Adjusted condition for backward movement
    
    
    
    enc_err = (-1) * LnewPosition + RnewPosition;

    mtr_cmd((-1)*(spd - enc_err), (-1)*(spd + enc_err)); // Adjusted motor commands for backward movement

    LnewPosition =   LEnc.read() - ev1bwd;
    RnewPosition =  REnc.read() - ev2bwd ;
    
  //  Serial.print(" EV(L:R): ");
  // Serial.print(ev1);
  // Serial.print(":");
  // Serial.println(ev2);
  // Serial.print(" Encoder Value(L:R): ");
  // Serial.print(LnewPosition);
  // Serial.print(":");
  // Serial.println(RnewPosition);
    
  distance_traveled = ((LnewPosition + RnewPosition) / 2.0) * 0.02; // Negated distance_traveled for correct backward movement
  Serial.println(distance_traveled);
    
    delay(0);
  }
  mtr_cmd(0, 0);
}




void fwd_enc_spd(float target_distance,int spd){
  LEnc.readAndReset();
  REnc.readAndReset();
  int ev1fwd = LEnc.read();
  int ev2fwd = REnc.read();
  LnewPosition = 0;
  RnewPosition = 0;
  float distance_traveled = 0;
  
  while (distance_traveled <= target_distance) {
    enc_err = (-1) * LnewPosition + RnewPosition;

    mtr_cmd(spd + enc_err, spd - enc_err);
    
    LnewPosition = LEnc.read() - ev1fwd;
    RnewPosition = REnc.read() - ev2fwd;
    
    distance_traveled = (-1)*((LnewPosition + RnewPosition) / 2.0) * 0.02;
    
    Serial.println(distance_traveled);
    delay(0);
  }
  mtr_cmd(0, 0);
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
  // Serial.print(DL5);
  // Serial.print("    ");
  // Serial.print(DL4);
  // Serial.print("    ");
  // Serial.print(DL3);
  // Serial.print("    ");
  // Serial.print(DL2);
  // Serial.print("    ");
  // Serial.print(DL1);
  // Serial.print("    ");
  // Serial.print(DR1);
  // Serial.print("    ");
  // Serial.print(DR2);
  // Serial.print("    ");
  // Serial.print(DR3);
  // Serial.print("    ");
  // Serial.print(DR4);
  // Serial.print("    ");
  // Serial.print(DR5);
  // Serial.println("    ");


}
/////////////////////////////////////////////////////////////////IR Digitalize///////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////Color Detection /////////////////////////////////////////////////////////////////////////

int colorDetect1() {

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
    return 1;
  }
  else if (rednew1 > greennew1 && greennew1 < bluenew1) { //GREEN
    return 2;
  }
  else if (bluenew1 < greennew1 && rednew1 > bluenew1) { //BLUE
    return 3;
  }else{
    return 0;
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

int colorDetect2() {

  digitalWrite(S22, LOW);
  digitalWrite(S32, LOW);
  red2 = pulseIn(OUT2, LOW);
  redColor2 = map(red2, red_min2, red_max2, 255, 0);

  digitalWrite(S22, HIGH);
  digitalWrite(S32, HIGH);
  green2 = pulseIn(OUT2, LOW);
  Serial.println(green2);
  Serial.print("Green Thres");
  
  greenColor2 = map(green2, green_min2, green_max2, 255, 0);

  digitalWrite(S22, LOW);
  digitalWrite(S32, HIGH);
  blue2 = pulseIn(OUT2, LOW);
  
  blueColor2 = map(blue2, blue_min2, blue_max2, 255, 0);

  total2 = red2 + green2 + blue2;


  rednew2 = (red2 / total2) * 100;
  greennew2 = (green2 / total2) * 100;
  bluenew2 = (blue2 / total2) * 100;
  

  if (rednew2 < greennew2 && rednew2 < bluenew2) { //RED
    return 1; //red
  }
  else if (rednew2 > greennew2 && greennew2 < bluenew2) { //GREEN
    return 2; //green
  }
  else if (bluenew2 < greennew2 && rednew2 > bluenew2) { //BLUE
    return 3; //blue

  }else{
    return 0; //none
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

