#include <Servo.h>
/*
 * Pin Discription
 * 1. -----
 * 2. AttachIntterupt Encoder Kiri
 * 3. AttachIntterupt Encoder Kanan
 * 4. Echo Pin
 * 5. Trigger Pin
 * 6. rpwm
 * 7. ldir1
 * 8. 
 * 9. rdir1 
 * 10. rdir2
 * 11. lpwm
 * 12. rdir1
 * 13. ------------
 *  
 */


Servo servo; 


//-----Require For Motor Driver L293N ----------------------
//First motor
#define ldir2 8
#define ldir1 7
#define lpwm 11  

//Second motor                                                                                                          
#define rdir2 10 
#define rdir1 12 
#define rpwm 6 
//----------------End L293N---------------------------------

//------Require For Echo and trigger Pin SR04---------------
#define echoPin 4 
#define initPin 5
unsigned long pulseTime = 0;  
//-----------End HC-SR04------------------------------------

//-------Require For Servo----------------------------------
#define servopin 9
int pos = 0;
//-------------End Servo------------------------------------

//------------Encorder ------------------------------------
#define encoder_kanan 1
#define encoder_kiri 0
//---------------------------------------------------------

//-----------------------------------------------
#define rEncoder 2
#define lEncoder 3
//-------------------------------------------------

volatile unsigned long rightWheel;

volatile unsigned long leftWheel;

byte maxspeed = 150;

byte leftSpeed = maxspeed;
byte rightSpeed = maxspeed;

double input, output, leftLastError, poportional, derivative, rightLastError;
double rightIntegral = 0;
double leftIntegral = 0;

int pidcount = 1;

double kp = 2;
double ki = 0.3;
double kd = 0.5;

double setPoint = 20;


void setup()
{
    Serial.begin(9600);
    //Setup For Servo
    servo.attach(servopin);

    pinMode(rEncoder, INPUT);
    pinMode(lEncoder, INPUT);

    //Setup For L293N
    pinMode(ldir2,OUTPUT);
    pinMode(ldir1,OUTPUT);
    pinMode(lpwm,OUTPUT);
    pinMode(rdir2,OUTPUT);
    pinMode(rdir1,OUTPUT);
    pinMode(rpwm,OUTPUT);
    
    //Setup Sr04
    pinMode(initPin, OUTPUT);
    pinMode(echoPin, INPUT); 

    digitalWrite(rEncoder, HIGH);
    digitalWrite(lEncoder, HIGH);

    attachInterrupt(1, rightEncoderISR, FALLING); 
    attachInterrupt(0, leftEncoderISR, FALLING);

    digitalWrite(ldir1, LOW);
    digitalWrite(ldir2, HIGH);
 
    digitalWrite(rdir1, LOW);
    digitalWrite(rdir2, HIGH);
} 

//-----------------Interrutp Encoder-----------------------

//void encoderA(){
//  if (digitalRead(2) == HIGH) { 
//
//    if (digitalRead(3) == LOW || digitalRead(3) == HIGH) {  
//        rightWheel++;         
//    } 
//    else {
//      rightWheel--;         
//    }
//  }
//}
//
//void encoderB(){
//  if (digitalRead(3) == HIGH) { 
//
//    if (digitalRead(2) == LOW || digitalRead(2) == HIGH) {  
//        leftWheel++;         
//    } 
//    else {
//      leftWheel--;         
//    }
//  }
//}

//------------------------------------------------------

//--------Turn Right Left With Encoder --------------
 void TurnWithEnco(int encoder, int pick){
  if(encoder == 0){
    rightWheel = 255;
    leftWheel = 0;
    while(leftWheel <= pick ){
      Serial.println(leftWheel);
      Kanan(130,130);
    }
  }
  if(encoder == 1){
    rightWheel = 0;
    leftWheel = 255;
   while(rightWheel <= pick ){
      Serial.println(rightWheel);
      Kiri(130,130);
    }
  }
  rightWheel = 0;
  leftWheel = 0;
 }

//-------------------------------------------------

//--------Back Up With Encoder --------------
 void BackupWithEnco(int encoder1, int encoder0 , int pick){
  //Serial.print("Masuk Mundur");
  if(encoder0 == 0 and encoder1 == 1){
    rightWheel = 0;
    leftWheel = 0;
    //Serial.println("masuk if mundur");
    while((rightWheel <= pick) && (leftWheel <= pick) ){
      Serial.println("");
      BackUp(100,100);
    }
  }
  //counterA = 0;
  //counterB = 0;
 }

//-------------------------------------------------

//--------Procedure for Motor-----

void MotorForward_and_Stop(int right, int left){
	analogWrite(lpwm, right);
	analogWrite(rpwm, left); 
	//---------First Motor-----------
	digitalWrite(ldir2, HIGH);
	digitalWrite(ldir1, LOW);
	//---------Second Motor----------
	digitalWrite(rdir2, HIGH);
	digitalWrite(rdir1, LOW);
}

void Kanan(int right, int left){
  analogWrite(lpwm, right);
  analogWrite(rpwm, left); 
  //---------First Motor-----------
  digitalWrite(ldir2, LOW);
  digitalWrite(ldir1, HIGH);
  //---------Second Motor----------
  digitalWrite(rdir2, HIGH);
  digitalWrite(rdir1, LOW);
}

void Kiri(int right, int left){
  analogWrite(lpwm, right);
  analogWrite(rpwm, left); 
  //---------First Motor-----------
  digitalWrite(ldir2, HIGH);
  digitalWrite(ldir1, LOW);
  //---------Second Motor----------
  digitalWrite(rdir2, LOW);
  digitalWrite(rdir1, HIGH);
}

void BackUp(int right, int left){
	analogWrite(lpwm, right);
	analogWrite(rpwm, left); 
	//---------First Motor-----------
	digitalWrite(ldir2, LOW);
	digitalWrite(ldir1, HIGH);
	//---------Second Motor----------
	digitalWrite(rdir2, LOW);
	digitalWrite(rdir1, HIGH);
}

//----------------------End Procedure L293N----------------

//-------------Procedure For Servo-----------------------
void Servo_spin(int pos_init, int pos_goal){
  for(pos = pos_init; pos < pos_goal; pos++){                                  
    servo.write(pos);              
    delay(7);                       
  }
}

void Servo_back_pos(int pos_init, int pos_goal){
  for(pos = pos_init; pos>=pos_goal; pos-=1)    
  {
    servo.write(pos);              
    delay(7);                       
  }
}
//----------------End Procedure Servo---------------------

//------------------HC-SR04 Procedure---------------------
long microsecondsToCentimeters(long microseconds)
{
    return microseconds / 29 / 2;
}

long get_distance_sr04(){
	digitalWrite(initPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(initPin, LOW);
	pulseTime = pulseIn(echoPin, HIGH);	
	return microsecondsToCentimeters(pulseTime);
}

long spin_and_get_direction(int pos_init, int pos_goal){
	if(pos_init > pos_goal){
		Servo_back_pos(pos_init,pos_goal);
	}
	else Servo_spin(pos_init,pos_goal);

	return get_distance_sr04();
}

//--------End Procedure HC-SR04---------

//--------Count For Stop--------------------
bool waittostop(){
  bool interrupt = true;
  int cin = 0;
  MotorForward_and_Stop(0,0);
  for(int i=0;i<=10;i++){
    if(get_distance_sr04()>200) return false;
    if(get_distance_sr04()>30){
      cin++;
      if(cin > 8){ return false;}
    }
  }
  return interrupt;
}

//-------------------------------------


//------------------------------------

void loop()
{

  if (get_distance_sr04() < 50) {
    rightWheel = 0;
    leftWheel = 0;
    if(waittostop()){
      MotorForward_and_Stop(0,0);
      long kanan_dist = spin_and_get_direction(90,0);
      delay(100);
      long depan_dist = spin_and_get_direction(0,90);
      delay(100);
      long kiri_dist = spin_and_get_direction(90,180);
      delay(100);
      Servo_back_pos(180,90);
      delay(50);
      if(kanan_dist <= 10 and kiri_dist <=10) {BackupWithEnco(encoder_kanan,encoder_kiri,30);delay(300);}
      //---------------------------------------------------------------------
      if(kanan_dist > 10 and kiri_dist <=10 and depan_dist <=7){BackupWithEnco(encoder_kanan,encoder_kiri,30);MotorForward_and_Stop(0,0);delay(300);TurnWithEnco(encoder_kiri,20);}
      if(kanan_dist <= 10 and kiri_dist > 10 and depan_dist <=7){BackupWithEnco(encoder_kanan,encoder_kiri,30);MotorForward_and_Stop(0,0);delay(300);TurnWithEnco(encoder_kanan,20);}
      //------------------------------------------------------------------------------------------------------------------------------
      if(kanan_dist > 10 and kiri_dist <=10 and depan_dist > 7){TurnWithEnco(encoder_kiri,20);}
      if(kanan_dist <= 10 and kiri_dist > 10 and depan_dist > 7){TurnWithEnco(encoder_kanan,20);}
      //--------------------------------------------------------------------------------------------------
      if(kanan_dist > 10 and kiri_dist > 10 and depan_dist <=7 and kanan_dist > kiri_dist){BackupWithEnco(encoder_kanan,encoder_kiri,30);MotorForward_and_Stop(0,0);delay(300);TurnWithEnco(encoder_kiri,20);}
      if(kanan_dist > 10 and kiri_dist > 10 and depan_dist <=7 and kanan_dist < kiri_dist){BackupWithEnco(encoder_kanan,encoder_kiri,30);MotorForward_and_Stop(0,0);delay(300);TurnWithEnco(encoder_kanan,20);}
      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      if(kanan_dist > 10 and kiri_dist > 10 and depan_dist > 7 and kanan_dist < kiri_dist){TurnWithEnco(encoder_kanan,20);}
      if(kanan_dist > 10 and kiri_dist > 10 and depan_dist > 7 and kanan_dist > kiri_dist){TurnWithEnco(encoder_kiri,20);}
      //---------------------------------------------------------------------------------------------------------------------------
      }
     MotorForward_and_Stop(0,0);
  }

  input = rightWheel;
  rightWheel = 0;
 
  poportional = setPoint - input;
  derivative = poportional - rightLastError;
  rightIntegral = (rightIntegral + poportional)/pidcount;
 
  output = kp * poportional + kd * derivative + ki * rightIntegral;
 
  rightLastError = poportional;
 
  if((rightSpeed + output) > 150) rightSpeed = 150;
  else rightSpeed = output + rightSpeed;
  Serial.println(rightSpeed);
  analogWrite(rpwm, rightSpeed);
 
  input = leftWheel;
  leftWheel = 0;

  poportional = setPoint - input;
  derivative = poportional - leftLastError;
  leftIntegral = (leftIntegral + poportional)/pidcount;
 
  output = kp * poportional + kd * derivative + ki * leftIntegral;
 
  leftLastError = poportional;
  pidcount++;

  if((leftSpeed + output) > 130) leftSpeed = 130;
  else leftSpeed = output + leftSpeed;
  Serial.println(leftSpeed);
  analogWrite(lpwm, leftSpeed);
 
  delay(100);
  //---------------------------------------------------------------
	//long gap = get_distance_sr04();
  //if(gap > 50 ) MotorForward_and_Stop(255,150);
  //else if(gap > 30 and gap <=50) MotorForward_and_Stop(150,150);
  //else if(gap > 20 and gap <=30) MotorForward_and_Stop(150,150);
  //else if(gap > 15 and gap <=20) MotorForward_and_Stop(110,110);
  //else if(gap > 10 and gap <=15) MotorForward_and_Stop(90,90);
	
//-------------------------
}

void rightEncoderISR(){
  rightWheel++;
}

void leftEncoderISR(){
  leftWheel++;
}
