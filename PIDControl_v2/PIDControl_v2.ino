#include <PinChangeInterrupt.h>
#include <PID_v1.h>                                   
#define encodPinA1      2                             // Quadrature encoder A pin
#define encodPinB1      3                             // Quadrature encoder B pin

#define ldir2 8
#define ldir1 7
#define lpwm 11

#define rdir2 10
#define rdir1 12
#define rpwm 6

double kp = 5 , ki = 1 , kd = 0.01 , input = 0, output = 0, setpoint = 1000;  // modify kp, ki and kd for optimal performance
long temp;
volatile long encoderPos = 0;
volatile long rightWheel = 0, leftWheel = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

void setup() {
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(0, rightEncoderISR, FALLING);               // update encoder position
  attachInterrupt(1, leftEncoderISR, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
}
void loop() {

  setpoint = temp / 500;                              // modify division to fit motor and encoder characteristics
  input = rightWheel ;                                // data from encoder
  myPID.Compute();                                    // calculate new output
  pwmOut(output, 1);                                  // drive L298N H-Bridge module
}

void pwmOut(int out, int direction_t) {               
  if (direction_t == 1 ) {                            // Forward
    digitalWrite(ldir1, HIGH);
    digitalWrite(ldir2, LOW);
    digitalWrite(rdir1, HIGH);
    digitalWrite(rdir2, LOW);
    analogWrite(lpwm, out);
    analogWrite(rpwm, out);
  }
  if (direction_t == 2 ) {                            // Backward
    digitalWrite(ldir1, LOW);
    digitalWrite(ldir2, HIGH);
    digitalWrite(rdir1, LOW);
    digitalWrite(rdir2, HIGH);
    analogWrite(lpwm, out);
    analogWrite(rpwm, out);
  }
  if (direction_t == 3 ) {                            // Turn Left
    digitalWrite(ldir1, LOW);
    digitalWrite(ldir2, HIGH);
    digitalWrite(rdir1, HIGH);
    digitalWrite(rdir2, LOW);
    analogWrite(lpwm, out);
    analogWrite(rpwm, out);
  }
  if (direction_t == 4 ) {                            // Turn Right
    digitalWrite(ldir1, HIGH);
    digitalWrite(ldir2, LOW);
    digitalWrite(rdir1, LOW);
    digitalWrite(rdir2, HIGH);
    analogWrite(lpwm, out);
    analogWrite(rpwm, out);
  }
}

void rightEncoderISR() {
  if (digitalRead(encodPinA1) == HIGH) rightWheel++;
  else rightWheel--;
}

void leftEncoderISR() {
  if (digitalRead(encodPinB1) == HIGH) leftWheel++;
  else leftWheel--;
}
