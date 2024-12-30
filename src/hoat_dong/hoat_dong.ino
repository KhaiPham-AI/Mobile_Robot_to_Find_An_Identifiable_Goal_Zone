#include<Arduino.h>
#include <ArduinoJson.h>


#define EncoderALeft 2
#define EncoderARight 3
#define EncoderBLeft 6
#define EncoderBRight 7
#define ENA 12                  // bÃ¡nh pháº£i
#define ENB 11                  // bÃ¡nh trÃ¡i
#define IN1 10
#define IN2 9
#define IN3 4
#define IN4 5



// nháº­n dá»¯ liá»‡u tá»« jetson
int action = 0;
int script = 0;
float angle = 0.0;


volatile long countPulseLeft = 0;
volatile long countPulseRight = 0;

// test variable
long test_left = 0;
long test_right = 0;

// globals:
long prevTime = 0;

int prevPulseRight = 0;
int prevPulseLeft = 0;

volatile int pulseRight = 0;
volatile int pulseLeft = 0;

volatile float accelerationR_i = 0;
volatile float accelerationL_i = 0;

volatile long prevTime_i = 0;

float prevErrRight = 0;
float preErrLeft = 0;


// PID cá»§a Ä‘iá»u khiá»ƒn Angle

const float Kp_angle = 0.8;
const float Ki_angle = 0;
const float Kd_angle = 0;
float normal_speed = 0.0 / 60; 

float error = 0;
float previous_error = 0;
float P = 0, I = 0, D = 0;
float  PID_value = 0;
long prevTimeMain = 0;







// PID parameters:
float kp = 940;
float ki = 4;
float kd = 5;

float targetR = 0; // round per second
float targetL = 0; // max 1.6 ~ 100/60

float eintegralR = 0;
float eintegralL = 0;

float dedtR = 0;
float dedtL = 0;

void encoderLeft();
void encoderRight();
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) {
    ; // Ã?i c?ng serial k?t n?i (ch? c?n thi?t trÃªn Leonardo, Micro, v.v.)
  }
  pinMode(EncoderALeft, INPUT_PULLUP);
  pinMode(EncoderBLeft, INPUT_PULLUP);
  pinMode(EncoderARight, INPUT_PULLUP);
  pinMode(EncoderBRight, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncoderALeft),encoderLeft,RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderARight),encoderRight,RISING);


  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  prevTime = micros();
  prevTimeMain = micros();

}
  void RightMotorStop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  void LeftMotorStop() {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  void RightForward() {
    digitalWrite(IN1, HIGH); 
    analogWrite(IN2, LOW);
  }

  void RightBackward() {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, HIGH);
  }

  void LeftForward() {
    analogWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  void LeftBackward() {
    analogWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
  }

  void encoderLeft() {
    int ValueEncoderLeft = digitalRead(EncoderBLeft);
    if (ValueEncoderLeft > 0) {
      countPulseLeft++;
    }
    else {
      countPulseLeft--;
    }
  }

  void encoderRight() {
    int ValueEncoderRight = digitalRead(EncoderBRight);
    if (ValueEncoderRight > 0) {
      countPulseRight--;
    }
    else {
      countPulseRight++;
    }
  }

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) { // pwmVal: 0 ~ 255;
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    // Turn forward:
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);  
  }
    // Turn backward:
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
}

void loop() {
  if (Serial.available()) {
      StaticJsonDocument<200> doc;
      String jsonString = Serial.readStringUntil('\n');
      DeserializationError error = deserializeJson(doc, jsonString);
      if (error) {
        return;
      }
      action = doc["action"];
      script = doc["script"];
      angle = doc["angle"];
      if(action == -1){
      normal_speed = doc["normal_speed"];
      normal_speed /= 60.0;
      }
      
      // Serial.print("action: ");
      // Serial.print(action);
      // Serial.print(" script: ");
      // Serial.print(script);
      // Serial.print(" angle: ");
      // Serial.println(angle);
    }

  if(action == 1){
    if(script == 0 || script == 2){
      targetR = normal_speed - (100.0/60-normal_speed) * Kp_angle * angle/90 ;
      targetL = normal_speed + (100.0/60-normal_speed) * Kp_angle * angle/90;
      if(targetR >100.0/60){
        targetR = 100.0/60;
      }
      if(targetL >100.0/60){
        targetL = 100.0/60;
      }
      if(targetR < -100.0/60){
        targetR = -100.0/60;
      }
      if(targetL < -100.0/60){
        targetL = -100.0/60;
      }

    } else {
      targetL = 60.0 / 60;
      targetR = -60.0 / 60;
    }
  noInterrupts();
  test_left = countPulseLeft;
  test_right = countPulseRight;
  interrupts();
  
  //put your main code here, to run repeatedly:
  

  // Serial.print(">xung left: ");
  // Serial.println(test_left);
  // Serial.print(">xung right: ");
  // Serial.println(test_right);

  noInterrupts();
  pulseRight = countPulseRight;
  pulseLeft = countPulseLeft;
  interrupts();

// Serial.print(">xung left: ");
// Serial.println(pulseLeft);
// Serial.print(">xung right: ");
// Serial.println(pulseRight);

  int pulseRight = 0;
  int pulseLeft = 0;

  noInterrupts();
  pulseRight = countPulseRight;
  pulseLeft = countPulseLeft;
  interrupts();

  long currentTime = micros();
  float deltaT = ((float)(currentTime - prevTime))/1.0e6;

  float velocityRight = (pulseRight - prevPulseRight)/495.0/deltaT;
  float velocityLeft = (pulseLeft - prevPulseLeft)/495.0/deltaT;

  prevPulseRight = pulseRight;
  prevPulseLeft = pulseLeft;

  prevTime = currentTime;

// Serial.print(">Velocity-Right: ");
// Serial.println(velocityRight);
// Serial.print(">Velocity-Left: ");
// Serial.println(velocityLeft);

  float v0R = velocityRight;
  float v0L = velocityLeft;

  float vtR = targetR;
  float vtL = targetL;

  float errorR = vtR - v0R;
  float errorL = vtL - v0L;

// Serial.print(">Error - Right: ");
// Serial.println(errorR);
// Serial.print(">Error - Left: ");
// Serial.println(errorL);

// Serial.print(">Target - error right: ");
// Serial.println(100*(vtR-velocityRight)/vtR);
// Serial.print(">Target - error left: ");
// Serial.println(100*(vtL-velocityLeft)/vtL);
// Serial.print(">Target - Right: ");
// Serial.println(vtR);
// Serial.print(">Target - Left: ");
// Serial.println(vtL);


  eintegralR = eintegralR + errorR * deltaT;
  eintegralL = eintegralL + errorL * deltaT;
  dedtR = (errorR - prevErrRight)/deltaT;
  dedtL = (errorL - preErrLeft)/deltaT;

  float uR = kp*errorR + ki*eintegralR + kd * dedtR;
  float uL = kp*errorL + ki*eintegralL + kd * dedtL;
  preErrLeft = errorL;
  prevErrRight = errorR;

// Serial.print(">PID control signal - Right: ");
// Serial.println(uR);

// Serial.print(">PID control signal - Left: ");
// Serial.println(uL);

  int dirR = 1;
  if (uR > 0) {
    dirR = -1;
  }

  int dirL = 1;
  if (uL > 0) {
    dirL = -1;
  }

  int pwrR = (int) fabs(uR);
  if (pwrR > 255) {
    pwrR = 255;
  }

  int pwrL = (int) fabs(uL);
  if (pwrL > 255) {
    pwrL = 255;
  }

  setMotor(dirR, pwrR, ENA, IN1, IN2); //Right 

  setMotor(dirL, pwrL, ENB, IN4, IN3); //Left

  } else {
    setMotor(1, 0, ENA, IN1, IN2); //Right 

    setMotor(1, 0, ENB, IN4, IN3); //Left
  }



  


/*
long currentTime = micros();
float deltaT = ((float)(currentTime - prevTime))/1.0e6;

float velocityRight = (test_right - prevPulseRight)/495.0/deltaT;
float velocityLeft = (test_left - prevPulseLeft)/495.0/deltaT;
prevTime = currentTime;
prevPulseRight = test_right;
prevPulseLeft = test_left;

Serial.print(">Vright: ");
Serial.println(velocityRight);
Serial.print(">Vleft: ");
Serial.println(velocityLeft);
*/

}
