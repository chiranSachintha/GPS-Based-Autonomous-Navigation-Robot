#include "Ublox.h"
#define SERIAL_BAUD 9600
#define GPS_BAUD 9600
#define N_FLOATS 4

#define destlon 79.8991639
#define destlat 6.7982118


#define gpsTol 0.00001
#define compTol 5

#include <Wire.h>
#include "compass.h"

#define Task_t 10          // Task Time in milli seconds

int dt = 0;
unsigned long t;

int crntheading, destheading;
float disttodest;

Ublox M8_Gps;
// Altitude - Latitude - Longitude - N Satellites
float gpsArray[N_FLOATS] = {0, 0, 0, 0};



float time=0;
int obs_dist_max[3]={50,50,50};
int obs[3]={};
int obs_count=0;
float distance[3]={};
const int LEFT_FRONT_MOTOR_1 = 23;
const int LEFT_FRONT_MOTOR_2 = 22;
const int LEFT_FRONT_PWM = 8;

const int RIGHT_FRONT_MOTOR_1 = 24;
const int RIGHT_FRONT_MOTOR_2 = 25;
const int RIGHT_FRONT_PWM = 10;

const int LEFT_BACK_MOTOR_1 = 27;
const int LEFT_BACK_MOTOR_2 = 26;
const int LEFT_BACK_PWM = 11;

const int RIGHT_BACK_MOTOR_1 = 5;
const int RIGHT_BACK_MOTOR_2 = 2;
const int RIGHT_BACK_PWM = 12;

const int FORWARD = 0;
const int Reverse = 1;

int leftDistance;
int rightDistance;
int previousDistanceStateSum = 0;
int currDistanceState = 0;
int count = 0;

const int trigger[3] = {35,33,31};
const int echo[3] ={36,34,32};

  
  


void setup() {
  Serial.begin(115200);
  Serial.begin(SERIAL_BAUD);
  Serial1.begin(GPS_BAUD);
  for(int i=0;i<3;i++){
  pinMode(echo[i],INPUT);
  pinMode(trigger[i],OUTPUT);
 }

  pinMode(LEFT_FRONT_MOTOR_1,OUTPUT);
  pinMode(LEFT_FRONT_MOTOR_2,OUTPUT);
  pinMode(LEFT_FRONT_PWM,OUTPUT);
  
  pinMode(RIGHT_FRONT_MOTOR_1,OUTPUT);
  pinMode(RIGHT_FRONT_MOTOR_2,OUTPUT);
  pinMode(RIGHT_FRONT_PWM,OUTPUT);
  
  pinMode(LEFT_BACK_MOTOR_1,OUTPUT);
  pinMode(LEFT_BACK_MOTOR_2,OUTPUT);
  pinMode(LEFT_BACK_PWM,OUTPUT);
  
  pinMode(RIGHT_BACK_MOTOR_1,OUTPUT);
  pinMode(RIGHT_BACK_MOTOR_2,OUTPUT);
  pinMode(RIGHT_BACK_PWM,OUTPUT);

  digitalWrite(LEFT_FRONT_MOTOR_2, HIGH);
  digitalWrite(LEFT_FRONT_MOTOR_1, LOW);
  digitalWrite(RIGHT_FRONT_MOTOR_1, HIGH);
  digitalWrite(RIGHT_FRONT_MOTOR_2, LOW);
  digitalWrite(LEFT_BACK_MOTOR_2, HIGH);
  digitalWrite(LEFT_BACK_MOTOR_1, LOW);
  digitalWrite(RIGHT_BACK_MOTOR_1, HIGH);
  digitalWrite(RIGHT_BACK_MOTOR_2, LOW);

  Wire.begin();
  compass_x_offset = -48.23;  //122.17;
  compass_y_offset = 284.69;  //230.08;
  compass_z_offset = 59.87;  //389.85;
  compass_x_gainError = 1.07;  //1.12;
  compass_y_gainError = 1.09;  //1.13;
  compass_z_gainError = 1.01;  //1.03;

  compass_init(2);
  Serial.println("Connecting to satelites");

}

void loop() {
  sensor();
  for(int i=0;i<3;i++) {
    if (distance[i]<=obs_dist_max[i]){
      ultra();
    }
    else{
      if (!Serial1.available())
    return;

  while (Serial1.available()) {
    char c = Serial1.read();
    if (M8_Gps.encode(c)) {
      gpsArray[0] = M8_Gps.altitude;
      gpsArray[1] = M8_Gps.latitude;
      gpsArray[2] = M8_Gps.longitude;
      gpsArray[3] = M8_Gps.sats_in_use;
    }
  }

  for (byte i = 0; i < N_FLOATS; i++) {
    Serial.print(gpsArray[i], 6); Serial.print(" ");
  }
  Serial.println();

  if (gpsArray[3] == 0) {
    halt();
    return; //stop robot
  }

  t = millis();

  float load;
  
    gpsArray[1] *= 10000;
    gpsArray[1] = int(gpsArray[1]) ;
    gpsArray[1] /= 10000;

    gpsArray[2] *= 10000;
    gpsArray[2] = int(gpsArray[2]) ;
    gpsArray[2] /= 10000;

    Serial.println(gpsArray[2],6);
    Serial.print("----->");
    Serial.println(gpsArray[1],6);
    delay(500);
  

  compass_scalled_reading();
  compass_heading();
  crntheading = abs(bearing - 360);
  destheading = readheading(gpsArray[2], gpsArray[1]);

  /*
    Serial.println(destheading);
    Serial.print("----->");
    Serial.println(crntheading);
    Serial.print("----------------->");
    Serial.println(readdist(gpsArray[2], gpsArray[1]),6);
    //delay(500);
  */

  disttodest = readdist(gpsArray[2], gpsArray[1]);

  //Serial.print("------------->");
  //Serial.println(disttodest, 6);


  if (disttodest > gpsTol) go();
  else halt();

  adjustheading(destheading - crntheading);

    }
  }

 //ultra();
  
}



void setMotorSpeed(int pin1, int pin2, int pwmPin, int pwmValue, int dir){
  if(dir == FORWARD){
    digitalWrite(pin1,LOW);
    digitalWrite(pin2,HIGH);
  }else if(dir == Reverse){
    digitalWrite(pin1,HIGH);
    digitalWrite(pin2,LOW);
  }

  pwmValue = pwmValue > 255 ? 255:(pwmValue < 0 ? 0:pwmValue);

  analogWrite(pwmPin,pwmValue);
}

void goForward(int pwmValue){
  //Robot moves forward. All 4 motors rotate at the same speed according to the pwmValue.
  setMotorSpeed(LEFT_FRONT_MOTOR_1, LEFT_FRONT_MOTOR_2, LEFT_FRONT_PWM, pwmValue, FORWARD);
  setMotorSpeed(RIGHT_FRONT_MOTOR_1, RIGHT_FRONT_MOTOR_2, RIGHT_FRONT_PWM, pwmValue, FORWARD);
  setMotorSpeed(LEFT_BACK_MOTOR_1, LEFT_BACK_MOTOR_2, LEFT_BACK_PWM, pwmValue, FORWARD);
  setMotorSpeed(RIGHT_BACK_MOTOR_1, RIGHT_BACK_MOTOR_2, RIGHT_BACK_PWM, pwmValue, FORWARD);
}

void reverse(int pwmValue){
  //Robot moves in reverse direction. All 4 motors rotate at the same speed according to the pwmValue.
  setMotorSpeed(LEFT_FRONT_MOTOR_1, LEFT_FRONT_MOTOR_2, LEFT_FRONT_PWM, pwmValue, Reverse);
  setMotorSpeed(RIGHT_FRONT_MOTOR_1, RIGHT_FRONT_MOTOR_2, RIGHT_FRONT_PWM, pwmValue, Reverse);
  setMotorSpeed(LEFT_BACK_MOTOR_1, LEFT_BACK_MOTOR_2, LEFT_BACK_PWM, pwmValue, Reverse);
  setMotorSpeed(RIGHT_BACK_MOTOR_1, RIGHT_BACK_MOTOR_2, RIGHT_BACK_PWM, pwmValue, Reverse);
}

void turnLeft(int leftPwmValue, int rightPwmValue){
  //Robot turns to left. Right side motors rotate forward. Left motors rotate forward if the leftPwmValue > 0. Otherwise they rotate reverse. 
  setMotorSpeed(RIGHT_FRONT_MOTOR_1, RIGHT_FRONT_MOTOR_2, RIGHT_FRONT_PWM, rightPwmValue, FORWARD);
  setMotorSpeed(RIGHT_BACK_MOTOR_1, RIGHT_BACK_MOTOR_2, RIGHT_BACK_PWM, rightPwmValue, FORWARD);
  if(leftPwmValue > 0){
    setMotorSpeed(LEFT_FRONT_MOTOR_1, LEFT_FRONT_MOTOR_2, LEFT_FRONT_PWM, leftPwmValue, FORWARD);
    setMotorSpeed(LEFT_BACK_MOTOR_1, LEFT_BACK_MOTOR_2, LEFT_BACK_PWM, leftPwmValue, FORWARD);
  }else{
    setMotorSpeed(LEFT_FRONT_MOTOR_1, LEFT_FRONT_MOTOR_2, LEFT_FRONT_PWM, -1*leftPwmValue, Reverse);
    setMotorSpeed(LEFT_BACK_MOTOR_1, LEFT_BACK_MOTOR_2, LEFT_BACK_PWM, -1*leftPwmValue, Reverse);
  }
}


void turnRight(int leftPwmValue, int rightPwmValue){
  //Robot turns to right. Left side motors rotate forward. Right motors rotate forward if the rightPwmValue > 0. Otherwise they rotate reverse. 
  setMotorSpeed(LEFT_FRONT_MOTOR_1, LEFT_FRONT_MOTOR_2, LEFT_FRONT_PWM, leftPwmValue, FORWARD);
  setMotorSpeed(LEFT_BACK_MOTOR_1, LEFT_BACK_MOTOR_2, LEFT_BACK_PWM, leftPwmValue, FORWARD);
  if(rightPwmValue > 0){
    setMotorSpeed(RIGHT_FRONT_MOTOR_1, RIGHT_FRONT_MOTOR_2, RIGHT_FRONT_PWM, rightPwmValue, FORWARD);
    setMotorSpeed(RIGHT_BACK_MOTOR_1, RIGHT_BACK_MOTOR_2, RIGHT_BACK_PWM, rightPwmValue, FORWARD);
  }else{
    setMotorSpeed(RIGHT_FRONT_MOTOR_1, RIGHT_FRONT_MOTOR_2, RIGHT_FRONT_PWM, -1*rightPwmValue, Reverse);
    setMotorSpeed(RIGHT_BACK_MOTOR_1, RIGHT_BACK_MOTOR_2, RIGHT_BACK_PWM, -1*rightPwmValue, Reverse);
  }
}

void breakRobot(){
  //stop the robot
  digitalWrite(LEFT_FRONT_MOTOR_1,HIGH);
  digitalWrite(LEFT_FRONT_MOTOR_2,HIGH);

  digitalWrite(LEFT_BACK_MOTOR_1,HIGH);
  digitalWrite(LEFT_BACK_MOTOR_2,HIGH);

  digitalWrite(RIGHT_FRONT_MOTOR_1,HIGH);
  digitalWrite(RIGHT_FRONT_MOTOR_2,HIGH);

  digitalWrite(RIGHT_BACK_MOTOR_1,HIGH);
  digitalWrite(RIGHT_BACK_MOTOR_2,HIGH);
}



void ultra()
{
 obs_count=0;
 sensor();
 
 for(int i=0;i<3;i++){
 if (distance[i]<=obs_dist_max[i]){
  obs[i]=1;
  obs_count+=1;
 }
 else{
  obs[i]=0;
 }
 
Serial.print(distance[i]);
Serial.print(" || ");
//delay(10);
 }

Serial.print(obs[0]); 
Serial.print(obs[1]); 
Serial.print(obs[2]); 
Serial.print(" || ");
Serial.println(obs_count);
if(obs_count>=1){
  if( distance[1]<=15 &&  distance[0]<=15 &&  distance[3]<=15){
    breakRobot();
    reverse(150);
    Serial.print("reverse ");
    
}
else {

//reverse(100);
  //brake(); 

if(obs[2]==0 && obs[0]==1){
breakRobot();

turnLeft(-200,200);
Serial.print("left ");
}
else if(obs[0]==0 && obs[2]==1){
breakRobot();

turnRight(200,-200);
Serial.print("right  ");
}
else if (obs[1]==1 or obs[0]==obs[1]==obs[2]==1) {
//
breakRobot();

turnLeft(-200,200);
//rotate_left(ride_speed);
Serial.print("brake  ");
}
}
}
else {
goForward(200);
Serial.print("forward  ");
}
}


void sensor(){
for(int i=0;i<3;i++){
 digitalWrite(trigger[i],HIGH);
 delayMicroseconds(10);
 digitalWrite(trigger[i],LOW);
 delayMicroseconds(2);
 time=pulseIn(echo[i],HIGH);
 distance[i]=time*340/20000; 
 Serial.print(distance[i]);
Serial.print(" || ");
}
Serial.println("");
}

void adjustheading(int fix) {
  if (fix > 180) fix = fix - 360;
  if (fix < -180) fix = fix + 360;

  //Serial.println(fix);

  fix *= 3;

  fix = int(constrain(fix, -255, 255));

  if (fix > 90)rotate(1);
  else if (fix < -90)rotate(-1);

  if (fix > 0) {
    //Serial.print("L,");
    analogWrite(RIGHT_FRONT_PWM, 200 - fix);
    analogWrite(LEFT_FRONT_PWM, 200);
    analogWrite(LEFT_BACK_PWM, 200);
    analogWrite(RIGHT_BACK_PWM, 200 - fix);
    //Serial.println(100 - int(fix));
  }
  else {
    //Serial.print("R,");
    analogWrite(RIGHT_FRONT_PWM, 200);
    analogWrite(LEFT_FRONT_PWM, 200 + fix);
    analogWrite(LEFT_BACK_PWM, 200 + fix);
    analogWrite(RIGHT_BACK_PWM, 200);
    //Serial.println(100 + int(fix));
  }
}

float readdist(float lon, float lat) {
  float dist = sqrt( pow((lat - destlat), 2) + pow((lon - destlon), 2));
  return dist;
}

void rotate(int dur) {
  if (dur < 0) {
    digitalWrite(LEFT_FRONT_MOTOR_2, LOW);
    digitalWrite(LEFT_FRONT_MOTOR_1, HIGH);
    digitalWrite(RIGHT_FRONT_MOTOR_1, HIGH);
    digitalWrite(RIGHT_FRONT_MOTOR_2, LOW);
    digitalWrite(LEFT_BACK_MOTOR_2, LOW);
    digitalWrite(LEFT_BACK_MOTOR_1, HIGH);
    digitalWrite(RIGHT_BACK_MOTOR_1, HIGH);
    digitalWrite(RIGHT_BACK_MOTOR_2, LOW);
  }

  else {
    digitalWrite(LEFT_FRONT_MOTOR_2, HIGH);
    digitalWrite(LEFT_FRONT_MOTOR_1, LOW);
    digitalWrite(RIGHT_FRONT_MOTOR_1, LOW);
    digitalWrite(RIGHT_FRONT_MOTOR_2, HIGH);
    digitalWrite(LEFT_BACK_MOTOR_2, HIGH);
    digitalWrite(LEFT_BACK_MOTOR_1, LOW);
    digitalWrite(RIGHT_BACK_MOTOR_1, LOW);
    digitalWrite(RIGHT_BACK_MOTOR_2, HIGH);
  }

  analogWrite(RIGHT_FRONT_PWM, 255);
  analogWrite(LEFT_FRONT_PWM, 255);
  analogWrite(LEFT_BACK_PWM, 255);
  analogWrite(RIGHT_BACK_PWM, 255);

  //delay(abs(dur));
}

int readheading(float lon, float lat) {
  float heading = atan(abs((lat - destlat) / (lon - destlon)));
  heading = (heading / 3.14159) * 180;

  if (lon < destlon && lat > destlat) heading += 90;
  if (lon > destlon && lat > destlat) heading += 180;
  if (lon > destlon && lat < destlat) heading += 270;

  return int(heading);
}

void go() {
  digitalWrite(LEFT_FRONT_MOTOR_2, HIGH);
  digitalWrite(LEFT_FRONT_MOTOR_1, LOW);
  digitalWrite(RIGHT_FRONT_MOTOR_1, HIGH);
  digitalWrite(RIGHT_FRONT_MOTOR_2, LOW);
  digitalWrite(LEFT_BACK_MOTOR_2, HIGH);
  digitalWrite(LEFT_BACK_MOTOR_1, LOW);
  digitalWrite(RIGHT_BACK_MOTOR_1, HIGH);
  digitalWrite(RIGHT_BACK_MOTOR_2, LOW);
}

void halt() {
  analogWrite(RIGHT_FRONT_PWM, 0);
  analogWrite(LEFT_FRONT_PWM, 0);
  analogWrite(LEFT_BACK_PWM, 0);
  analogWrite(RIGHT_BACK_PWM, 0);
  digitalWrite(LEFT_FRONT_MOTOR_2, HIGH);
  digitalWrite(LEFT_FRONT_MOTOR_1, HIGH);
  digitalWrite(RIGHT_FRONT_MOTOR_1, HIGH);
  digitalWrite(RIGHT_FRONT_MOTOR_2, HIGH);
  digitalWrite(LEFT_BACK_MOTOR_2, HIGH);
  digitalWrite(LEFT_BACK_MOTOR_1, HIGH);
  digitalWrite(RIGHT_BACK_MOTOR_1, HIGH);
  digitalWrite(RIGHT_BACK_MOTOR_2, HIGH);
}

float atan(float c) {
  float out;
  out = asin(c / (sqrt(1 + c * c)));
  return out;
}

float asin(float c) {

  float out;
  out = ((c + (c * c * c) / 6 + (3 * c * c * c * c * c) / 40 + (5 * c * c * c * c * c * c * c) / 112 +
          (35 * c * c * c * c * c * c * c * c * c) / 1152 + (c * c * c * c * c * c * c * c * c * c * c * 0.022) +
          (c * c * c * c * c * c * c * c * c * c * c * c * c * .0173) + (c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * .0139) +
          (c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * 0.0115) + (c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * 0.01)
         ));
  //asin
  if (c >= .96 && c < .97) {
    out = 1.287 + (3.82 * (c - .96));
  }

  if (c >= .97 && c < .98) {
    out = (1.325 + 4.5 * (c - .97)); // arcsin
  }

  if (c >= .98 && c < .99) {
    out = (1.37 + 6 * (c - .98));
  }

  if (c >= .99 && c <= 1) {
    out = (1.43 + 14 * (c - .99));
  }

  return out;
}
