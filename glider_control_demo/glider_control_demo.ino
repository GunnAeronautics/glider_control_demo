#include<Quaternion.h>
#include<SPI.h>
#include<BMI160Gen.h>
#include<Servo.h>
#define GRANGE 2000
#define ARANGE 16
#define samplerate 20
const int imucs = 9;

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

int axraw;
int ayraw;
int azraw;
int gxraw;
int gyraw;
int gzraw;

Quaternion accl;
Quaternion total;
Quaternion input;
Quaternion grav;
Quaternion ground;
Servo servs[2];

void setup() {
  //regular setup
  Serial.begin(9600);

  SPI.setSCK(27);
  SPI.begin();

  accl = Quaternion(0,0,0,0);
  total = Quaternion(0,0,0,0);
  input = Quaternion(0,0,0,0);
  grav = Quaternion(0,0,0,0);

  delay(1000);
  BMI160.begin(imucs, -1);
  initVars();
  BMI160.autoCalibrateGyroOffset();
  BMI160.setAccelerometerRange(ARANGE);
  BMI160.setGyroRange(GRANGE);
  delay(1000/samplerate);
  initVars();

  for(int i = 0; i < 2; i++){
    servs[i].attach(35+i);
    servs[i].write(90);
  }

}

void loop() {
  //pretty normal loop
  BMI160.readGyro(gxraw, gyraw, gzraw);
  input.gyro(convertRawGyro(gxraw), convertRawGyro(gyraw), convertRawGyro(gzraw));
  input.normalize();
  /*
  input.mult(total);
  total = input.copy();
  total.normalize();
  */
  //what so it isn't backwards now?? i give up
  total.mult(input);
  total.normalize();

  ground = Quaternion(0, 0, 1, 0);
  ground.rot(total);

  int poss[2] = {0, 0};
  //*
  if(abs(ground.j) > 0.2){
    poss[0] += 20*sgn(ground.j);
    poss[1] += 20*sgn(ground.j);
  }
  //*/
  //*
  if(abs(ground.i) > 0.2){
    poss[0] += 20*sgn(ground.i);
    poss[1] -= 20*sgn(ground.i);
  }
  //*/
  servs[0].write(90 + poss[0]);
  servs[1].write(90 - poss[1]);

  Quaternion writ = ground;
  Serial.print(writ.w, 4);
  Serial.print('\t');
  Serial.print(writ.i, 4);
  Serial.print('\t');
  Serial.print(writ.j, 4);
  Serial.print('\t');
  Serial.print(writ.k, 4);
  Serial.print('\n');
  delay(1000/samplerate);

}

void initVars(){
  //initializes the vars
  accl = Quaternion(0, 0, 0, 0);
  total = Quaternion(1, 0, 0, 0);
  input = Quaternion(1, 0, 0, 0);
  grav = Quaternion(0, 0, 0, 0);
  ground = Quaternion(0, 0, 0, 1);

  //get gravity vector
  BMI160.readAccelerometer(axraw, ayraw, azraw);
  grav = Quaternion(0, convertRawAccl(axraw),
                       convertRawAccl(ayraw),
                       convertRawAccl(azraw));
  //find quaternion needed to transform "normal" grav vector to current grav vec
  total.fromAngleVec(atan2(grav.j,sqrt(grav.i*grav.i + grav.k*grav.k)) - HALF_PI,
                      atan2(grav.k,grav.i) - HALF_PI, 0);
  
}

void serialEvent(){
  //recalc orientation if sent serial
  initVars();
  while(Serial.available()){
    Serial.read();
  }
}
// conversion factors
float convertRawAccl(int raw){
  return (ARANGE * (float)raw / 32768.0);
}

float convertRawGyro(int raw){
  return radians(GRANGE * (float)raw / (32768.0 * samplerate));
}
