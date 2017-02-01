#include<Wire.h>
#include <Servo.h>

float filteredAngle = 0;
float gyroInt = 0;
float accAngle = 0;
int throttle = 100;
float dt = 0.01;
float Pbuffer = 0;
float lastPout = 0;

float bufferCircle[] = {0, 0, 0, 0, 0, 0, 0, 0};
int bufferPosition = 0;

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

Servo leftMotor;
Servo rightMotor;

#define analogInPin0 A0  // Analog input pin that the potentiometer is attached to
#define analogInPin1 A1  // Analog input pin that the potentiometer is attached to
#define analogInPin2 A2  // Analog input pin that the potentiometer is attached to


void setup(){
  // attatch servo objects
  leftMotor.attach(10);  
  rightMotor.attach(11);
  delay(10);

  
  //arm the Escs
  rightMotor.write(40);
  leftMotor.write(40);
  delay(1000);
  leftMotor.write(1);
  rightMotor.write(1);
  delay(2000);

  
  //set up i2c
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //serial begin
  Serial.begin(115200);
}


void loop(){
  int pidOut = 0;
  
  //I2c the data from the MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  /*
  //output the mpu6050 values
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.println(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  */

  //bring the data in line with real world values
  float AccY = (float)AcY/(float)16500;
  float AccZ = (float)AcZ/(float)16500;
  float gyroDpS = (GyX/131)*-1;

    
  accAngle = atan2(AccY, AccZ*-1) * 180/M_PI;    // get accelerometer view of data
  //float smoothed = average(accAngle); accAngle = smoothed;   // get smoothed acc values
  
  gyroInt += 2*(gyroDpS*dt);                     //gyro integrated view
  filteredAngle = 0.995 * (filteredAngle + 1.5*(gyroDpS *dt)) + (0.005 * accAngle); //filtered view (why 1.5? no idea just seemed needed to make it work)

  pidOut = PID(filteredAngle);                   //get pid output for comp filter input
  pidOut = constrain(pidOut, -79, 79);           //stop it going past the 180 servo library
  
  leftMotor.write(throttle + (int)pidOut);
  rightMotor.write(throttle - (int)pidOut); 
  delay(10);
}


int PID(float currentAngle){
  //set local variables (success with, P = 782, I = 611 , D = 152)
  int Pgain = 782, Igain = 611, Dgain = 152, setPoint = 0, windupLimit = 50;
  float Pout, Iout, Dout;
  
  Pgain = map(analogRead(analogInPin0), 0, 1023, 0, 2000);
  Igain = map(analogRead(analogInPin1), 0, 1023, 0, 2000);
  Dgain = map(analogRead(analogInPin2), 0, 1023, 0, 2000);

  //print the pid gain levels
  //Serial.print("P "); Serial.print(Pgain); Serial.print("\t I "); Serial.print(Igain); Serial.print("\t D "); Serial.println(Dgain); //Serial.println("");
  
  
  //calculate PID
  Pout = currentAngle - setPoint;
  Pbuffer += Pout*dt;
  Iout = Pbuffer;
  Dout = ((Pout - lastPout)/dt);
  lastPout = Pout;

  //anti windup code
  if(Pbuffer > windupLimit)         { Pbuffer = windupLimit;} 
  else if(Pbuffer < windupLimit*-1) { Pbuffer = windupLimit*-1;}
  else                              { Pbuffer = Pbuffer;}
  //Integral anti-overshoot code
  if(currentAngle >= -0.3 && currentAngle <= 0.3) { Pbuffer = 0; }

  float reducer = 0.001;
  Pout *= reducer;
  Iout *= reducer;
  Dout *= reducer;

  
  
  // calculate output given gains
  float PIDOutput = (Pout *Pgain) + (Iout *Igain) + (Dout *Dgain);
  int PIDOutput2 = (int)PIDOutput;

  //send information to processing
  String stringAngle = (String)filteredAngle;
  String stringPerror = (String)(Pout *Pgain);
  String stringIerror = (String)(Iout *Igain);
  String stringDerror = (String)(Dout *Dgain);
  Serial.println(stringAngle + "A" + stringPerror + "B" + stringIerror + "C" + stringDerror + "D");
  //Serial.println(filteredAngle);
  
  return PIDOutput2;
}


float average(float acc){
  float averageBuff = 0;
  int sizeofBuffer = sizeof(bufferCircle)/sizeof(float);

  //update position
  if(bufferPosition <= sizeofBuffer) { bufferPosition++; } 
  else{ bufferPosition = 0; }

  bufferCircle[bufferPosition] = acc;

  //sum the buffer
  for (int i = 0; i < sizeofBuffer; i++) {
    averageBuff += bufferCircle[i];
  }

  return (averageBuff/sizeofBuffer);
}


