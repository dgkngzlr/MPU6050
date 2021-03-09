#include <Kalman.h>
#include<Wire.h>
#include<math.h>

Kalman kalmanX;
Kalman kalmanY;

const int MPU_adr = 0x68; //MPU6050 nin slave adresi.
long AcX,AcY,AcZ,GyX,GyY,GyZ;
float gForceX,gForceY,gForceZ,rotX,rotY,rotZ;

float pitch_acc, roll_acc;
float pitch_gyro,roll_gyro;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  configMPU();
  recordAccelRegisters();
  recordGyroRegisters();
  accelDeg();
  gyroDeg();
  pitch_gyro = pitch_acc;
  roll_gyro = roll_acc;
  kalmanX.setAngle(pitch_acc); // Set starting angle
  kalmanY.setAngle(roll_acc);

}

void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  accelDeg();
  gyroDeg();
}

void configMPU(){
  Wire.beginTransmission(MPU_adr);
  Wire.write(0x6B); // Power Man. regisera ulastık.Sleep moddan cıkarmak icin.
  Wire.write(0b00000000); //Sleep moddan cıkardık
  Wire.endTransmission();
  Wire.beginTransmission(MPU_adr);
  Wire.write(0x1C); // Accelerometer configs AFS_SEL = 0 -> +-2g, AFS_SEL = 1 -> +-4g, AFS_SEL = 2 -> +-8g...vb
  Wire.write(0b00000000); // (0000 0000)+-2g, (0000 1000) +-4g, (0001 0000) +-8g, (0001 1000)+-16g
  Wire.endTransmission();
  Wire.beginTransmission(MPU_adr);
  Wire.write(0x1B); // Gyro config register FS_SEL
  Wire.write(0b00000000); // +-250 deg/sec ayarlandı. (0000 0000)+-250, (0000 1000) +-500, (0001 0000) +-1000, (0001 1000)+-2000
  Wire.endTransmission();
  
  
  }

void recordAccelRegisters(){
  Wire.beginTransmission(MPU_adr);
  Wire.write(0x3B);//Starting register for accel reading 3B-40
  Wire.endTransmission();
  Wire.requestFrom(MPU_adr,6); //6 bytlik veri istendi
  while(Wire.available()<6);

  AcX=Wire.read()<<8|Wire.read();   
  AcY=Wire.read()<<8|Wire.read(); 
  AcZ=Wire.read()<<8|Wire.read();


   gForceX = AcX / 16384.0; // Datasheete bak. Anlamlı verilere dondu.
   gForceY = AcY / 16384.0;
   gForceZ = AcZ / 16384.0;
  
  
  }
void recordGyroRegisters(){

  Wire.beginTransmission(MPU_adr);
  Wire.write(0x43);//Starting register for gyro reading 43-48
  Wire.endTransmission();
  Wire.requestFrom(MPU_adr,6); //6 bytlik veri istendi
  while(Wire.available()<6);

  GyX=Wire.read()<<8|Wire.read();   
  GyY=Wire.read()<<8|Wire.read(); 
  GyZ=Wire.read()<<8|Wire.read();

  rotX = GyX / 131.0;
  rotY = GyY / 131.0;
  rotZ = GyZ / 131.0;
  
  }
void printValues(){
  Serial.print("Accel(g force)");
  Serial.print(" X:");
  Serial.print(gForceX);
  Serial.print(" Y:");
  Serial.print(gForceY);
  Serial.print(" Z:");
  Serial.println(gForceZ);
  Serial.print("Gyro(deg/s)");
  Serial.print(" X:");
  Serial.print(rotX);
  Serial.print(" Y:");
  Serial.print(rotY);
  Serial.print(" Z:");
  Serial.println(rotZ);
  }
void accelDeg(){
  /*float R = sqrt(gForceX*gForceX+gForceY*gForceY+gForceZ*gForceZ);
  X_deg_acc = (180*acos(gForceX/sqrt(gForceX*gForceX+gForceZ*gForceZ))/3.14)-90;
  Y_deg_acc = (180*acos(gForceY/sqrt(gForceY*gForceY+gForceZ*gForceZ))/3.14)-90;*/

  
  pitch_acc = atan2(-gForceX,gForceZ)*180/3.141591;
  roll_acc = atan2(gForceY,gForceZ)*180/3.141591;
  
  }
void gyroDeg(){
  unsigned long pre_time = millis();
  delay(10);
  float delta_angle = rotY * (millis() - pre_time)/1000;
  pitch_gyro = pitch_gyro + delta_angle*2;
  
  /*Serial.println(pitch_gyro);
  Serial.print(",");
  Serial.println(pitch_acc);
  Serial.print(",");*/
  Serial.println(kalmanX.getAngle(pitch_acc,pitch_gyro,0.001));  

   
  }
