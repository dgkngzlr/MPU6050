#include<Wire.h>
#include<math.h>

const int MPU_adr = 0x68;
const int pi = 3.14159265;

long time_pre;
long AcX,AcY,AcZ,GyX,GyY,GyZ;
float gForceX,gForceY,gForceZ,rotX,rotY,rotZ;

float pitch_acc, roll_acc;
float pitch_gyro,roll_gyro;
float pitch_comp,roll_comp;

float delta_angleX,delta_angleY;

float errP,errR;
float errPt,errRt = 0;

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
  pitch_comp = pitch_acc;
  roll_comp = roll_acc;
  for(int i = 0; i<100;i++){
    recordAccelRegisters();
    accelDeg();
    errPt += pitch_acc;
    errRt += roll_acc;
    delay(10);
    }
  errP = errPt/100.0;
  errR = errRt/100.0;
  
  Serial.print("Pitch error :");
  Serial.print(errP);
  Serial.print(" Roll error :");
  Serial.println(errR);
  
}

void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  compDeg();
  accelDeg();
  gyroDeg();
  time_pre = millis();
  delay(10);
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

void accelDeg(){

  pitch_acc = atan2(-gForceX,gForceZ)*180/pi;
  roll_acc = atan2(gForceY,gForceZ)*180/pi;
  
  }

void gyroDeg(){
  
  delta_angleY = rotY * (millis() - time_pre)/1000;
  delta_angleX = rotX * (millis() - time_pre)/1000;
  
  pitch_gyro = pitch_gyro + delta_angleY;
  roll_gyro = roll_gyro + delta_angleX;
   
   
  }

void compDeg(){

  pitch_comp = 0.98 * (pitch_comp + delta_angleY ) + 0.02 * (pitch_acc);
  roll_comp = 0.98 * (roll_comp + delta_angleX ) + 0.02 * (roll_acc);

  Serial.print("Pitch :");
  Serial.print(pitch_comp-errP);
  Serial.print(",");
  Serial.print("Roll :");
  Serial.print(roll_comp-errR);
  Serial.println(",");
  
  }
