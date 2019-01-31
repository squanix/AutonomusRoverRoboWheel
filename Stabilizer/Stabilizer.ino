#include <Servo.h>
#include <Wire.h>
#include "kalman.h"
Kalman kalmanX;
Kalman kalmanY;

// Example Editing

uint8_t IMUAddress = 0x68;//Jenis unsigned integer panjang 8 bit

int16_t accX;//Integer untuk 16 bit dalam sebuah program . int16_t adalah 16 bit integer
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

int moveX;
int mapX;
int correctionX;

double accXangle;
double accYangle;
double gyroXangle = 9;
double gyroYangle = 180;
double compAngleX = 90;
double compAngleY = 90;
double kalAngleX;
double kalAngleY;
uint32_t timer;

Servo xservo;

void setup() {
  Serial.begin(57600);//Menetapkan data rate dalam bit per detik untuk transmisi data serial.
  xservo.attach(11);//Menetapkan pin 11 pada arduino untuk output servo, bisa diganti
  Wire.begin();//Mulai menggunakan Wire dalam mode, di mana akan merespon pada "alamat" ketika chip I2C lain melakukan komunikasi.
  i2cWrite(0x6B, 0x00);//memulai operasi I2C menulis tanpa data yang dikirim.  
  if (i2cRead(0x75, 1)[0] != 0x68)//memulai operasi baca dari posisi saat ini dari masukan daftar pointer. Byte akan disimpan di buffer internal dan akan memiliki batasan ukuran 32 byte. Data dapat dibaca dari buffer menggunakan I2c.receive ().
  {
    Serial.print(F("MPU-6050 with address 0x"));
    Serial.print(IMUAddress, HEX);
    Serial.println(F(" is not connected"));
    while (1);
  }
  kalmanX.setAngle(90);
  kalmanY.setAngle(90);
  timer = micros();//micro seccond, dimana 1000 sama dengan 1 detik.
}

void loop() {

  uint8_t* data = i2cRead(0x3B, 14);
  accX = ((data[0] << 8) | data[1]);// Bitshift Operator << Operator ini menyebabkan bit dalam operan kiri akan bergeser kiri atau kanan dengan jumlah posisi yang ditentukan oleh operan kanan.
  accY = ((data[2] << 8) | data[3]);// Ouput semua berupa integer, bilangan bulat, Bilangan Bulat 16 bit
  accZ = ((data[4] << 8) | data[5]);
  tempRaw = ((data[6] << 8) | data[7]);
  gyroX = ((data[8] << 8) | data[9]);
  gyroY = ((data[10] << 8) | data[11]);
  gyroZ = ((data[12] << 8) | data[13]);

  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG; // arc tangent of accX/accZ ditambah dengan PHI
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  double gyroXrate = (double)gyroX / 130.0; //bergerak ke x untuk positif
  double gyroYrate = -((double)gyroY / 130.0); // bergerak ke Y untuk negatif
  gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // kalibrasi dimana gyroXrate akan dikali dengan mili second waktu, untuk pergerakan dan presisi posisi

  gyroXangle += kalmanX.getRate() * ((double)(micros() - timer) / 1000000);
  compAngleX = (0.93 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.07* accXangle);

  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000);
  timer = micros();
  mapX = map(kalAngleX, 0, 200, 0, 179);

  correctionX = 27; 

  moveX = 270 - (kalAngleX) + correctionX;

  Serial.print(" gyroX Pos: ");Serial.print(gyroX);Serial.print("\t");
  Serial.print(" gyroY Pos: ");Serial.print(gyroY);Serial.print("\t");
  Serial.print(" gyroZ Pos: ");Serial.print(gyroZ);Serial.print("\t");
  Serial.print(" accX Pos: ");Serial.print(accX);Serial.print("\t");
  Serial.print(" accY Pos: ");Serial.print(accY);Serial.print("\t");
  Serial.print(" accZ Pos: ");Serial.print(accZ);Serial.print("\t");
  Serial.print(" X Pos: ");Serial.print(moveX); Serial.print("\t");
  Serial.print("\n");

  xservo.write(moveX);//Berputarnya Servo sesuai dengan besarnya moveX
  delay(15);// jeda 0,015 detik

  delay(1);//jeda 0,001 detk untuk mengulang program kembali dalam vod loop
}


void i2cWrite(uint8_t registerAddress, uint8_t data) 
{
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();                      
}

uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) 
{
  uint8_t data[nbytes];
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); 
  Wire.requestFrom(IMUAddress, nbytes); 
  for (uint8_t i = 0; i < nbytes; i++)
    data[i] = Wire.read();
  return data;
}
