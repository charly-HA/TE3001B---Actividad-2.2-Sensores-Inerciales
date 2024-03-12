#include <Wire.h>
#include <math.h>

#define MPU 0x68  // Dirección I2C del MPU6050
#define RES_ACC 32768.0 / 2  // Factor de conversión de la aceleración: 16 bits mapeados a 2g

int16_t Acc_x, Acc_y, Acc_z;
float Accx, Accy, Accz;
float roll_acc, pitch_acc;

void setup() {
  Serial.begin(115200);  // Inicializamos la conexión serial
  Wire.begin();  // Inicializamos el bus I2C
  
  // Configuramos el MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // Dirección del registro de power management
  Wire.write(0x00);  // Valor para despertar el sensor
  Wire.endTransmission(true);
}

void loop() {
  // Leer los valores del acelerómetro
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // Dirección del primer registro del acelerómetro
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);  // Solicitamos los 6 bytes del acelerómetro
  
  Acc_x = Wire.read() << 8 | Wire.read();  // Combinamos los bytes para cada eje
  Acc_y = Wire.read() << 8 | Wire.read();
  Acc_z = Wire.read() << 8 | Wire.read();
  
  // Convertimos los valores a unidades estándar (g)
  Accx = Acc_x / RES_ACC;
  Accy = Acc_y / RES_ACC;
  Accz = Acc_z / RES_ACC;

  // Cálculo de ángulos usando aceleración
  roll_acc = atan2(Accy, Accz) * 180 / M_PI;
  pitch_acc = atan2(-Accx, sqrt(Accy * Accy + Accz * Accz)) * 180 / M_PI;

  // Impresión de valores
  Serial.print("Roll: "); Serial.print(roll_acc);
  Serial.print(" Pitch: "); Serial.print(pitch_acc);
  Serial.print(" Accx: "); Serial.print(Accx);
  Serial.print(" Accy: "); Serial.print(Accy);
  Serial.print(" Accz: "); Serial.println(Accz);

  delay(2000);  // Pequeña pausa
}
