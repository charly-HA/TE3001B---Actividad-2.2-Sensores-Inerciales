#include <Wire.h>  // Incluye la biblioteca Wire para la comunicación I2C.
#include <math.h>  // Incluye la biblioteca matemática para funciones como atan2 y sqrt.

// Definiciones de constantes para la configuración del MPU6050.
#define MPU 0x68  // Dirección I2C del MPU6050.
#define RES_ACC 32768.0 / 2  // Conversión de la salida del acelerómetro a g, asumiendo sensibilidad de ±2g.
#define RES_VEL 32768.0 / 250  // Conversión de la salida del giroscopio a grados por segundo, asumiendo sensibilidad de ±250°/s.

// Declaración de variables para almacenar los valores brutos del acelerómetro y el giroscopio.
int16_t Acc_x, Acc_y, Acc_z, Gyr_x, Gyr_y, Gyr_z;

// Variables para almacenar los valores convertidos (en g y grados por segundo).
float Accx, Accy, Accz, Gyrx, Gyry, Gyrz;

// Variables para calcular los ángulos a partir de la aceleración y la velocidad angular.
float roll_acc, pitch_acc, roll_gyr, pitch_gyr, roll, pitch, t_ant, dt;

void setup() {
  Serial.begin(115200);  // Inicia la comunicación serial a 115200 baudios.
  Wire.begin();  // Inicia la comunicación I2C.

  // Configura el MPU6050 para que comience a medir.
  Wire.beginTransmission(MPU);  // Inicia la transmisión al dispositivo MPU6050.
  Wire.write(0x6B);  // Selecciona el registro de gestión de energía del MPU6050.
  Wire.write(0x00);  // Establece el bit de despertar del sensor.
  Wire.endTransmission(true);  // Finaliza la transmisión y libera el bus.
  
  t_ant = micros();  // Inicializa el contador de tiempo para el cálculo del intervalo (dt).
}

void loop() {
  // Lectura de los valores del acelerómetro.
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // Dirección del primer registro de datos del acelerómetro.
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);  // Solicita 6 bytes desde el MPU (2 por cada eje del acelerómetro).

  // Combina los bytes para obtener los valores de los ejes X, Y y Z del acelerómetro.
  Acc_x = Wire.read() << 8 | Wire.read();  
  Acc_y = Wire.read() << 8 | Wire.read();
  Acc_z = Wire.read() << 8 | Wire.read();

  // Convierte los valores brutos a g.
  Accx = Acc_x / RES_ACC;
  Accy = Acc_y / RES_ACC;
  Accz = Acc_z / RES_ACC;

  // Cálculo de ángulos usando aceleración.
  roll_acc = atan2(Accy, Accz) * 180 / M_PI;
  pitch_acc = atan2(-Accx, sqrt(Accy * Accy + Accz * Accz)) * 180 / M_PI;

  // Lectura de los valores del giroscopio.
  Wire.beginTransmission(MPU);
  Wire.write(0x43);  // Dirección del primer registro de datos del giroscopio.
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);  // Solicita 6 bytes desde el MPU (2 por cada eje del giroscopio).

  // Combina los bytes para obtener los valores de los ejes X, Y y Z del giroscopio.
  Gyr_x = Wire.read() << 8 | Wire.read();  
  Gyr_y = Wire.read() << 8 | Wire.read();
  Gyr_z = Wire.read() << 8 | Wire.read();

  // Convierte los valores brutos a grados por segundo.
  Gyrx = Gyr_x / RES_VEL;
  Gyry = Gyr_y / RES_VEL;
  Gyrz = Gyr_z / RES_VEL;

  // Cálculo de ángulos de orientación usando los datos del giroscopio.
  dt = (micros() - t_ant) / 1000000.0;  // Calcula el intervalo de tiempo transcurrido.
  roll_gyr += dt * Gyrx;  // Integra la velocidad angular para obtener el ángulo.
  pitch_gyr += dt * Gyry;
  t_ant = micros();  // Actualiza el tiempo anterior para el próximo cálculo.

  // Filtro complementario para fusionar los ángulos del acelerómetro y del giroscopio.
  roll = 0.95 * roll_gyr + 0.05 * roll_acc;
  pitch = 0.95 * pitch_gyr + 0.05 * pitch_acc;

  // Muestra los resultados por el puerto serial.
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Accx: "); Serial.print(Accx);
  Serial.print(" Accy: "); Serial.print(Accy);
  Serial.print(" Accz: "); Serial.print(Accz);
  Serial.print(" Gyrx: "); Serial.print(Gyrx);
  Serial.print(" Gyry: "); Serial.print(Gyry);
  Serial.print(" Gyrz: "); Serial.println(Gyrz);

  delay(2000);  // Espera un corto tiempo antes de la próxima iteración del bucle.
}
