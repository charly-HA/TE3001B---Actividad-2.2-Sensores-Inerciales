#include <Wire.h>  // Incluye la biblioteca Wire para la comunicación I2C.

// Definiciones de constantes para la configuración del MPU6050.
#define MPU 0x68  // Dirección I2C del MPU6050.
#define RES_VEL 32768.0 / 250  // Conversión de la salida del giroscopio a grados por segundo, asumiendo sensibilidad de ±250°/s.

// Declaración de variables para almacenar los valores brutos del giroscopio.
int16_t Gyr_x, Gyr_y, Gyr_z;

// Variables para almacenar los valores convertidos en grados por segundo.
float Gyrx, Gyry, Gyrz;

// Variable para calcular el intervalo de tiempo para la integración del giroscopio.
float roll_gyr, pitch_gyr, yaw_gyr, t_ant, dt;

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

  // Cálculo de ángulos de orientación usando los datos del giroscopio (integración simple).
  dt = (micros() - t_ant) / 1000000.0;  // Calcula el intervalo de tiempo transcurrido.
  roll_gyr += dt * Gyrx;  // Integra la velocidad angular para obtener el ángulo.
  pitch_gyr += dt * Gyry;
  yaw_gyr += dt * Gyrz;
  t_ant = micros();  // Actualiza el tiempo anterior para el próximo cálculo.

  // Muestra los resultados por el puerto serial.
  Serial.print("Gyrx: "); Serial.print(Gyrx);
  Serial.print(" Gyry: "); Serial.print(Gyry);
  Serial.print(" Gyrz: "); Serial.println(Gyrz);
  Serial.print("Roll: "); Serial.print(roll_gyr);
  Serial.print(" Pitch: "); Serial.print(pitch_gyr);
  Serial.print(" Yaw: "); Serial.println(yaw_gyr);

  delay(2000);  // Espera un corto tiempo antes de la próxima iteración del bucle.
}

