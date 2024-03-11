#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Variables para calibración
float accX_offset = 0;
float accY_offset = 0;
float accZ_offset = 0;
float gyroX_offset = 0;
float gyroY_offset = 0;
float gyroZ_offset = 0;

// Tiempo
unsigned long lastTime = 0;
float deltaTime = 0;

// Ángulos calculados por el filtro complementario
float roll = 0;
float pitch = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);

  calibrateSensors();
  lastTime = millis(); // Inicializar la última vez que se tomó una lectura
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Aplicar los offsets de calibración
  a.acceleration.x -= accX_offset;
  a.acceleration.y -= accY_offset;
  a.acceleration.z -= accZ_offset;
  g.gyro.x -= gyroX_offset;
  g.gyro.y -= gyroY_offset;
  g.gyro.z -= gyroZ_offset;

  // Calcular deltaTime
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0; // Convertir a segundos
  lastTime = currentTime;

  // Estimación de ángulos usando giroscopio
  float gyroRoll = roll + g.gyro.x * deltaTime; // Integra la tasa de giro en el tiempo
  float gyroPitch = pitch + g.gyro.y * deltaTime; // Integra la tasa de giro en el tiempo

  // Estimación de ángulos usando acelerómetro
  float accRoll = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
  float accPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;

  // Filtro complementario
  roll = gyroRoll * 0.95 + accRoll * 0.05;
  pitch = gyroPitch * 0.95 + accPitch * 0.05;

  // Imprimir los valores de aceleración, giroscopio, roll y pitch
  Serial.print("AccX: "); Serial.print(a.acceleration.x);
  Serial.print(" AccY: "); Serial.print(a.acceleration.y);
  Serial.print(" AccZ: "); Serial.println(a.acceleration.z);
  Serial.print("GyroX: "); Serial.print(g.gyro.x);
  Serial.print(" GyroY: "); Serial.print(g.gyro.y);
  Serial.print(" GyroZ: "); Serial.println(g.gyro.z);
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.println(pitch);

  delay(100);
}

void calibrateSensors() {
  const int numReadings = 1000;

  float accX_sum = 0;
  float accY_sum = 0;
  float accZ_sum = 0;
  float gyroX_sum = 0;
  float gyroY_sum = 0;
  float gyroZ_sum = 0;

  Serial.println("Calibrando sensores, por favor espere...");

  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accX_sum += a.acceleration.x;
    accY_sum += a.acceleration.y;
    accZ_sum += a.acceleration.z;
    gyroX_sum += g.gyro.x;
    gyroY_sum += g.gyro.y;
    gyroZ_sum += g.gyro.z;

    delay(10); // Pequeña pausa entre muestras
  }
  // Calcular promedios y establecer los offsets
  accX_offset = accX_sum / numReadings;
  accY_offset = accY_sum / numReadings;
  accZ_offset = accZ_sum / numReadings - 9.81; // Ajuste para la gravedad
  gyroX_offset = gyroX_sum / numReadings;
  gyroY_offset = gyroY_sum / numReadings;
  gyroZ_offset = gyroZ_sum / numReadings;

  Serial.println("Calibración completa");
}
