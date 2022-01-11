/*
   ES 234 - TVI
   EE.L3.Pa template

   last revised 16 December 2021
   by Haedo Cho & J. Evan Smith
*/

#include <Adafruit_BNO08x.h>
#define BNO08X_RESET -1

struct euler_t {
  float pitch;
  float roll;
  float yaw;
} pry;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setup(void) {

  Serial.begin(115200);
  delay(500);

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(100);
      if (bno08x.begin_I2C()) {
        break;
      }
    }
  }
  Serial.println("BNO08x initialized");
  delay(500);

  setReports();
}

void setReports() {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* pry, bool degrees = false) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  pry->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  pry->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  pry->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));

  if (degrees) {
    pry->pitch *= RAD_TO_DEG;
    pry->roll *= RAD_TO_DEG;
    pry->yaw *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* pry, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, pry, degrees);
}

void loop() {

  if (bno08x.wasReset()) {
    Serial.println("Sensor was reset... ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  switch (sensorValue.sensorId) {
    case SH2_ARVR_STABILIZED_RV:
      quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &pry, true);
      break;
  }

  unsigned long curMillis = millis();
  Serial.print("Time:");
  Serial.print(curMillis);  
  Serial.print(",");
  
  Serial.print("Pitch:");
  Serial.print(pry.pitch);
  Serial.print(",");
  Serial.print("Yaw:");
  Serial.print(pry.yaw);
  Serial.print(",");
  Serial.print("Roll:");
  Serial.print(pry.roll);
  Serial.print(",");
  
  Serial.print("Accel_x: ");
  Serial.print(sensorValue.un.accelerometer.x);
  Serial.print(",");
  Serial.print("Accel_y: ");
  Serial.print(sensorValue.un.accelerometer.y);
  Serial.print(",");
  Serial.print("Accel_z: ");
  Serial.print(sensorValue.un.accelerometer.z);
  Serial.print(",");
  
  Serial.print("Gyro_x: ");
  Serial.print(sensorValue.un.gyroscope.x);
  Serial.print(",");
  Serial.print("Gyro_y: ");
  Serial.print(sensorValue.un.gyroscope.y);
  Serial.print(",");
  Serial.print("Gyro_z: ");
  Serial.print(sensorValue.un.gyroscope.z);
  Serial.print(",");
  
  Serial.print("Mag_x: ");
  Serial.print(sensorValue.un.magneticField.x);
  Serial.print(",");
  Serial.print("Magl_y: ");
  Serial.print(sensorValue.un.magneticField.y);
  Serial.print(",");
  Serial.print("Mag_z: ");
  Serial.println(sensorValue.un.magneticField.z);

}
