#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

Servo myServo1;  // Roll servo
Servo myServo2;  // Pitch servo
Servo myServo3;  // Yaw servo

MPU6050 mpu;  // Create an object for the MPU6050 sensor

float roll = 0;  // Initialize roll value
float pitch = 0;  // Initialize pitch value
float yaw = 0;  // Initialize yaw value
unsigned long lastTime = 0;  // To keep track of time for yaw calculation

void setup() {
  Serial.begin(9600);
  
  myServo1.attach(9);  // Roll servo
  myServo2.attach(10);  // Pitch servo
  myServo3.attach(11);  // Yaw servo
  
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  
  lastTime = millis();
}

void loop() {
  readAndPrintSensorData();
  
  // Read MPU6050 angles
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // Get accelerometer and gyroscope values

  float accel_x = ax / 16384.0;  // Convert raw accelerometer data to Gs
  float accel_y = ay / 16384.0;
  float accel_z = az / 16384.0;

  float gyro_x = gx / 131.0;  // Convert raw gyroscope data to degrees per second
  float gyro_y = gy / 131.0;
  float gyro_z = gz / 131.0;

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // Time difference in seconds
  yaw += gyro_z ;  // Integrate the gyro_z to get yaw
  lastTime = currentTime;  // Update lastTime for the next calculation

  roll = atan2(accel_y, accel_z) * 180 / PI;
  pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180 / PI;

  Serial.print("Calculated angles: Roll: ");
  Serial.print(roll);
  Serial.print(", Pitch: ");
  Serial.print(pitch);
  Serial.print(", Yaw: ");
  Serial.println(yaw);

  // Assign angles to servos
  moveServoWithBurst(myServo1, roll, 50);  // Assuming 50ms duration for roll
  moveServoWithBurst(myServo2, pitch, 50);  // Assuming 50ms duration for pitch
  moveServoWithBurst(myServo3, yaw, 50);     // Assuming 50ms duration for yaw
}

void moveServoWithBurst(Servo &servo, float angle, int duration) {
  if (abs(angle) > 30) {
    if (angle > 30) {
      moveServoForDuration(servo, 75, 150);  // 75 degrees for 25ms
    } else {
      moveServoForDuration(servo, 105, 150);  // 105 degrees for 25ms
    }
  } else {
    moveServoForDuration(servo, 90, duration);
  }
}

void moveServoForDuration(Servo &servo, int angle, int duration) {
  servo.write(angle);          // Move to specified angle
  Serial.print("Moving servo to angle: ");
  Serial.println(angle);
  delay(duration);             // Hold the position for the specified duration
  servo.write(90);             // Optionally move to 90 degrees (neutral position)
  Serial.println("Stopped at angle 90.");
}

void readAndPrintSensorData() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // Get accelerometer and gyroscope values

  float accel_x = ax / 16384.0;  // Convert raw accelerometer data to Gs
  float accel_y = ay / 16384.0;
  float accel_z = az / 16384.0;

  float gyro_x = gx / 131.0;  // Convert raw gyroscope data to degrees per second
  float gyro_y = gy / 131.0;
  float gyro_z = gz / 131.0;

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // Time difference in seconds
  yaw += gyro_z * dt;  // Integrate the gyro_z to get yaw
  lastTime = currentTime;  // Update lastTime for the next calculation

  roll = atan2(accel_y, accel_z) * 180 / PI;
  pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180 / PI;

  Serial.print("Raw sensor data: Ax: ");
  Serial.print(ax);
  Serial.print(", Ay: ");
  Serial.print(ay);
  Serial.print(", Az: ");
  Serial.print(az);
  Serial.print(", Gx: ");
  Serial.print(gx);
  Serial.print(", Gy: ");
  Serial.print(gy);
  Serial.print(", Gz: ");
  Serial.println(gz);

  Serial.print("Calculated angles: Roll: ");
  Serial.print(roll);
  Serial.print(", Pitch: ");
  Serial.print(pitch);
  Serial.print(", Yaw: ");
  Serial.println(yaw);
}
