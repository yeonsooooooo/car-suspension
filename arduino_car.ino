#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

// MPU6050 constants
const int MPU6050_addr = 0x68;   // I2C address of the MPU6050
const int MPU6050_xAccel_reg = 0x3B;  // Register address for X-axis accelerometer data
const int MPU6050_yAccel_reg = 0x3D;  // Register address for Y-axis accelerometer data
const int MPU6050_zAccel_reg = 0x3F;  // Register address for Z-axis accelerometer data
const int MPU6050_xGyro_reg = 0x43;   // Register address for X-axis gyroscope data
const int MPU6050_yGyro_reg = 0x45;   // Register address for Y-axis gyroscope data
const int MPU6050_zGyro_reg = 0x47;   // Register address for Z-axis gyroscope data

// PID constants
double kp = 1.0;    // Proportional gain
double ki = 0.0001;    // Integral gain
double kd = 0.00001;    // Derivative gain

// Servo objects
Servo servoX;
Servo servoY;

// Variables for MPU6050 data
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

// Variables for PID control
double setpointX = 0;    // Target angle for X-axis stabilization
double setpointY = 0;    // Target angle for Y-axis stabilization
double inputX;
double inputY;   // Current angle values from MPU6050
double outputX;
double outputY; // Output values from PID controller

// PID objects
PID pidX(&inputX, &outputX, &setpointX, kp, ki, kd, DIRECT);
PID pidY(&inputY, &outputY, &setpointY, kp, ki, kd, DIRECT);

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  // Initialize MPU6050
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);             // PWR_MGMT_1 register
  Wire.write(0);                // Set to 0 to activate the MPU-6050
  Wire.endTransmission(true);
  
  // Attach servo motors to pins
  servoX.attach(9);
  servoY.attach(10);
  
  // Set servo motor initial positions
  servoX.write(0);
  servoY.write(00);
  
  // Set PID parameters
  pidX.SetMode(AUTOMATIC);
  pidX.SetSampleTime(10);  // Update PID controller every 10 milliseconds
  pidY.SetMode(AUTOMATIC);
  pidY.SetSampleTime(10);  // Update PID controller every 10 milliseconds
}

void loop() {
  // Read MPU6050 data
  readMPU6050Data();
  
  // Convert raw accelerometer and gyroscope data to angles
  inputX = getAccAngle(accY, accZ);
  inputY = getAccAngle(accY, accZ);
  
  // Perform PID calculations
  pidX.Compute();
  pidY.Compute();
  
  // Update servo motor positions
  int servoXPosition = map(outputX, -45, 135, 0, 180);
  int servoYPosition = map(outputY, -45, 135, 180, 0);
  servoX.write(servoXPosition);
  servoY.write(servoYPosition);
  
  // Print values for debugging
  Serial.print("X: ");
  Serial.print(inputX);
  Serial.print(" | Y: ");
  Serial.print(inputY);
  Serial.print(" | X: ");    
  Serial.print(outputX);
  Serial.print(" | Y: ");
  Serial.print(outputY);    
  Serial.print(" | Servo X: ");
  Serial.print(servoXPosition);
  Serial.print(" | Servo Y: ");
  Serial.println(servoYPosition);
  
  delay(10);  // Delay for PID sample time
}

void readMPU6050Data() {
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(MPU6050_xAccel_reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr, 14, true);
  
  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

double getAccAngle(int16_t acc1, int16_t acc2) {
  double angle = atan2(acc1, acc2) * (180 / PI);
  return angle - 45;
}
