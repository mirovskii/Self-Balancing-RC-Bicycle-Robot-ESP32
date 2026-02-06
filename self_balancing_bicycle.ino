#include "Wire.h"
#include "I2Cdev.h"
#include "math.h"
#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Arduino.h>

Servo servo;
Adafruit_MPU6050 mpu;

#define Kp  50
#define Kd  0.005
#define Ki  20
#define sampleTime  0.002
#define targetAngle 0

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;


void setup() {
  // Start serial communication
  Serial.begin(115200);
  
  // Start I2C communication
  Wire.begin(21, 22);

  // Attach the servo motor to pin 4
  servo.attach(4);

  // Initialize the servo position to 0 degrees
  servo.write(0);

  //Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 başlatılamadı! Bağlantıları kontrol edin.");
    while (1);
  }
  Serial.println("MPU6050 başarıyla başlatıldı!");

  //Set sensor data rate and range
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  delay(100);

}

void loop() {

  // Create sensor event objects for accelerometer, gyroscope, and temperature
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // read acceleration and gyroscope values
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  gyroX = g.gyro.x;

  // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);                           // Map the gyroscope data to the appropriate range
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.6666667*(prevAngle + gyroAngle) + 0.33333333*(accAngle);  // Combine the accelerometer and gyroscope angles using a complementary filter
  
  //calculate error and add error to errorsum
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);                                 // Limit the integral term to prevent windup

  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)/sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
 
  // Update the previous angle for the next iteration
  prevAngle = currentAngle;

  // Map the PID output to the servo range and write to the servo
  servo.write(map(motorPower, -900, 900, 0,180));
  Serial.println(map(motorPower, -900, 900, 0,180));


}
