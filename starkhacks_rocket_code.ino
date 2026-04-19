#include <Arduino_LSM6DSO.h> // Modulino Movement
#include <MadgwickAHRS.h>    // Sensor Fusion
#include <PID_v1.h>          // PID Control

/* --- Hardware Mapping --- */
// 4 Motors: 0 (Front/+X), 1 (Back/-X), 2 (Right/+Y), 3 (Left/-Y)
const int motorPins[4] = {12, 13, 14, 27}; 
const int PWM_FREQ = 5000;
const int PWM_RES  = 8; 

/* --- Control Variables --- */
unsigned long lastUpdate = 0;
const float sampleFreq = 100.0; // 100Hz
Madgwick filter;

// PID Tuning (Proportional, Integral, Derivative)
// Start LOW and increase Kp gradually.
double Kp = 3.5, Ki = 0.1, Kd = 0.5;

double setpointX = 0, inputX, outputX;
double setpointY = 0, inputY, outputY;

PID pidX(&inputX, &outputX, &setpointX, Kp, Ki, Kd, DIRECT);
PID pidY(&inputY, &outputY, &setpointY, Kp, Ki, Kd, DIRECT);

/* --- Initialization --- */
void setup() {
  Serial.begin(115200);
  
  if (!IMU.begin()) {
    Serial.println("LSM6DSO not found!");
    while (1);
  }

  filter.begin(sampleFreq);

  // Setup PWM Channels
  for (int i = 0; i < 4; i++) {
    ledcAttach(motorPins[i], PWM_FREQ, PWM_RES);
  }

  // Configure PID
  pidX.SetMode(AUTOMATIC);
  pidX.SetOutputLimits(-255, 255); // Full range for differential thrust
  pidY.SetMode(AUTOMATIC);
  pidY.SetOutputLimits(-255, 255);

  Serial.println("RCS System Initialized. Waiting for stable orientation...");
  delay(2000); 
}

/* --- Main Loop --- */
void loop() {
  if (micros() - lastUpdate >= (1000000 / sampleFreq)) {
    lastUpdate = micros();

    float ax, ay, az, gx, gy, gz;

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);

      // 1. Update Sensor Fusion
      // Note: Check IMU orientation. Standard is X-forward, Y-left, Z-up.
      filter.updateIMU(gx, gy, gz, ax, ay, az);

      // 2. Extract Quaternion components
      float qw = filter.getQuatW();
      float qx = filter.getQuatX();
      float qy = filter.getQuatY();
      float qz = filter.getQuatZ();

      // 3. Calculate Tilt Error (Direction Cosine Matrix components)
      // These represent the projection of the global 'Up' onto rocket's X and Y
      inputX = 2.0f * (qx * qz - qw * qy);
      inputY = 2.0f * (qw * qx + qy * qz);

      // 4. Compute PID
      pidX.Compute();
      pidY.Compute();

      // 5. Motor Mixing Logic (RCS Pushing Style)
      // Motor 0 (+X) fires if error is positive, Motor 1 (-X) if negative.
      int m0 = (outputX > 0) ? abs(outputX) : 0;
      int m1 = (outputX < 0) ? abs(outputX) : 0;
      int m2 = (outputY > 0) ? abs(outputY) : 0;
      int m3 = (outputY < 0) ? abs(outputY) : 0;

      // 6. Write to Motors
      ledcWrite(motorPins[0], m0);
      ledcWrite(motorPins[1], m1);
      ledcWrite(motorPins[2], m2);
      ledcWrite(motorPins[3], m3);

      // Debugging
      // Serial.printf("X_Err: %.2f | Y_Err: %.2f | OutX: %.1f\n", inputX, inputY, outputX);
    }
  }
}
