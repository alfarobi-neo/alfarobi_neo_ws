//program in C++ for PID control of head-tilt and head-pan servos based on the error_x and error_y values:

#include <Servo.h>

// Define PID gains
float Kp_x = 0.1;
float Ki_x = 0.01;
float Kd_x = 0.001;

float Kp_y = 0.1;
float Ki_y = 0.01;
float Kd_y = 0.001;

// Define servo limits
int servo_min_x = 0;
int servo_max_x = 180;

int servo_min_y = 0;
int servo_max_y = 180;

// Initialize variables
float prev_error_x = 0;
float prev_error_y = 0;
float integral_x = 0;
float integral_y = 0;
int servo_angle_x = 90;
int servo_angle_y = 90;

// Create servo objects
Servo head_tilt;
Servo head_pan;

void setup() {
  // Attach servos to pins
  head_tilt.attach(9);
  head_pan.attach(10);
}

void loop() {
  // Get ball position and camera frame size
  int ball_x = /* get ball x position */;
  int ball_y = /* get ball y position */;
  int frame_width = /* get camera frame width */;
  int frame_height = /* get camera frame height */;
  
  // Calculate errors
  float error_x = ball_x - (frame_width / 2);
  float error_y = ball_y - (frame_height / 2);
  
  // Calculate PID terms
  integral_x = integral_x + error_x;
  integral_y = integral_y + error_y;
  
  float derivative_x = error_x - prev_error_x;
  float derivative_y = error_y - prev_error_y;
  
  // Calculate servo angles
  servo_angle_x = Kp_x * error_x + Ki_x * integral_x + Kd_x * derivative_x;
  servo_angle_y = Kp_y * error_y + Ki_y * integral_y + Kd_y * derivative_y;
  
  // Clamp servo angles to limits
  servo_angle_x = max(min(servo_angle_x, servo_max_x), servo_min_x);
  servo_angle_y = max(min(servo_angle_y, servo_max_y), servo_min_y);
  
  // Set servo angles
  head_tilt.write(servo_angle_x);
  head_pan.write(servo_angle_y);
  
  // Update previous errors
  prev_error_x = error_x;
  prev_error_y = error_y;
  
  // Wait for next frame
  delay(10);  // adjust this delay as needed
}
