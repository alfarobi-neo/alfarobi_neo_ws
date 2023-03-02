//In the camera vision program, publish the ball position and frame dimensions to a ROS topic:

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::Publisher ball_pos_pub;

void publish_ball_pos(float ball_x, float ball_y, float frame_width, float frame_height) {
  std_msgs::Float32MultiArray msg;
  msg.data.push_back(ball_x);
  msg.data.push_back(ball_y);
  msg.data.push_back(frame_width);
  msg.data.push_back(frame_height);
  ball_pos_pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_vision");
  ros::NodeHandle nh;
  ball_pos_pub = nh.advertise<std_msgs::Float32MultiArray>("ball_pos", 1);
  // ... main loop to detect ball position and frame dimensions
  publish_ball_pos(ball_x, ball_y, frame_width, frame_height);
  // ...
}


//In the PID control program, subscribe to the ball position topic and calculate the servo angles based on the received data:

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Servo.h>

// Define position PID gains
float Kp_x = 0.1;
float Ki_x = 0.01;
float Kd_x = 0.001;

float Kp_y = 0.1;
float Ki_y = 0.01;
float Kd_y = 0.001;

// Define velocity PID gains
float Kv_x = 0.1;
float Ka_x = 0.001;

float Kv_y = 0.1;
float Ka_y = 0.001;

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
float prev_velocity_x = 0;
float prev_velocity_y = 0;

// Create servo objects
Servo head_tilt;
Servo head_pan;

void ball_pos_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  // Extract ball position and frame dimensions from message
  float ball_x = msg->data[0];
  float ball_y = msg->data[1];
  float frame_width = msg->data[2];
  float frame_height = msg->data[3];
  // Calculate error
  float error_x = ball_x - frame_width / 2;
  float error_y = ball_y - frame_height / 2;
  // Calculate position and velocity PIDs
  float velocity_x = Kv_x * error_x + Ka_x * (error_x - prev_error_x);
  float velocity_y = Kv_y * error_y + Ka_y * (error_y - prev_error_y);
  integral_x += error_x;
  integral_y += error_y;
  float position_x = Kp_x * error_x + Ki_x * integral_x + Kd_x * (error_x - prev_error_x);
  float position_y = Kp_y * error_y + Ki_y * integral_y + Kd
