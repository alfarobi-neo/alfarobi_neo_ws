#include <opencv2/opencv.hpp>
#include <iostream>
#include "alfarobi_dxlsdk/servo_controller.h"

//id 19 = head_pan
//id 20 = head_tilt

// Define camera parameters
const int CAMERA_INDEX = 0;
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

// Define HSV color range for the orange ball
const cv::Scalar ORANGE_MIN(0, 100, 100);
const cv::Scalar ORANGE_MAX(20, 255, 255);

int main()
{
    // Initialize the camera
    cv::VideoCapture cap(CAMERA_INDEX);
    if (!cap.isOpened())
    {
        std::cout << "Failed to open camera!" << std::endl;
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    // Create a window to display the video stream
    cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);

    // Define the PID gains for controlling the camera movement
    double Kp = 0.1;
    double Ki = 0.01;
    double Kd = 0.01;

    // Define variables for PID control
    double error_x = 0.0;
    double error_y = 0.0;
    double error_x_prev = 0.0;
    double error_y_prev = 0.0;
    double error_x_sum = 0.0;
    double error_y_sum = 0.0;

    // Define the desired position of the ball in the frame
    int desired_x = FRAME_WIDTH / 2; //320
    int desired_y = FRAME_HEIGHT / 2; //240

    // Loop over frames from the camera
    while (true)
    {
        // Read a frame from the camera
        cv::Mat frame;
        cap.read(frame);

        // Convert the frame to HSV color space
        cv::Mat hsv_frame;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        // Threshold the image to extract the orange ball
        cv::Mat orange_mask;
        cv::inRange(hsv_frame, ORANGE_MIN, ORANGE_MAX, orange_mask);

        // Morphological opening to remove noise
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(orange_mask, orange_mask, cv::MORPH_OPEN, kernel);

        // Find the center of the orange ball
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(orange_mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        cv::Point2f center;
        float radius = 0.0;
        if (contours.size() > 0)
        {
            cv::minEnclosingCircle(contours[0], center, radius); 
            cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 0, 255), 2);
        }

        // Calculate the error between the ball's center and the desired position
        error_x = center.x - desired_x;
        error_y = center.y - desired_y;

        // Update the PID error sum
        error_x_sum += error_x;
        error_y_sum += error_y;

        // Calculate the PID control signal
        double control_x = Kp * error_x + Ki * error_x_sum + Kd * (error_x - error_x_prev);
        double control_y = Kp * error_y + Ki * error_y_sum + Kd * (error_y - error_y_prev);

        if (control_y > 1.0)
        {
            control_y > 1.0;
        }
        else if (control_y < -1.0)
        {
            control_y = -1.0;
        }


        //update the previous errors for the next iteration
        error_x_prev = error_x;
        error_y_prev = error_y;

        // Display the frame with the ball's center and the desired position
        cv::circle(frame, cv::Point(desired_x, desired_y), 5, cv::Scalar(255, 0, 0), 2);
        cv::imshow("Video", frame);

        // goal_pos1 = cv::Point;
        // goal_pos2 = cv::Point;

        // // Control the camera movement based on the PID control signal
        // robot->write(19, robot->deg2Bit(goal_pos1), goal_time1); //max 132 if using velocity-based, in milliseconds if using time-based
        // robot->write(20, robot->deg2Bit(goal_pos2), goal_time2);


        // Exit the loop if the user presses the 'q' key
        if (cv::waitKey(1) == 'q')
        {
            break;
        }

        // Connect to the servo controller
        if (!servo_controller.connect())
        {
            std::cout << "Failed to connect to servo controller!" << std::endl;
            return -1;
        }

        // Set the camera movement to its initial position
        int initial_position = 90;
        servo_controller.move(19, initial_position);
        servo_controller.move(20, initial_position);

        // Define the maximum and minimum servo positions
        int max_position = 180;
        int min_position = 0;

        // Define the time interval for updating the servo position
        int update_interval = 10; // in milliseconds

        // Loop over frames from the camera
        while (true)
        {
            // Read a frame from the camera
            cv::Mat frame;
            cap.read(frame);

            // Convert the frame to HSV color space
            cv::Mat hsv_frame;
            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

            // Threshold the image to extract the orange ball
            cv::Mat orange_mask;
            cv::inRange(hsv_frame, ORANGE_MIN, ORANGE_MAX, orange_mask);

            // Morphological opening to remove noise
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            cv::morphologyEx(orange_mask, orange_mask, cv::MORPH_OPEN, kernel);

            // Find the center of the orange ball
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(orange_mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            cv::Point2f center;
            float radius = 0.0;
            if (contours.size() > 0)
            {
                cv::minEnclosingCircle(contours[0], center, radius); 
                cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 0, 255), 2);
            }

            // Calculate the error between the ball's center and the desired position
            error_x = center.x - desired_x;
            error_y = center.y - desired_y;

            // Update the PID error sum
            error_x_sum += error_x;
            error_y_sum += error_y;

            // Calculate the PID control signal
            double control_x = Kp * error_x + Ki * error_x_sum + Kd * (error_x - error_x_prev);
            double control_y = Kp * error_y + Ki * error_y_sum + Kd * (error_y - error_y_prev);

            // Calculate the desired position of the camera based on the PID control signal
            int goal_position_x = initial_position + static_cast<int>(control_x);
            int goal_position_y = initial_position + static_cast<int>(control_y);

            // Clamp the desired position within the maximum and minimum servo positions
            goal_position_x = std::max(min_position, std::min(max_position, goal_position_x));
            goal_position_y = std::max(min_position, std::min(max_position, goal_position_y));

            // Update the servo position if it has changed
            if (goal_position_x != goal_pos1.x)
            {
                goal_pos1.x = goal_position_x;
                // servo_controller.move(19, goal_position_x);
                robot->write(19, robot->deg2Bit(goal_position_x), goal_time1);


            }
            if (goal_position_y != goal_pos2.y)
            {
                goal_pos2.y = goal_position_y;
                // servo_controller.move(20, goal_position_y);
                robot->write(20, robot->deg2Bit(goal_position_y), goal_time2);

            }
        }

    }

    // robot->write(19, robot->deg2Bit(goal_pos1), goal_time1); //max 132 if using velocity-based, in milliseconds if using time-based
    // robot->write(20, robot->deg2Bit(goal_pos2), goal_time2);

}