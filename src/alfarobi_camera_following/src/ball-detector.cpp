#include <opencv2/opencv.hpp>
#include <iostream>

// Define camera parameters
const int CAMERA_INDEX = 0;
const int FRAME_WIDTH = 61.25 //640;
const int FRAME_HEIGHT = 41.60 //480;

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
    int desired_x = FRAME_WIDTH / 2;
    int desired_y = FRAME_HEIGHT / 2;

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
    }
}
