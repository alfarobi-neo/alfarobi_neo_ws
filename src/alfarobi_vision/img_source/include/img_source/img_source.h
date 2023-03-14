#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <yaml-cpp/yaml.h>

#include <opencv2/highgui/highgui.hpp>

#include <string.h>
#include <fstream>
#include <sstream>

#include "vision_utils/vision_common.h"
#include "vision_utils/FieldBoundary.h"
#include "vision_utils//LUT.h"

#define FROM_VIDEO 0

class ImageSource{
private:
    ros::NodeHandle nh_;

    //converting between ROS image messages to OpenCV images
    cv_bridge::CvImagePtr cv_img_ptr_subs_;
    cv_bridge::CvImage cv_img_pubs_;
    cv_bridge::CvImage cv_src_img_pubs_; //getting edited image from img_source package

    //for image transporting purposes from usb_cam to img_source
    image_transport::ImageTransport it_;
    image_transport::Subscriber it_subs_;
    image_transport::Publisher it_pubs_;
    image_transport::Publisher it_src_pubs_;

    void imageCallback(const sensor_msgs::ImageConstPtr &_msg);

    unsigned int img_encoding_;

    //stores image header which consist of time stamp and frame_id
    ros::Time stamp_;
    std::string frame_id_;

    /*https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html    
      for camera calibration purposes
    */
    sensor_msgs::CameraInfo cam_info_msg_;
    ros::Subscriber cam_info_sub_;
    ros::Publisher cam_info_pub_;

    void cameraInfoCallback(const sensor_msgs::CameraInfo &_msg);

    //Stores input image from usb_cam and calibrating it here
    cv::Mat in_img_;
    cv::Mat& setInputImage();
    //Stores output image and published to vision monitor (original, hsv, threshold)
    cv::Mat out_img_;
    void setOutputImage(const cv::Mat &_out_img);

    //storing frame mode image for hsv and threshold
    cv::Mat in_hsv_;
    cv::Mat thresh_image_;

    //LUT purposes variables
    //For getting LUT data of each color on field
    //Green, White, Orange (ball), etc. (if needed)
    cv::Mat LUT_data;
    std::string LUT_dir;
    ros::Subscriber LUT_sub_;
    void lutCallback(const vision_utils::LUTConstPtr &_msg);

    /*For frame modes in vision monitor
    1 -> Input
    2 -> HSV
    3 -> Threshold
    4 -> Output
    */
    int frame_mode_;
    ros::Subscriber frame_mode_subs_;
    void frameModeCallback(const std_msgs::Int8::ConstPtr &_msg);
    
    ros::Subscriber save_param_subs_;
    void saveLUTCallback(const std_msgs::Empty::ConstPtr &_msg);

    ros::Publisher update_params_pub_;

    //function to get convex hull of the full field image
    std::pair<cv::Mat, vision_utils::FieldBoundary> getFieldImage(const cv::Mat &_segmented_green);
    ros::Publisher field_boundary_pub_; //localization putposes
 
    //for transporting segmented colors to object detectors (Ball, Line, Goalpost, etc.)
    image_transport::Publisher it_sw_pub_;
    image_transport::Publisher it_inv_sg_pub_;
    image_transport::Publisher it_sbc_pub_;
    image_transport::Publisher it_sg_pub_;
    //yellow goalpost
    // image_transport::Publisher it_sy_pub_;
    // image_transport::Publisher it_sb_pub_;
    //---

    //cv_bridge for segmented colors
    cv_bridge::CvImage cv_sw_pub_;
    cv_bridge::CvImage cv_inv_sg_pub_;
    cv_bridge::CvImage cv_sbc_pub_;
    cv_bridge::CvImage cv_sg_pub_;
    //yellow goalpost
    // cv_bridge::CvImage cv_sy_pub_;
    // cv_bridge::CvImage cv_sb_pub_;
    //---

    static const float MIN_CONTOUR_AREA;
    static const float MIN_FIELD_CONTOUR_AREA;

    //function to segment color from source image/video
    /* Color description
    - Line       : White
    - Ball       : Orange
    - Goalpost   : Yellow
    - Background : background (for goalpost)
    */
    cv::Mat segmentColor(cv::Mat &_segmented_white, cv::Mat &_inv_segmented_green,
                                    cv::Mat &_segmented_ball_color, cv::Mat &_segmented_green
                                    //yellow goalpost
                                    // ,cv::Mat &_segmented_yellow, cv::Mat &_segmented_background
                                    );
    void localizationInputEnhance(cv::Mat &_input); //for invert_green and white color, localization purposes
    
    //publish segmented colorspublishObjectColors
    void publishObjectColors(const cv::Mat &_segmented_white, const cv::Mat &_inv_segmented_green,
                                          const cv::Mat &_segmented_ball_color, const cv::Mat &_segmented_green
                                          //yellow goalpost
                                        //   ,const cv::Mat &_segmented_yellow, const cv::Mat &_segmented_background
                                          ,vision_utils::FieldBoundary _field_boundary);
    
    //publish img_out variable to image_out topic
    void publishImage();

public:
    //Constructor, destructor and process must always be public
    ImageSource();
    ~ImageSource();
    void process();

    void saveLUT();
    void loadLUT();
};