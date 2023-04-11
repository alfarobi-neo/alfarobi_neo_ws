#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
//#include <v9_ball_detector/BallDetectorConfig.h>
#include <v9_ball_detector/BallDetectorParamsConfig.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>

#include <yaml-cpp/yaml.h>

#include "vision_utils/vision_common.h"
#include "vision_utils/LineTip.h"
#include "vision_utils/fitcircle.h"

#include <string.h>
#include <fstream>
#include <sstream>

// #define DEBUG

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image> SyncPolicy;
                                                    
typedef message_filters::Synchronizer<SyncPolicy> InputUtils;

class BallDetector{
private:
    ros::NodeHandle nh_;

    cv_bridge::CvImagePtr cv_img_ptr_subs_;
    cv_bridge::CvImage cv_img_pubs_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber it_subs_;
    image_transport::Publisher it_pubs_;

    void imageCallback(const sensor_msgs::ImageConstPtr &_msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfo &_msg);
    void publishImage();

    ros::Publisher update_params_pub_;

    cv::Mat in_img_;
    cv::Mat out_img_;

    sensor_msgs::CameraInfo cam_info_msg_;
    ros::Subscriber cam_info_sub_;
    ros::Publisher cam_info_pub_;

    unsigned int img_encoding_;

    geometry_msgs::Point ball_pos_;
    ros::Publisher ball_pos_pub_;

    ros::Time stamp_;
    std::string frame_id_;

    cv::Mat& setInputImage();
    void setOutputImage(const cv::Mat &_out_img);

    std::string ball_config_path;
    v9_ball_detector::BallDetectorParamsConfig config_;
    dynamic_reconfigure::Server<v9_ball_detector::BallDetectorParamsConfig> param_server_;
    dynamic_reconfigure::Server<v9_ball_detector::BallDetectorParamsConfig>::CallbackType param_cb_;
    void paramCallback(v9_ball_detector::BallDetectorParamsConfig &_config, uint32_t level);

//baru
//--
    cv::Mat segmented_ball_color;
    cv::Mat segmented_green;

    int sbc_encoding_;
    int sg_encoding_;

    cv_bridge::CvImagePtr cv_sbc_ptr_sub_;
    cv_bridge::CvImagePtr cv_sg_ptr_sub_;

    void segmentColorCallback(const sensor_msgs::ImageConstPtr &_segment_ball_msg,
                             const sensor_msgs::ImageConstPtr &_segment_green_msg);

    message_filters::Subscriber<sensor_msgs::Image > segmented_ball_color_sub_;
    message_filters::Subscriber<sensor_msgs::Image > segmented_green_sub_;

    InputUtils input_sync_;
//--

    //====================

    void ballRefCallback(const sensor_msgs::ImageConstPtr &_msg);
    image_transport::Subscriber it_bf_sub_;
    cv_bridge::CvImagePtr cv_bf_ptr_sub_;
    cv::Mat ball_ref_;
    cv::MatND ball_ref_hist_;
    float checkTargetHistogram(cv::Mat _target_roi);
    class HistParam{
    public:
        int channels[2];
        int hist_size[2];
        float h_ranges[2];
        float s_ranges[2];
        float v_ranges[2];
        const float* ranges[2];
        HistParam():channels{0,1},hist_size{32,32},
            h_ranges{0,255},s_ranges{0,255},v_ranges{0,255},
            ranges{s_ranges,s_ranges}{

        }
    }hist_param_;

    static const float MIN_CONTOUR_AREA;
    static const float MIN_OBJECT_CONTOUR_AREA;

    cv::Mat cvtMulti(const cv::Mat &_ball_ref);

    void filterContourData(std::vector<cv::Mat> &divided_roi, cv::Point top_left_pt,
                           std::vector<Points > &selected_data, cv::Mat *debug_mat, int sub_mode);

    int frame_mode_;
    ros::Subscriber frame_mode_subs_;
    void frameModeCallback(const std_msgs::Int8::ConstPtr &_msg);

    ros::Subscriber save_param_subs_;
    void saveParamCallback(const std_msgs::Empty::ConstPtr &_msg);

    cv::Mat getImageContours(const cv::Mat &_segmented_color);

    ros::Subscriber pred_status_sub_;
    bool pred_status_;
    void predStatusCb(const std_msgs::Bool::ConstPtr &_msg){
        pred_status_ = _msg->data;
    }

    void lineTipCallback(const vision_utils::LineTipConstPtr &_msg);
    ros::Subscriber line_tip_sub_;
    vision_utils::LineTip line_tip_;

    //Prediction Part
    Points regression_data_;
    std::vector<cv::Mat > est_trajectory_;
    std::vector<cv::Mat> getBallPosPrediction(const Points &_data);

public:
    BallDetector();
    ~BallDetector();
    void process();
    void saveParam();
    void loadParam();
};
