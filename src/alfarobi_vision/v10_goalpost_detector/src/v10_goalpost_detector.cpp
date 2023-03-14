#include "v10_goalpost_detector/v10_goalpost_detector.h"

#define FROM_VIDEO 0

const float GoalpostDetector::MIN_CONTOUR_AREA = 100.0f;
const float GoalpostDetector::MIN_OBJECT_CONTOUR_AREA = 1600.0f;

GoalpostDetector::GoalpostDetector()
    :nh_(ros::this_node::getName()),
      it_(this->nh_),
      it_subs_(it_.subscribe("image_in", 1, &GoalpostDetector::imageCallback, this)),
      it_pubs_(it_.advertise("image_out", 100)),
      cam_info_sub_(nh_.subscribe("camera_info_in", 100, &GoalpostDetector::cameraInfoCallback, this)),
      cam_info_pub_(nh_.advertise<sensor_msgs::CameraInfo>("camera_info_out", 100)),
      frame_mode_subs_(nh_.subscribe("frame_mode", 1, &GoalpostDetector::frameModeCallback, this)),
      save_param_subs_(nh_.subscribe("save_param", 1, &GoalpostDetector::saveParamCallback, this)),
      update_params_pub_(nh_.advertise<std_msgs::Empty > ("update_params", 10)),
      it_bf_sub_(it_.subscribe("goalpost_ref", 1, &GoalpostDetector::goalpostRefCallback, this)),
//baru
      segmented_yellow_sub_(nh_, "/img_source_node/segment_yellow",1),
      segmented_background_sub_(nh_, "/img_source_node/segment_background",1),
      input_sync_(SyncPolicy(10), segmented_yellow_sub_, segmented_background_sub_),

      goalpost_pos_pub_(nh_.advertise<geometry_msgs::Point > ("goalpost_pos", 100)),
      frame_mode_(0){

    nh_.param<std::string>("goalpost_config_path", goalpost_config_path,
                           ros::package::getPath("v10_goalpost_detector") + "/config/saved_config.yaml");

    param_cb_ = boost::bind(&GoalpostDetector::paramCallback, this, _1, _2);
    param_server_.setCallback(param_cb_);

//baru
    input_sync_.registerCallback(boost::bind(&GoalpostDetector::segmentColorCallback, this, _1, _2)) ;

    loadParam();
}

GoalpostDetector::~GoalpostDetector(){
}

void GoalpostDetector::segmentColorCallback(const sensor_msgs::ImageConstPtr &_segment_yellow_msg,
                                        const sensor_msgs::ImageConstPtr &_segment_background_msg){
    try{
        //[HW] : Fix encoding type
        sy_encoding_ = (_segment_yellow_msg->encoding.compare(sensor_msgs::image_encodings::MONO8))?Alfarobi::BGR8Bit:Alfarobi::GRAY8Bit;
        sb_encoding_ = (_segment_background_msg->encoding.compare(sensor_msgs::image_encodings::MONO8))?Alfarobi::BGR8Bit:Alfarobi::GRAY8Bit;
        
    }catch(cv_bridge::Exception &e){
        ROS_ERROR("[v10_goalpost_detector] cv_bridge exception: %s",e.what());
    }

    cv_sy_ptr_sub_ = cv_bridge::toCvCopy(_segment_yellow_msg);
    cv_sb_ptr_sub_ = cv_bridge::toCvCopy(_segment_background_msg);
}

void GoalpostDetector::loadParam(){
    YAML::Node config_file;
    try{
        config_file = YAML::LoadFile(goalpost_config_path.c_str());
    }catch(const std::exception &e){
        ROS_ERROR("[v10_goalpost_detector] Unable to open config file: %s", e.what());
    }

    config_.canny1 = config_file["canny1"].as<int>();
    config_.canny2 = config_file["canny2"].as<int>();
    config_.intersections = config_file["intersections"].as<int>();
    config_.threshold_corner = config_file["threshold_corner"].as<int>();

    goalpost_ref_ = cv::imread(ros::package::getPath("v10_goalpost_detector") + "/config/goalpost_ref.jpg");
    goalpost_ref_ = cvtMulti(goalpost_ref_);
    if(!goalpost_ref_.empty()){
        cv::calcHist(&goalpost_ref_, 1, hist_param_.channels, cv::Mat(), goalpost_ref_hist_, 2, hist_param_.hist_size, hist_param_.ranges);
        cv::normalize(goalpost_ref_hist_,goalpost_ref_hist_, .0, 1.0, cv::NORM_MINMAX);
    }
}

void GoalpostDetector::saveParam(){
    YAML::Emitter yaml_out;
    yaml_out << YAML::BeginMap;
    yaml_out << YAML::Key << "canny1" << YAML::Value << config_.canny1;
    yaml_out << YAML::Key << "canny2" << YAML::Value << config_.canny2;
    yaml_out << YAML::Key << "intersections" << YAML::Value << config_.intersections;
    yaml_out << YAML::Key << "threshold_corner" << YAML::Value << config_.threshold_corner;
    yaml_out << YAML::EndMap;
    std::ofstream file_out(goalpost_config_path.c_str());
    file_out << yaml_out.c_str();
    file_out.close();

    cv::imwrite(ros::package::getPath("v10_goalpost_detector") + "/config/goalpost_ref.jpg", goalpost_ref_);
}

void GoalpostDetector::frameModeCallback(const std_msgs::Int8::ConstPtr &_msg){
    frame_mode_ = _msg->data;
}

void GoalpostDetector::saveParamCallback(const std_msgs::Empty::ConstPtr &_msg){
    (void)_msg;
    saveParam();
}

void GoalpostDetector::goalpostRefCallback(const sensor_msgs::ImageConstPtr &_msg){
    cv_bf_ptr_sub_ = cv_bridge::toCvCopy(_msg,_msg->encoding);
    goalpost_ref_ = cv_bf_ptr_sub_->image;
    goalpost_ref_ = cvtMulti(goalpost_ref_);
    cv::calcHist(&goalpost_ref_, 1, hist_param_.channels, cv::Mat(), goalpost_ref_hist_, 2, hist_param_.hist_size, hist_param_.ranges);
    cv::normalize(goalpost_ref_hist_, goalpost_ref_hist_, .0, 1.0 , cv::NORM_MINMAX);
}

void GoalpostDetector::imageCallback(const sensor_msgs::ImageConstPtr &_msg){

    try{
        img_encoding_ = Alfarobi::GRAY8Bit;
        if(_msg->encoding.compare(sensor_msgs::image_encodings::MONO8))
            img_encoding_ = Alfarobi::GRAY8Bit;
#if FROM_VIDEO == 0
        if(_msg->encoding.compare(sensor_msgs::image_encodings::BGR8))
            img_encoding_ = Alfarobi::BGR8Bit;
#else
        if(_msg->encoding.compare(sensor_msgs::image_encodings::RGB8))
            img_encoding_ = Alfarobi::BGR8Bit;
#endif
    }catch(cv_bridge::Exception &e){
        ROS_ERROR("[v10_goalpost_detector] cv bridge exception: %s",e.what());
        return;
    }

    cv_img_ptr_subs_ = cv_bridge::toCvCopy(_msg,_msg->encoding);
    this->stamp_ = _msg->header.stamp;
    this->frame_id_ = _msg->header.frame_id;
}

void GoalpostDetector::cameraInfoCallback(const sensor_msgs::CameraInfo &_msg){
    //cam_info_msg_ = *_msg;
    
//    ROS_INFO("CHECK...");
}

void GoalpostDetector::paramCallback(v10_goalpost_detector::GoalpostDetectorParamsConfig &_config, uint32_t level){
    (void)level;
    this->config_ = _config;
}

void GoalpostDetector::publishImage(){
    cv_img_pubs_.image = out_img_.clone();

    //Stamp
    cv_img_pubs_.header.seq++;
    cv_img_pubs_.header.stamp = this->stamp_;
    cv_img_pubs_.header.frame_id = this->frame_id_;

    //microsoft lifecam brightness setting only work when the camera is capturing
    //setting first to zero brightness after first 2 frame then set to desired value
    //3 April 2019
    if(cv_img_pubs_.header.seq == 2){
        std_msgs::Empty empty_msg;
        update_params_pub_.publish(empty_msg);
    }else if(cv_img_pubs_.header.seq == 4){
        std_msgs::Empty empty_msg;
        update_params_pub_.publish(empty_msg);
    }

    switch(img_encoding_){
        case Alfarobi::GRAY8Bit:cv_img_pubs_.encoding = sensor_msgs::image_encodings::MONO8;break;
        case Alfarobi::BGR8Bit:cv_img_pubs_.encoding = sensor_msgs::image_encodings::RGB8;break;
        default:cv_img_pubs_.encoding = sensor_msgs::image_encodings::RGB8;break;
    }

    it_pubs_.publish(cv_img_pubs_.toImageMsg());
    cam_info_pub_.publish(cam_info_msg_);
}

cv::Mat& GoalpostDetector::setInputImage(){
    return in_img_;
}

void GoalpostDetector::setOutputImage(const cv::Mat &_out_img){
    out_img_ = _out_img.clone();
}

cv::Mat GoalpostDetector::cvtMulti(const cv::Mat &_goalpost_ref){
    cv::Mat yuv;
    cv::cvtColor(_goalpost_ref,yuv,CV_BGR2YUV);
    return yuv.clone();
}

float GoalpostDetector::checkTargetHistogram(cv::Mat _target_roi){

    if(goalpost_ref_.empty()){
        ROS_ERROR("[v10_goalpost_detector] Goalpost reference not found !!!");
        return -1;
    }
    _target_roi = cvtMulti(_target_roi);
    cv::MatND target_hist;
    cv::calcHist(&_target_roi, 1, hist_param_.channels, cv::Mat(), target_hist, 2, hist_param_.hist_size, hist_param_.ranges);
    cv::normalize(target_hist,target_hist, .0, 1.0, cv::NORM_MINMAX);

    return cv::compareHist(goalpost_ref_hist_, target_hist, CV_COMP_KL_DIV);
}

cv::Mat GoalpostDetector::getImageContours(const cv::Mat &_segmented_color){
    cv::Mat _object_contour = cv::Mat::zeros(_segmented_color.size(), CV_8UC1);
    Points contour_points;
    std::vector<Points > contours;
    std::vector<cv::Vec4i > hierarchy;

    cv::findContours(_segmented_color, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    for(size_t i = 0; i < contours.size(); i++){
        if(cv::contourArea(contours[i]) > MIN_OBJECT_CONTOUR_AREA){
            contour_points.insert(contour_points.end(), contours[i].begin(), contours[i].end());
        }
    }

    if(contour_points.size()){
        std::vector<Points > contour(1);
        cv::convexHull(contour_points,contour[0]);
        cv::Rect object_bound = cv::boundingRect(contour[0]);
        drawContours(_object_contour, contour, 0, cv::Scalar(255), cv::FILLED);
    }

    return _object_contour;
}

void GoalpostDetector::process(){
    if(cv_img_ptr_subs_ == nullptr)return;
    static geometry_msgs::Point last_goalpost_pos_;
    setInputImage() = cv_img_ptr_subs_->image;

    cv::Mat output_view = in_img_.clone();

    segmented_yellow = cv_sy_ptr_sub_->image;
    segmented_background = cv_sb_ptr_sub_->image;

    cv::Mat background_contour;
    background_contour = getImageContours(segmented_background);

    cv::Mat goalpost_in_background;
    cv::bitwise_and(segmented_yellow,background_contour,goalpost_in_background);

    // cv::dilate(goalpost_in_background, goalpost_in_background, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
    // cv::erode(goalpost_in_background, goalpost_in_background, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));

    cv::Mat goalpost_contour;
    goalpost_contour = getImageContours(goalpost_in_background);

    std::vector<Points > contours;
    //Approx NONE to get the authentic contours
    cv::findContours(goalpost_contour, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    cv::Mat ROI;
    cv::Rect goalpost_roi;
    cv::Mat goalpost;
    for(size_t i = 0; i < contours.size(); i++){
        float contour_area = cv::contourArea(contours[i]);
        std::cout<<contour_area<<std::endl; //max65000-69999
        if(contour_area > MIN_CONTOUR_AREA){
            cv::Rect rect_rough_roi = cv::boundingRect(contours[i]);
            cv::Point tl_pt = rect_rough_roi.tl();
//            cv::Point br_pt = rect_rough_roi.br();

            cv::Mat frame_rough_roi(goalpost_contour,rect_rough_roi);

            float roi_ratio = (rect_rough_roi.width < rect_rough_roi.height)?(float)rect_rough_roi.width/(float)rect_rough_roi.height:
                                                                             (float)rect_rough_roi.height/(float)rect_rough_roi.width;
            // std::cout<<"Ratio 1 : "<<roi_ratio<<std::endl;
            // std::cout<<"Ratio 2 : "<<1.0f/roi_ratio<<std::endl;
            std::vector<cv::Mat> sub_frame;
            if(roi_ratio >= 0.16f && 1.0f/roi_ratio <= 8.33f){ //0.30  kanan->3.16
                if(contour_area < 69999){
                    Points outer_rect;
                    cv::convexHull(contours[i],outer_rect);
                    cv::Moments moment;
                    moment = cv::moments(outer_rect,true);
                    cv::Point goalpost_com(moment.m10/moment.m00, moment.m01/moment.m00);

                    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
                    std::vector<cv::Rect> boundRect( contours.size() );
                    std::vector<cv::Point2f> centers( contours.size() );

                    cv::approxPolyDP( contours[i], contours_poly[i], 3, true );
                    boundRect[i] = cv::boundingRect( contours_poly[i] );

                    cv::Mat output_view_roi(output_view,boundRect[i]);
                    cv::Mat goalpost_region(output_view_roi.size(),CV_8UC3,cv::Scalar(0,255,255));
                    goalpost = goalpost_region.clone();
                    cv::addWeighted(output_view_roi, .5, goalpost_region, .5, .0,output_view_roi);

                    cv::Point2f point_x;
                    point_x = (boundRect[i].br() + boundRect[i].tl())*0.5;

                    // cv::circle(output_view, cv::Point(point_x.x, boundRect[i].br().y-10), 3, cv::Scalar(255, 255, 255),cv::FILLED);
                    
                    if(!contours.size()){
                        goalpost_pos_.x = -1;
                        goalpost_pos_.y = -1;
                        goalpost_pos_.z = 0;
                        goalpost_pos_pub_.publish(goalpost_pos_);
                    }else {
                        goalpost_pos_.x = point_x.x;
                        goalpost_pos_.y = boundRect[i].br().y-10;
                        goalpost_pos_.z = 0;
                        goalpost_pos_pub_.publish(goalpost_pos_);
                    }
                }
            }
        }
    }

//    cv::imshow("goalpost_contour",goalpost_contour);
//    cv::imshow("Goalpost", output_view);
//    cv::waitKey(1);

#if DEBUG == 1
    ROS_WARN("[Settings] Canny 1: %i", config_.canny1);
    ROS_WARN("[Settings] Canny 2: %i", config_.canny2);
    ROS_WARN("[Settings] Intersections: %i", config_.intersections);
    ROS_WARN("[Settings] Threshold: %i", config_.threshold_corner);    
#endif

    //For purpose GUI only
    // switch(frame_mode_){
    // case 1:setOutputImage(in_hsv_);break;
    // case 2:setOutputImage(thresh_image);break;
    // case 3:setOutputImage(output_view);break;
    // default:setOutputImage(in_img_);break;
    // }
    // publishImage();

    //yellow goalpost
    // if(frame_mode_ == 3){
    //     setOutputImage(output_view);
    // }
    // publishImage();
}