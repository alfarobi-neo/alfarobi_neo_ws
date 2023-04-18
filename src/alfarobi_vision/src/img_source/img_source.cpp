#include "img_source/img_source.h"

#define FROM_VIDEO 0

const float ImageSource::MIN_CONTOUR_AREA = 100.0f;
const float ImageSource::MIN_FIELD_CONTOUR_AREA = 1600.0f;

ImageSource::ImageSource()
    :nh_(ros::this_node::getName()),
        it_(this->nh_),
        it_subs_(it_.subscribe("image_in", 1, &ImageSource::imageCallback, this)),
        it_pubs_(it_.advertise("image_out", 100)),
        it_src_pubs_(it_.advertise("image_src", 100)),
        cam_info_sub_(nh_.subscribe("camera_info_in", 100, &ImageSource::cameraInfoCallback, this)),
        cam_info_pub_(nh_.advertise<sensor_msgs::CameraInfo>("camera_info_out", 100)),
        update_params_pub_(nh_.advertise<std_msgs::Empty> ("update_params", 10)),
        frame_mode_subs_(nh_.subscribe("frame_mode", 1, &ImageSource::frameModeCallback, this)),
        save_param_subs_(nh_.subscribe("save_param", 1, &ImageSource::saveLUTCallback, this)),
        LUT_sub_(nh_.subscribe("LUT_data", 1, &ImageSource::lutCallback, this)),
        it_sw_pub_(it_.advertise("segment_white", 10)), //lines
        it_sg_pub_(it_.advertise("segment_green", 10)), //green
        it_inv_sg_pub_(it_.advertise("inv_segment_green", 10)), //localization purposes
        it_sbc_pub_(it_.advertise("segment_ball_color", 10)), //orange ball
        //yellow goalpost
        // it_sy_pub_(it_.advertise("segment_yellow", 10)), // goalpost
        // it_sb_pub_(it_.advertise("segment_background", 10)), // background
        //---
        field_boundary_pub_(nh_.advertise<alfarobi_msgs_srvs_actions::FieldBoundary > ("field_boundary", 10)),
        frame_mode_(0) {

        LUT_dir = ros::package::getPath("alfarobi_vision") + "/LUT/tabel_warna.xml";
        
        loadLUT();
}

ImageSource::~ImageSource(){
}

//set input image
cv::Mat& ImageSource::setInputImage(){
    return in_img_;
}

//set output image to publish
void ImageSource::setOutputImage(const cv::Mat &_out_img){
    out_img_ = _out_img.clone();
}

void ImageSource::imageCallback(const sensor_msgs::ImageConstPtr &_msg){

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
        ROS_ERROR("[img_source] cv bridge exception: %s",e.what());
        return;
    }

    cv_img_ptr_subs_ = cv_bridge::toCvCopy(_msg,_msg->encoding);
    this->stamp_ = _msg->header.stamp;
    this->frame_id_ = _msg->header.frame_id;
}

void ImageSource::cameraInfoCallback(const sensor_msgs::CameraInfo &_msg){
//    cam_info_msg_ = *_msg;

//    ROS_INFO("CHECK...");
}

void ImageSource::frameModeCallback(const std_msgs::Int8::ConstPtr &_msg){
    frame_mode_ = _msg->data;
}

void ImageSource::lutCallback(const alfarobi_msgs_srvs_actions::LUTConstPtr &_msg){
//    uchar* LUT_ptr = LUT_data.data;
    for(size_t i = 0; i < _msg->color.size(); i++){
        int h = (int)_msg->color[i].x;
        int s = (int)_msg->color[i].y;
        LUT_data.at<uchar>(h,s) = (int) _msg->color_class.data;
//        std::cout << h << " , " << s << " ; " << (int) _msg->color_class.data << std::endl;
//        LUT_ptr[s + h*256] = (int) _msg->color[i].z;
    }
//    cv::Mat diff = tempor != LUT_data;
//    if(cv::countNonZero(diff)==0)std::cout << "SAMA" << std::endl;
}

void ImageSource::saveLUTCallback(const std_msgs::Empty::ConstPtr &_msg){
    (void)_msg;
    saveLUT();
}

void ImageSource::loadLUT(){
    cv::FileStorage fs(LUT_dir.c_str(),cv::FileStorage::READ);
    fs["Tabel_Warna"] >> LUT_data;
    fs.release();
}

void ImageSource::saveLUT(){
    cv::FileStorage fs(LUT_dir.c_str(),cv::FileStorage::WRITE);
    fs << "Tabel_Warna" << LUT_data;
    fs.release();
}

//function to segment color from source image/video
/* Color description
    - Line       : White
    - Ball       : Orange
    - Goalpost   : Yellow
    - Background : background (for goalpost)
*/
cv::Mat ImageSource::segmentColor(cv::Mat &_segmented_white, cv::Mat &_inv_segmented_green,
                                    cv::Mat &_segmented_ball_color, cv::Mat &_segmented_green
                                    // yellow goalpost
                                    // ,cv::Mat &_segmented_yellow, cv::Mat &_segmented_background
                                    ){

    cv::Mat blank = cv::Mat::zeros(Alfarobi::FRAME_HEIGHT, Alfarobi::FRAME_WIDTH, CV_8UC1);
    cv::Mat out_segment = cv::Mat::zeros(Alfarobi::FRAME_HEIGHT, Alfarobi::FRAME_WIDTH, CV_8UC3);
    
    //ball
    cv::Mat segmented_ball_color= blank.clone();
    //localization utilities
    cv::Mat segmented_green = blank.clone();
    cv::Mat segmented_white = blank.clone();
    //yellow goalpost
    // cv::Mat segmented_yellow = blank.clone();
    // cv::Mat segmented_background = blank.clone();

    cv::cvtColor(in_img_,in_hsv_,CV_BGR2HSV);

    /*cv::Mat gray;
    cv::cvtColor(in_img_,gray,CV_BGR2GRAY);
    cv::medianBlur(gray,gray,3);
    cv::Mat kernel = (cv::Mat_<double>(3,3) << 0.111111111,0.111111111,0.111111111,0.111111111,0.111111111,0.111111111,0.111111111,0.111111111,0.111111111);
    cv::filter2D(gray,gray,CV_8UC1,kernel);
    cv::Mat lutable = cv::Mat(1,256,CV_8U);
    uchar *lutable_ptr = lutable.ptr();
    for(int i=0;i<256;i++){
        lutable_ptr[i] = pow(i/255.0,0.2)*255.0;
    }
    cv::Mat resulttt = gray.clone();
    cv::LUT(gray,lutable,resulttt);
    cv::equalizeHist(gray,gray);
    cv::adaptiveThreshold(gray,gray,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,11,1);
    cv::imshow("GRAY",gray);*/

    int num_cols = Alfarobi::FRAME_WIDTH;
    int num_rows = Alfarobi::FRAME_HEIGHT;

    // auto LUT_ptr = LUT_data.data;
    for(int i = 0; i < num_rows; i++){
        cv::Vec3b* in_hsv_ptr = in_hsv_.ptr<cv::Vec3b>(i);
        cv::Vec3b* out_segment_ptr = out_segment.ptr<cv::Vec3b>(i);
        uchar* sg_ptr = segmented_green.ptr<uchar>(i);
        uchar* sbc_ptr = segmented_ball_color.ptr<uchar>(i);
        uchar* sw_ptr = segmented_white.ptr<uchar>(i);
        //yellow goalpost
        // uchar* sy_ptr = segmented_yellow.ptr<uchar>(i);
        // uchar* sb_ptr = segmented_background.ptr<uchar>(i); 
        for(int j = 0; j < num_cols; j++){
//            std::cout << i << " , " << j << " ; " << (int)in_hsv_ptr[j][0] << " , " << (int)in_hsv_ptr[j][1] << std::endl;
            // if(LUT_data.at<uchar>(in_hsv_ptr[j][0], in_hsv_ptr[j][1]) == 1){
            // uchar pres_class = LUT_ptr[in_hsv_ptr[j][1] + in_hsv_ptr[j][0]*num_cols];
            uchar pres_class = LUT_data.at<uchar>(in_hsv_ptr[j][0], in_hsv_ptr[j][1]);
            //field
            if(pres_class == 1){
                sg_ptr[j] = 255;
                out_segment_ptr[j][0] = 0;
                out_segment_ptr[j][1] = 200;
                out_segment_ptr[j][2] = 0;  
            //ball             
            }else if(pres_class == 2){
                sbc_ptr[j] = 255;
                out_segment_ptr[j][0] = 0;
                out_segment_ptr[j][1] = 140;
                out_segment_ptr[j][2] = 255;
            //line
            }else if(pres_class == 3){
                sw_ptr[j] = 255;
                out_segment_ptr[j][0] = 255;
                out_segment_ptr[j][1] = 255;
                out_segment_ptr[j][2] = 255;
            }
            //yellow goalpost
            // //goalpost
            // else if(pres_class == 5){
            //     sy_ptr[j] = 255;
            //     out_segment_ptr[j][0] = 255;
            //     out_segment_ptr[j][1] = 255;
            //     out_segment_ptr[j][2] = 0;
            // //background
            // }else if(pres_class == 6){
            //     sb_ptr[j] = 255;
            //     out_segment_ptr[j][0] = 0;
            //     out_segment_ptr[j][1] = 0;
            //     out_segment_ptr[j][2] = 255;
            // }
            //---
        }
    }

    cv::Mat inv_segmented_green;
    cv::bitwise_not(segmented_green,inv_segmented_green);

    localizationInputEnhance(segmented_white);
    localizationInputEnhance(inv_segmented_green);
    
    _segmented_green = segmented_green.clone();
    _inv_segmented_green = inv_segmented_green.clone();
    _segmented_ball_color= segmented_ball_color.clone();
    _segmented_white = segmented_white.clone();
    //yellow goalpost
    // _segmented_yellow = segmented_yellow.clone();
    // _segmented_background = segmented_background.clone();

    return out_segment;
}

void ImageSource::localizationInputEnhance(cv::Mat &_input){
    cv::Mat result = _input.clone();
    cv::dilate(result, result ,cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)), cv::Point(), 1);

    std::vector<Points > contours;
    std::vector<cv::Vec4i > hierarchy;
    std::vector<double > contours_area;

    cv::findContours(result, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    result = cv::Mat::zeros(result.size(),CV_8UC1);

    for(size_t i=0;i<contours.size();i++){
        contours_area.emplace_back(cv::contourArea(contours[i]));
        if(contours_area[i] > MIN_CONTOUR_AREA && hierarchy[i][3] == -1){
            Points approx_curve;
            cv::approxPolyDP(contours[i], approx_curve, 0.002*cv::arcLength(contours[i],true),true);
            std::vector<Points > target_contour;
            target_contour.push_back(approx_curve);
            drawContours(result, target_contour, 0, cv::Scalar(255), cv::FILLED);
        }
    }

    for(size_t i=0;i<contours.size();i++){
        if(contours_area[i] > MIN_CONTOUR_AREA && hierarchy[i][3] > -1){
            Points approx_curve;
            cv::approxPolyDP(contours[i], approx_curve, 0.002*cv::arcLength(contours[i],true),true);
            std::vector<Points > target_contour;
            target_contour.push_back(approx_curve);
            drawContours(result, target_contour, 0, cv::Scalar(0), cv::FILLED);
        }
    }
    _input = result.clone();
}

//function to get convex hull of the full field image (including other object in the field)
std::pair<cv::Mat, alfarobi_msgs_srvs_actions::FieldBoundary > ImageSource::getFieldImage(const cv::Mat &_segmented_green){
    cv::Mat _field_contour = cv::Mat::zeros(_segmented_green.size(), CV_8UC1);
    alfarobi_msgs_srvs_actions::FieldBoundary field_boundary;
    Points contour_points;
    std::vector<Points > contours;
    std::vector<cv::Vec4i > hierarchy;

    cv::findContours(_segmented_green, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    for(size_t i = 0; i < contours.size(); i++){
        if(cv::contourArea(contours[i]) > MIN_FIELD_CONTOUR_AREA){
            contour_points.insert(contour_points.end(), contours[i].begin(), contours[i].end());
        }
    }

    if(contour_points.size()){
        std::vector<Points > contour(1);
        cv::convexHull(contour_points,contour[0]);
        cv::Rect field_bound = cv::boundingRect(contour[0]);
        drawContours(_field_contour, contour, 0, cv::Scalar(255), cv::FILLED);
        //[HW] Scan from dual direction
        for(int i=field_bound.tl().x;
            i<field_bound.br().x;i++){
            geometry_msgs::Vector3 temp;
            temp.x = i;
            temp.y = -1;
            temp.z = field_bound.br().y-1;
            for(int j=field_bound.tl().y;
                j<field_bound.br().y;j++){
                if(_field_contour.at<uchar >(j,i) > 0 &&
                        temp.y==-1){
                    temp.y = j;
                }else if(_field_contour.at<uchar >(j,i) == 0 &&
                         temp.y!=-1){
                    temp.z = j-1;
                    break;
                }
            }
            field_boundary.bound1.push_back(temp);
        }

        for(int i=field_bound.tl().y;
            i<field_bound.br().y;i++){
            geometry_msgs::Vector3 temp;
            temp.x = i;
            temp.y = -1;
            temp.z = field_bound.br().x-1;
            for(int j=field_bound.tl().x;
                j<field_bound.br().x;j++){
                if(_field_contour.at<uchar >(i,j) > 0 &&
                        temp.y==-1){
                    temp.y = j;
                }else if(_field_contour.at<uchar >(i,j) == 0 &&
                         temp.y!=-1){
                    temp.z = j-1;
                    break;
                }
            }
            field_boundary.bound2.push_back(temp);
        }
    }

    std::pair<cv::Mat, alfarobi_msgs_srvs_actions::FieldBoundary > result;
    result.first = _field_contour;
    result.second = field_boundary;
    return result;
}

//function for publishing segmented colors to other detector/program that needs it
void ImageSource::publishObjectColors(const cv::Mat &_segmented_white, const cv::Mat &_inv_segmented_green,
                                      const cv::Mat &_segmented_ball_color, const cv::Mat &_segmented_green
                                    // yellow goalpost
                                    //   ,const cv::Mat &_segmented_yellow, const cv::Mat &_segmented_background,
                                      ,alfarobi_msgs_srvs_actions::FieldBoundary _field_boundary){
    cv_sw_pub_.image = _segmented_white.clone();
    cv_inv_sg_pub_.image = _inv_segmented_green.clone();
    cv_sbc_pub_.image = _segmented_ball_color.clone();
    cv_sg_pub_.image = _segmented_green.clone();
    //yellow goalpost
    // cv_sy_pub_.image = _segmented_yellow.clone();
    // cv_sb_pub_.image = _segmented_background.clone();

    cv_sw_pub_.header.seq++;
    cv_inv_sg_pub_.header.seq++;
    cv_sbc_pub_.header.seq++;
    cv_sg_pub_.header.seq++;
    //yellow goalpost
    // cv_sy_pub_.header.seq++;
    // cv_sb_pub_.header.seq++;
    _field_boundary.header.seq++;

    cv_sw_pub_.header.stamp = this->stamp_;
    cv_inv_sg_pub_.header.stamp = this->stamp_;
    cv_sbc_pub_.header.stamp = this->stamp_;
    cv_sg_pub_.header.stamp = this->stamp_;
    //yellow goalpost
    // cv_sy_pub_.header.stamp = this->stamp_;
    // cv_sb_pub_.header.stamp = this->stamp_;
    _field_boundary.header.stamp = this->stamp_;

    cv_sw_pub_.header.frame_id = this->frame_id_;
    cv_inv_sg_pub_.header.frame_id = this->frame_id_;
    cv_sbc_pub_.header.frame_id = this->frame_id_;
    cv_sg_pub_.header.frame_id = this->frame_id_;
    //yellow goalpost
    // cv_sy_pub_.header.frame_id = this->frame_id_;
    // cv_sb_pub_.header.frame_id = this->frame_id_;
    _field_boundary.header.frame_id = this->frame_id_;

    cv_sw_pub_.encoding = sensor_msgs::image_encodings::MONO8;
    cv_inv_sg_pub_.encoding = sensor_msgs::image_encodings::MONO8;
    cv_sbc_pub_.encoding = sensor_msgs::image_encodings::MONO8;
    cv_sg_pub_.encoding = sensor_msgs::image_encodings::MONO8;
    //yellow goalpost
    // cv_sy_pub_.encoding = sensor_msgs::image_encodings::MONO8;
    // cv_sb_pub_.encoding = sensor_msgs::image_encodings::MONO8;

    it_sw_pub_.publish(cv_sw_pub_.toImageMsg());
    it_inv_sg_pub_.publish(cv_inv_sg_pub_.toImageMsg());
    it_sbc_pub_.publish(cv_sbc_pub_.toImageMsg());
    it_sg_pub_.publish(cv_sbc_pub_.toImageMsg());
    //yellow goalpost
    // it_sy_pub_.publish(cv_sy_pub_.toImageMsg());
    // it_sb_pub_.publish(cv_sb_pub_.toImageMsg());
    field_boundary_pub_.publish(_field_boundary);
}

//publishing image
void ImageSource::publishImage(){
    cv_img_pubs_.image = out_img_.clone();
    cv_src_img_pubs_.image = in_img_.clone();

    //Stamp
    cv_img_pubs_.header.seq++;
    cv_src_img_pubs_.header.seq++;
    cv_img_pubs_.header.stamp = this->stamp_;
    cv_src_img_pubs_.header.stamp = this->stamp_;
    cv_img_pubs_.header.frame_id = this->frame_id_;
    cv_src_img_pubs_.header.frame_id = this->frame_id_;

    //microsoft lifecam brightness setting only work when the camera is capturing
    //setting first to zero brightness after first 2 frame then set to desired value
    //3 April 2019
    if(cv_img_pubs_.header.seq == 2 || cv_src_img_pubs_.header.seq == 2 ){
        std_msgs::Empty empty_msg;
        update_params_pub_.publish(empty_msg);
    }else if(cv_img_pubs_.header.seq == 4 || cv_src_img_pubs_.header.seq == 4){
        std_msgs::Empty empty_msg;
        update_params_pub_.publish(empty_msg);
    }

    switch(img_encoding_){
        case Alfarobi::GRAY8Bit:
            cv_img_pubs_.encoding = sensor_msgs::image_encodings::MONO8;
            cv_src_img_pubs_.encoding = sensor_msgs::image_encodings::MONO8;
            break;
        case Alfarobi::BGR8Bit: 
            cv_img_pubs_.encoding = sensor_msgs::image_encodings::RGB8;
            cv_src_img_pubs_.encoding = sensor_msgs::image_encodings::RGB8;
            break;
        default:
            cv_img_pubs_.encoding = sensor_msgs::image_encodings::RGB8;
            cv_src_img_pubs_.encoding = sensor_msgs::image_encodings::RGB8;
            break;
    }


    it_pubs_.publish(cv_img_pubs_.toImageMsg());
    it_src_pubs_.publish(cv_src_img_pubs_.toImageMsg());

    cam_info_pub_.publish(cam_info_msg_);
}

//main process is happening here
void ImageSource::process(){
    //checking if there's input coming from usb cam
    if(cv_img_ptr_subs_ == nullptr)return;

    //set the input image in img_source package from usb_cam
    setInputImage() = cv_img_ptr_subs_->image;

    // cv::Mat output_view = in_img_.clone();

    //segmenting color
    cv::Mat segmented_green,segmented_ball_color,
            segmented_white, inv_segmented_green;
            //yellow goalpost
            // segmented_yellow, segmented_background;

    /*this variable keeps all the segmented colors
      when viewing from vision monitor, it can be seen
      if the color is in the LUT data or not from
      the grayscale that appeared in the particular
      shade of the color 
    */
    thresh_image_ = segmentColor(segmented_white, inv_segmented_green, 
                                        segmented_ball_color, segmented_green);
                                        //yellow goalpost
                                        // segmented_yellow,segmented_background);
    
    //variable for storing full field image, including the objects in the field
    cv::Mat field_contour;
    std::pair<cv::Mat, alfarobi_msgs_srvs_actions::FieldBoundary > field_prop = getFieldImage(segmented_green);
    
    //kalau mau contour lapangan full, publish variabel "field_contour" aja
    field_contour = field_prop.first;
    
    publishObjectColors(segmented_white,inv_segmented_green,
                        segmented_ball_color, segmented_green,
                        //yellow goalpost
                        // segmented_yellow,segmented_background,
                        field_prop.second);

    // cv::imshow("img_src",src_img_);
    // cv::imshow("img_in",in_img_);
    // cv::imshow("segmented_ball_color",segmented_ball_color);
    // cv::imshow("thresh_image",thresh_image_);
    // cv::imshow("segmented_green",segmented_green);
    // cv::imshow("segmented_white",segmented_white);

    // Setting the output image (out_img_) thats going to
    // be published to vision monitor depending on the frame mode
    switch(frame_mode_){
        case 1:setOutputImage(in_hsv_);break;
        case 2:setOutputImage(thresh_image_);break;
        // case 3:setOutputImage(output_view);break;
        case 3:break;
        default:setOutputImage(in_img_);break;
    }
    // cv::imshow("img_out",out_img_);
    // cv::waitKey(1);
    //publised the image
    publishImage();
}