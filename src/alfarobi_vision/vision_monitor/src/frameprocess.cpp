#include "vision_monitor/frameprocess.h"

FrameProcess::FrameProcess(QObject *parent):QThread(parent),
    nh_(ros::this_node::getName()),it_(this->nh_){
    running=false;
    
    it_subs_ = it_.subscribe("/img_source_node/image_out",1,&FrameProcess::imgCallback,this);
    it_ball_detector_subs_ = it_.subscribe("/v9_ball_detector_node/image_out",1,&FrameProcess::imgOutCallback,this);
    
    frame_mode_subs_= nh_.subscribe("frame_mode", 1, &FrameProcess::frameModeCallback, this);

    //yellow goalpost
    //comment the ball detector if want to check goalpost, or make a new output of other object
    // it_goalpost_detector_subs_ = it_.subscribe("/v10_goalpost_detector_node/image_out",1,&FrameProcess::imgOutCallback,this);

//    it_pubs_ = it_.advertise("image_out",100);

    frame_rate_ = 30;
}

void FrameProcess::imgCallback(const sensor_msgs::ImageConstPtr &_msg){

    try{
        if(_msg->encoding.compare(sensor_msgs::image_encodings::MONO8))
            encoding_ = 0;
        else if(_msg->encoding.compare(sensor_msgs::image_encodings::BGR8))
            encoding_ = 1;
        else
            encoding_ = 0;
    }catch(cv_bridge::Exception &e){
        ROS_ERROR("[vision monitor] cv_bridge exception: %s", e.what());
    }

    cv_img_ptr_ = cv_bridge::toCvCopy(_msg,_msg->encoding);
}

void FrameProcess::imgOutCallback(const sensor_msgs::ImageConstPtr &_msg){

    try{
        if(_msg->encoding.compare(sensor_msgs::image_encodings::MONO8))
            encoding_ = 0;
        else if(_msg->encoding.compare(sensor_msgs::image_encodings::BGR8))
            encoding_ = 1;
        else
            encoding_ = 0;
    }catch(cv_bridge::Exception &e){
        ROS_ERROR("[vision monitor] cv_bridge exception: %s", e.what());
    }

    cv_out_img_ptr_ = cv_bridge::toCvCopy(_msg,_msg->encoding);
}

void FrameProcess::frameModeCallback(const std_msgs::Int8::ConstPtr &_msg){
    frame_mode_ = _msg->data;
}

FrameProcess::~FrameProcess(){
    mutex_.lock();
    running=false;
    //release capture
    wait_cond_.wakeOne();
    mutex_.unlock();
    this->wait();
}

void FrameProcess::run(){
    mutex_.lock();
    ros::Rate loop_rate(frame_rate_);
    while(ros::ok() && running){
        ros::spinOnce();

        if (frame_mode_ == 3)
        {
            cv_img_ptr_ = NULL;
            if(cv_out_img_ptr_ != NULL){
            frame_ = cv_out_img_ptr_->image;
//            cv::cvtColor(frame_,frame_,CV_BGR2RGB);

                QImage qimage = QImage(frame_.data,frame_.cols,frame_.rows, QImage::Format_RGB888);

                emit capturedFrame(qimage);
            }
        }
        else
        {
            if(cv_img_ptr_ != NULL){
            frame_ = cv_img_ptr_->image;
//            cv::cvtColor(frame_,frame_,CV_BGR2RGB);

                QImage qimage = QImage(frame_.data,frame_.cols,frame_.rows, QImage::Format_RGB888);

                emit capturedFrame(qimage);
            }
        }
        

//         if(cv_img_ptr_ != NULL){
//             frame_ = cv_img_ptr_->image;
// //            cv::cvtColor(frame_,frame_,CV_BGR2RGB);

//             QImage qimage = QImage(frame_.data,frame_.cols,frame_.rows, QImage::Format_RGB888);

//             emit capturedFrame(qimage);
//         }

//        this->msleep(1000/frame_rate_);
        loop_rate.sleep();
    }
    mutex_.unlock();
}

void FrameProcess::play(){
    running=true;
    if(!this->isRunning()){
        this->start(LowPriority);
    }
}

void FrameProcess::stop(){
    running=false;
}
