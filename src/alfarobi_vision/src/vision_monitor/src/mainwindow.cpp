#include "vision_monitor/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    nh_(ros::this_node::getName()),
    it_(nh_){

    ui->setupUi(this);

    central_layout = new QGridLayout;
    central_widget = new QWidget;

    settingWidgets();
    settingActions();
    loadBallConfig();
    //yellow goalpost
    // loadGoalpostConfig();

    central_widget->setLayout(central_layout);

    std::stringstream icon_path;
    icon_path << ros::package::getPath("vision_monitor") << "/icon/logo_alfarobi.png";
    QIcon icon_file;
    icon_file.addFile(icon_path.str().c_str());
    this->setWindowIcon(icon_file);

    std::stringstream style_path;
    style_path << ros::package::getPath("vision_monitor") << "/style/vision_style.qss";
    QFile style_file(style_path.str().c_str());
    style_file.open(QFile::ReadOnly);
    QString read_style(style_file.readAll());
    this->setStyleSheet(read_style);

    this->setCentralWidget(central_widget);
    this->removeToolBar(ui->mainToolBar);
    this->setWindowTitle(tr("Vision Monitor"));
    this->setMaximumSize(this->centralWidget()->width(),this->centralWidget()->height());

    LUT_pub_ = nh_.advertise<vision_utils::LUT>("LUT_data",10);
    frame_mode_pub_ = nh_.advertise<std_msgs::Int8>("frame_mode",10);
    save_param_pub_ = nh_.advertise<std_msgs::Empty>("save_param",10);

    it_ball_pub_ = it_.advertise("ball_ref",10);
    //yellow goalpost
    // it_goalpost_pub_ = it_.advertise("goalpost_ref",10);

    frame_viewer->startStream();
}

MainWindow::~MainWindow(){
    frame_viewer->stopStream();
    delete ui;
}

void MainWindow::settingWidgets(){
    frame_viewer = new FrameViewer;
    frame_viewer->setFixedSize(QSize(640,480));
    frame_viewer->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    frame_viewer->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    frame_viewer->verticalScrollBar()->blockSignals(true);
    frame_viewer->horizontalScrollBar()->blockSignals(true);

    frame_mode_gb = new QGroupBox;
    frame_mode_layout = new QVBoxLayout;
    frame_mode_rb1 = new QRadioButton;
    frame_mode_rb2 = new QRadioButton;
    frame_mode_rb3 = new QRadioButton;
    frame_mode_rb4 = new QRadioButton;

    frame_mode_rb1->setText(tr("Input"));
    frame_mode_rb1->setChecked(true);
    frame_mode_rb2->setText(tr("HSV"));
    frame_mode_rb3->setText(tr("Threshold"));
    frame_mode_rb4->setText(tr("Output"));
    frame_mode_layout->addWidget(frame_mode_rb1);
    frame_mode_layout->addWidget(frame_mode_rb2);
    frame_mode_layout->addWidget(frame_mode_rb3);
    frame_mode_layout->addWidget(frame_mode_rb4);
    frame_mode_layout->addStretch(1);
    frame_mode_gb->setTitle(tr("Frame Mode"));
    frame_mode_gb->setLayout(frame_mode_layout);
    frame_mode_gb->setFixedSize(frame_mode_gb->minimumSizeHint());

    save_param_pb = new QPushButton;
    save_param_pb->setText(tr("Save"));

    vision_mode_gb = new QGroupBox;
    vision_mode_layout = new QHBoxLayout;
    vision_mode_rb1 = new QRadioButton;
    vision_mode_rb2 = new QRadioButton;
    vision_mode_rb1->setText(tr("Uniform"));
    vision_mode_rb1->setChecked(true);
    vision_mode_rb2->setText(tr("Non-Uniform"));
    vision_mode_layout->addWidget(vision_mode_rb1);
    vision_mode_layout->addWidget(vision_mode_rb2);
    vision_mode_layout->addStretch(1);
    vision_mode_gb->setTitle(tr("Vision Mode"));
    vision_mode_gb->setLayout(vision_mode_layout);
    vision_mode_gb->setFixedSize(vision_mode_gb->minimumSizeHint());

    //param1
    lut_dir_label = new QLabel;
    lut_dir_label->setText(tr("LUT Directory"));

    lut_dir_display = new QLineEdit;
    lut_dir_display->setText(tr("/home/koseng/LUT.xml"));
    lut_dir_display->setEnabled(false);
//    lut_dir_display->setFixedSize(lut_dir_display->minimumSizeHint());

    lut_dir_browse = new QPushButton;
    lut_dir_browse->setText(tr("Browse"));
    lut_dir_browse->setFixedSize(lut_dir_browse->minimumSizeHint());

    score_slider_ = new QSlider;
    score_slider_->setMaximum(2000);
    score_slider_->setMinimum(0);
    score_slider_->setTickInterval(2000);
    score_slider_->setSingleStep(1);
    score_slider_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    score_slider_->setOrientation(Qt::Horizontal);
    score_slider_->setValue(20);//Default value

    score_label_ = new QLabel;
    score_label_->setText(tr("Score : %1").arg((float)score_slider_->value()/10.0f));

    cost_slider_ = new QSlider;
    cost_slider_->setMaximum(1000);
    cost_slider_->setMinimum(0);
    cost_slider_->setTickInterval(100);
    cost_slider_->setSingleStep(1);
    cost_slider_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    cost_slider_->setOrientation(Qt::Horizontal);
    cost_slider_->setValue(20);//Default value

    cost_label_ = new QLabel;
    cost_label_->setText(tr("Cost : %1").arg(cost_slider_->value()));

    //yellow goalpost
    // canny1_slider_ = new QSlider;
    // canny1_slider_->setMaximum(255);
    // canny1_slider_->setMinimum(0);
    // canny1_slider_->setTickInterval(255);
    // canny1_slider_->setSingleStep(1);
    // canny1_slider_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    // canny1_slider_->setOrientation(Qt::Horizontal);
    // canny1_slider_->setValue(20);//Default value

    // canny1_label_ = new QLabel;
    // canny1_label_->setText(tr("Canny 1 : %1").arg(canny1_slider_->value()));

    // canny2_slider_ = new QSlider;
    // canny2_slider_->setMaximum(255);
    // canny2_slider_->setMinimum(0);
    // canny2_slider_->setTickInterval(255);
    // canny2_slider_->setSingleStep(1);
    // canny2_slider_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    // canny2_slider_->setOrientation(Qt::Horizontal);
    // canny2_slider_->setValue(20);//Default value

    // canny2_label_ = new QLabel;
    // canny2_label_->setText(tr("Canny 2 : %1").arg(canny2_slider_->value()));

    // intersections_slider_ = new QSlider;
    // intersections_slider_->setMaximum(255);
    // intersections_slider_->setMinimum(0);
    // intersections_slider_->setTickInterval(255);
    // intersections_slider_->setSingleStep(1);
    // intersections_slider_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    // intersections_slider_->setOrientation(Qt::Horizontal);
    // intersections_slider_->setValue(20);//Default value

    // intersections_label_ = new QLabel;
    // intersections_label_->setText(tr("Line : %1").arg(intersections_slider_->value()));


    // threshold_corner_slider_ = new QSlider;
    // threshold_corner_slider_->setMaximum(255);
    // threshold_corner_slider_->setMinimum(0);
    // threshold_corner_slider_->setTickInterval(255);
    // threshold_corner_slider_->setSingleStep(1);
    // threshold_corner_slider_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    // threshold_corner_slider_->setOrientation(Qt::Horizontal);
    // threshold_corner_slider_->setValue(20);//Default value

    // threshold_corner_label_ = new QLabel;
    // threshold_corner_label_->setText(tr("Corner : %1").arg(threshold_corner_slider_->value()));

    // ----

    param1_layout = new QGridLayout;
    param1_layout->addWidget(lut_dir_label,  0,0,1,1);
    param1_layout->addWidget(lut_dir_display,1,0,1,1);
    param1_layout->addWidget(lut_dir_browse, 1,1,1,1);
    param1_layout->addWidget(score_label_,   2,0,1,1);
    param1_layout->addWidget(score_slider_,  3,0,1,2);
    param1_layout->addWidget(cost_label_,    4,0,1,1);
    param1_layout->addWidget(cost_slider_,   5,0,1,2);
  
    //yellow goalpost
    // param1_layout->addWidget(canny1_label_,   6,0,1,1);
    // param1_layout->addWidget(canny1_slider_,  7,0,1,2);
    // param1_layout->addWidget(canny2_label_,    8,0,1,1);
    // param1_layout->addWidget(canny2_slider_,   9,0,1,2);
    // param1_layout->addWidget(intersections_label_,    10,0,1,1);
    // param1_layout->addWidget(intersections_slider_,  11,0,1,2);    
    // param1_layout->addWidget(threshold_corner_label_,    12,0,1,1);
    // param1_layout->addWidget(threshold_corner_slider_,   13,0,1,2);
    //---

    param1_layout->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),6,2);

    param1_widget = new QWidget;
    param1_widget->setLayout(param1_layout);

    param_tab = new QTabWidget;
    param_tab->addTab(param1_widget,"Uniform");
    param_tab->addTab(new QWidget(),"Non-Uniform");
    param_tab->setFixedSize(param_tab->minimumSizeHint());

    central_layout->addWidget(frame_viewer,  0,0,4,1);
    central_layout->addWidget(frame_mode_gb, 0,1,1,1);
    central_layout->addWidget(save_param_pb, 1,1,1,1);
    central_layout->addWidget(param_tab,     0,2,2,1);
    central_layout->addWidget(vision_mode_gb,4,0,1,1);
    central_layout->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding, QSizePolicy::Expanding),5,3);

//    central_layout->setColumnStretch(0,1);
//    central_layout->setRowStretch(0,500);
}

void MainWindow::settingActions(){
    connect(frame_mode_rb1,SIGNAL(clicked(bool)),this,SLOT(frameModeChange()));
    connect(frame_mode_rb2,SIGNAL(clicked(bool)),this,SLOT(frameModeChange()));
    connect(frame_mode_rb3,SIGNAL(clicked(bool)),this,SLOT(frameModeChange()));
    connect(frame_mode_rb4,SIGNAL(clicked(bool)),this,SLOT(frameModeChange()));
    connect(frame_viewer,SIGNAL(getROI(QImage)),this,SLOT(frameROI(QImage)));
    connect(frame_viewer,SIGNAL(colorClass(int)),this,SLOT(getColorClass(int)));
    connect(frame_viewer,SIGNAL(ballReference(QImage)),this,SLOT(getBallReference(QImage)));
    //yellow goalpost
    // connect(frame_viewer,SIGNAL(goalpostReference(QImage)),this,SLOT(getGoalpostReference(QImage)));
    
    connect(lut_dir_browse,SIGNAL(clicked(bool)),this,SLOT(browseLUT()));
    connect(save_param_pb,SIGNAL(clicked(bool)),this,SLOT(saveParam()));
    connect(score_slider_,SIGNAL(valueChanged(int)),this,SLOT(updateParam1Value(int)));
    connect(cost_slider_,SIGNAL(valueChanged(int)),this,SLOT(updateParam1Value(int)));
    //yellow goalpost
    // connect(canny1_slider_,SIGNAL(valueChanged(int)),this,SLOT(updateParam2Value(int)));
    // connect(canny2_slider_,SIGNAL(valueChanged(int)),this,SLOT(updateParam2Value(int)));
    // connect(threshold_corner_slider_,SIGNAL(valueChanged(int)),this,SLOT(updateParam2Value(int)));
    // connect(intersections_slider_,SIGNAL(valueChanged(int)),this,SLOT(updateParam2Value(int)));
    //---
}

void MainWindow::saveParam(){
    QMessageBox save_msg_box;
    save_msg_box.setWindowTitle(tr("Info"));
    save_msg_box.setText(tr("Save ?"));
    save_msg_box.setStandardButtons(QMessageBox::Save | QMessageBox::Cancel);
    save_msg_box.setDefaultButton(QMessageBox::Save);
    int ret = save_msg_box.exec();
    if(ret == QMessageBox::Save){
        std_msgs::Empty msg;
        save_param_pub_.publish(msg);
//        save_param_msg_.data = true;
//        save_param_pub_.publish(save_param_msg_);
    }
}

void MainWindow::browseLUT(){
    QString LUT_path = QFileDialog::getOpenFileName(this,"Search LUT file...",ros::package::getPath("img_source").c_str(),"XML File(*.xml)");
    if(LUT_path.isEmpty())return;
    lut_dir_display->setText(LUT_path);
    cv::FileStorage fs(LUT_path.toStdString().c_str(),cv::FileStorage::READ);
    cv::Mat loaded_LUT;    
    fs["Tabel_Warna"] >> loaded_LUT;    
    fs.release();
    LUT_msg_.color.resize(loaded_LUT.cols*loaded_LUT.rows);
    for(int i=0;i<loaded_LUT.rows;i++){
        uchar *loaded_LUT_ptr = loaded_LUT.ptr<uchar>(i);
        for(int j=0;j<loaded_LUT.cols;j++){
            int curr_idx = j + i*loaded_LUT.cols;
            LUT_msg_.color[curr_idx].x = i;
            LUT_msg_.color[curr_idx].y = j;
            LUT_msg_.color[curr_idx].z = loaded_LUT_ptr[j];
        }
    }
    LUT_pub_.publish(LUT_msg_);
    LUT_msg_.color.clear();
}

void MainWindow::frameModeChange(){
    QObject *_sender = sender();
    if(_sender == frame_mode_rb1)
        frame_mode_msg_.data = 0;
    else if(_sender == frame_mode_rb2)
        frame_mode_msg_.data = 1;
    else if(_sender == frame_mode_rb3)
        frame_mode_msg_.data = 2;
    else if(_sender == frame_mode_rb4)
        frame_mode_msg_.data = 3;
    else
        frame_mode_msg_.data = 0;

    frame_mode_pub_.publish(frame_mode_msg_);
}

void MainWindow::getColorClass(int idx){
    LUT_msg_.color_class.data = idx;
}

void MainWindow::frameROI(QImage _img){
    LUT_msg_.color.clear();
    uchar* data_buffer = _img.bits();
    int LUT_size = _img.width()*_img.height();
    LUT_msg_.color.resize(LUT_size);
    cv::Mat frame_roi(_img.height(),_img.width(),CV_8UC3,data_buffer,_img.bytesPerLine());
    cv::cvtColor(frame_roi,frame_roi,CV_BGR2HSV);
    for(int i=0;i<frame_roi.rows;i++){
        for(int j=0;j<frame_roi.cols;j++){
            int curr_idx = j + i*frame_roi.cols;
            LUT_msg_.color[curr_idx].x = frame_roi.at<cv::Vec3b > (i,j)[0];
            LUT_msg_.color[curr_idx].y = frame_roi.at<cv::Vec3b > (i,j)[1];

        }
    }
    LUT_pub_.publish(LUT_msg_);
//    cv::imshow("TEST",frame_roi);
//    cv::waitKey(0);
//    cv::destroyWindow("TEST");
}

void MainWindow::getBallReference(QImage _img){
    cv::Mat ball_ref(_img.height(), _img.width(), CV_8UC3, _img.bits(), _img.bytesPerLine());
//    cv::cvtColor(ball_ref,ball_ref,CV_RGB2HSV);
    cv::cvtColor(ball_ref,ball_ref,CV_BGR2RGB);
    // cv::cvtColor(ball_ref,ball_ref,CV_RGB2BGR);
    cv_ball_ref_pub_.image = ball_ref;
    cv_ball_ref_pub_.encoding = sensor_msgs::image_encodings::BGR8;
    it_ball_pub_.publish(cv_ball_ref_pub_.toImageMsg());
}

//yellow goalpost
// void MainWindow::getGoalpostReference(QImage _img){
//     cv::Mat goalpost_ref(_img.height(), _img.width(), CV_8UC3, _img.bits(), _img.bytesPerLine());
//     cv::cvtColor(goalpost_ref,goalpost_ref,CV_BGR2RGB);
//     cv_goalpost_ref_pub_.image = goalpost_ref;
//     cv_goalpost_ref_pub_.encoding = sensor_msgs::image_encodings::BGR8;
//     it_goalpost_pub_.publish(cv_goalpost_ref_pub_.toImageMsg());
// }

void MainWindow::updateParam1Value(int _value){
    QObject *object = sender();

    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::Config conf;

    if(object == score_slider_){
        score_label_->setText(tr("Score : %1").arg((float)_value/10.0f));
        int_param.name = "score";
        int_param.value = _value;
    }else if(object == cost_slider_){
        cost_label_->setText(tr("Cost : %1").arg(_value));
        int_param.name = "cost";
        int_param.value = _value;

    }
    conf.ints.push_back(int_param);
    srv_ball_req_.config = conf;
    ros::service::call("/v9_ball_detector_node/set_parameters",srv_ball_req_,srv_ball_resp_);
}

//yellow goalpost
// void MainWindow::updateParam2Value(int _value){
//     QObject *object = sender();

//     dynamic_reconfigure::IntParameter int_param;
//     dynamic_reconfigure::Config conf;

//     if(object == canny1_slider_){
//         canny1_label_->setText(tr("Canny 1 : %1").arg(_value));
//         int_param.name = "canny1";
//         int_param.value = _value;
//     }else if(object == canny2_slider_){
//         canny2_label_->setText(tr("Canny 2 : %1").arg(_value));
//         int_param.name = "canny2";
//         int_param.value = _value;
//     }else if(object == threshold_corner_slider_){
//         threshold_corner_label_->setText(tr("Corner : %1").arg(_value));
//         int_param.name = "threshold_corner";
//         int_param.value = _value;
//     }else if(object == intersections_slider_){
//         intersections_label_->setText(tr("Line : %1").arg(_value));
//         int_param.name = "intersections";
//         int_param.value = _value;
//     }
//     conf.ints.push_back(int_param);
//     srv_goalpost_req_.config = conf;
//     ros::service::call("/v10_goalpost_detector_node/set_parameters",srv_goalpost_req_,srv_goalpost_resp_);
// }

//----

void MainWindow::loadBallConfig(){
    YAML::Node config_file;
    try{
        config_file = YAML::LoadFile(ros::package::getPath("v9_ball_detector") + "/config/saved_config.yaml");
    }catch(const std::exception &e){
        ROS_ERROR("[vision_monitor] Problem while opening config file !!!");
    }

    score_slider_->setValue(config_file["score"].as<int>());
    cost_slider_->setValue(config_file["cost"].as<int>());

}

//yellow goalpost
// void MainWindow::loadGoalpostConfig(){
//     YAML::Node config_file;
//     try{
//         config_file = YAML::LoadFile(ros::package::getPath("v10_goalpost_detector") + "/config/saved_config.yaml");
//     }catch(const std::exception &e){
//         ROS_ERROR("[vision_monitor] Problem while opening config file !!!");
//     }

//     canny1_slider_->setValue(config_file["canny1"].as<int>());
//     canny2_slider_->setValue(config_file["canny2"].as<int>());
//     intersections_slider_->setValue(config_file["intersections"].as<int>());
//     threshold_corner_slider_->setValue(config_file["threshold_corner"].as<int>());
// }
//----