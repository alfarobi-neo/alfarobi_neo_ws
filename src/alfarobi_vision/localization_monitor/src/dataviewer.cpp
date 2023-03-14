#include "localization_monitor/dataviewer.h"

DataViewer::DataViewer(QObject *parent):
    nh_(ros::this_node::getName()),
    robot_state_sub_(nh_,"/v9_localization_node/robot_state",1),
    particles_state_sub_(nh_,"/v9_localization_node/particles_state",1),
    features_sub_(nh_,"/v9_localization_node/field_features",1),
    projected_ball_sub_(nh_,"/v9_localization_node/stamped_projected_ball",1),
    input_sync_(robot_state_sub_,particles_state_sub_,
                features_sub_,projected_ball_sub_,10),
    show_grid_(false),
    show_particles_(false){

    (void)parent;

    this->setFixedSize(BORDER_STRIP_WIDTH*2 + FIELD_LENGTH,BORDER_STRIP_WIDTH*2 + FIELD_WIDTH);
    std::stringstream field_bg_path;
#ifdef NASIONAL
    field_bg_path << ros::package::getPath("localization_monitor") << "/data/field_background.jpg";
#else
    field_bg_path << ros::package::getPath("localization_monitor") << "/data/field_background_regional.jpg";
#endif
    field_background_.load(field_bg_path.str().c_str());

    data_scene_ = new QGraphicsScene;
    data_scene_->addPixmap(std::move(QPixmap::fromImage(field_background_)));
    this->setScene(data_scene_);

    input_sync_.registerCallback(boost::bind(&DataViewer::inputUtilsCallback,this,_1,_2,_3,_4));

//    monitor_utils_sub_ = nh_.subscribe("monitor_utils",1,&DataViewer::monitorUtilsCallback,this);
//    robot_state_sub_ = nh_.subscribe("robot_state",1,&DataViewer::robotStateCallback,this);

    data_process_ = new DataProcess;

}

DataViewer::~DataViewer(){

}

void DataViewer::inputUtilsCallback(const geometry_msgs::PoseStampedConstPtr &_msg1,
                                    const vision_utils::ParticlesConstPtr &_msg2,
                                    const vision_utils::FeaturesConstPtr &_msg3,
                                    const geometry_msgs::PointStampedConstPtr &_msg4){
    robot_state_.x = _msg1->pose.position.x;
    robot_state_.y = _msg1->pose.position.y;
    robot_state_.theta = _msg1->pose.orientation.z;

    particles_state_.particle = _msg2->particle;

    features_.feature = _msg3->feature;

    projected_ball_.x = _msg4->point.x;
    projected_ball_.y = _msg4->point.y;
    projected_ball_.z = _msg4->point.z;

    emit updateViewer();
}

//void DataViewer::monitorUtilsCallback(const v9_localization::MonitorUtilsConstPtr &_msg){
//    monitor_utils_.line_models = _msg->line_models;
//    monitor_utils_.particles = _msg->particles;
//    emit updateViewer();
//}

//void DataViewer::robotStateCallback(const geometry_msgs::Pose2DConstPtr &_msg){
//    robot_state_.x = _msg->x;
//    robot_state_.y = _msg->y;
//    robot_state_.theta = _msg->theta;
//}

void DataViewer::updateData(){
    data_scene_->clear();
    data_scene_->addPixmap(std::move(QPixmap::fromImage(field_background_)));
    if(show_particles_)
        drawParticles();
    drawRobot();
    drawProjectedLines();
    drawProjectedBall();
    if(show_grid_)
        drawGrid();
    this->scene()->update();

}

void DataViewer::drawRobot(){
    if(robot_state_.x == 999.0 && robot_state_.y == 999.0){
        robot_state_.x = BORDER_STRIP_WIDTH>>1;
        robot_state_.y = BORDER_STRIP_WIDTH>>1;
        robot_state_.theta = .0;
    }

    double rad = robot_state_.theta * Math::DEG2RAD;
    constexpr int w = 20,l = 54;
    constexpr double alpha = atan(3.0*w/l);
    constexpr float constant = (float)sqrt((l*l)/9.0f + w*w);

    QPolygonF robot_poly;
    robot_poly << QPoint(robot_state_.x-constant*cos(alpha+rad),robot_state_.y-constant*sin(alpha+rad))
            << QPoint(robot_state_.x-constant*cos(alpha-rad),robot_state_.y+constant*sin(alpha-rad))
            << QPoint(robot_state_.x+(int)(2.0*l*cos(rad)/3.0),robot_state_.y+(int)(2.0*l*sin(rad)/3.0));
    QPen robot_pen(QColor(221,153,255), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    data_scene_->addPolygon(robot_poly,robot_pen);
}

void DataViewer::drawParticles(){
    QPen particle_body_pen(QColor(114,160,193),2,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
    QPen particle_head_pen(Qt::blue,2,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
    for(std::vector<geometry_msgs::Quaternion>::iterator it=particles_state_.particle.begin();
        it!=particles_state_.particle.end();it++){
        double rad = Math::DEG2RAD * it->z;
        data_scene_->addEllipse(it->x - 2,it->y - 2, 4, 4, particle_body_pen);
        data_scene_->addLine(it->x,it->y,it->x + 8.0*cos(rad), it->y + 8.0*sin(rad),particle_head_pen);
    }
}

void DataViewer::drawProjectedLines(){
    QPen feature_pen(Qt::yellow, 2, Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
    for(std::vector<vision_utils::Feature>::iterator it=features_.feature.begin();
        it != features_.feature.end(); it++){

//        std::cout << " TIP1 : " << it->param1 << " ; " << it->param2 << std::endl;
//        std::cout << " TIP2 : " << it->param3 << " ; " << it->param4 << std::endl;
        float robot_theta = robot_state_.theta*Math::DEG2RAD;
        float c_t = cos(robot_theta);
        float s_t = sin(robot_theta);
        if(it->feature_type == 4){

            data_scene_->addLine(robot_state_.x + it->param2*c_t - it->param1*s_t,
                                 robot_state_.y + it->param2*s_t + it->param1*c_t,
                                 robot_state_.x + it->param4*c_t - it->param3*s_t,
                                 robot_state_.y + it->param4*s_t + it->param3*c_t,
                                 feature_pen);
        }else if(it->feature_type == 3){
            qreal circle_rad =  it->param3;
            data_scene_->addEllipse(robot_state_.x - circle_rad + it->param2*c_t - it->param1*s_t,
                                    robot_state_.y - circle_rad + it->param2*s_t + it->param1*c_t,
//                                    150,150,
                                    circle_rad*2,circle_rad*2,
                                    feature_pen);
        }
    }
}

void DataViewer::drawProjectedBall(){
    if(projected_ball_.z > .0f){
        constexpr float ball_radius = 5.0f;
        float robot_theta = robot_state_.theta*Math::DEG2RAD;
        float c_t = cos(robot_theta);
        float s_t = sin(robot_theta);
        QPen ball_pen(QColor(255,69,0),3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        QPoint global_ball_pos(robot_state_.x + projected_ball_.y*c_t - projected_ball_.x*s_t,
                               robot_state_.y + projected_ball_.y*s_t + projected_ball_.x*c_t);
        data_scene_->addEllipse(global_ball_pos.x() - ball_radius,
                                global_ball_pos.y() - ball_radius,
                                ball_radius*2, ball_radius*2,ball_pen);
        float rad = projected_ball_.z*Math::DEG2RAD;
        QPoint shoot_dir_origin(global_ball_pos.x(), global_ball_pos.y());
        QPoint shoot_dir_tip(shoot_dir_origin.x() + 50*cos(rad),
                             shoot_dir_origin.y() + 50*sin(rad));
        QPen shoot_dir_pen(QColor(255,0,0), 5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        data_scene_->addLine(shoot_dir_origin.x(),
                             shoot_dir_origin.y(),
                             shoot_dir_tip.x(),
                             shoot_dir_tip.y(),
                             shoot_dir_pen);
        constexpr int w = 20,l = 20;
        constexpr double alpha = atan(3.0*w/l);
        constexpr float constant = (float)sqrt((l*l)/9.0f + w*w);
        QPolygonF shoot_dir_head;
        shoot_dir_head << QPoint(shoot_dir_tip.x() - constant*cos(alpha+rad), shoot_dir_tip.y() - constant*sin(alpha+rad))
                       << QPoint(shoot_dir_tip.x() - constant*cos(alpha-rad), shoot_dir_tip.y() + constant*sin(alpha-rad))
                       << QPoint(shoot_dir_tip.x() + (int)(2.0*l*cos(rad)/3.0), shoot_dir_tip.y() + (int)(2.0*l*sin(rad)/3.0));
        QBrush shoot_dir_brush;
        shoot_dir_brush.setStyle(Qt::SolidPattern);
        data_scene_->addPolygon(shoot_dir_head,shoot_dir_pen,shoot_dir_brush);
    }
}

void DataViewer::drawGrid(){
    // 9x6 Grid
    QSize grid_size(9,6);
    QPen line_pen(QColor(50,200,200));
    int length_step = FIELD_LENGTH/grid_size.width();
    int width_step = FIELD_WIDTH/grid_size.height();
    for(int i=1;i < grid_size.width() ;i++){
        data_scene_->addLine(BORDER_STRIP_WIDTH + length_step*i, BORDER_STRIP_WIDTH,
                             BORDER_STRIP_WIDTH + length_step*i, BORDER_STRIP_WIDTH + FIELD_WIDTH,
                             line_pen);
    }

    for(int i=1;i < grid_size.height();i++){
        data_scene_->addLine(BORDER_STRIP_WIDTH , BORDER_STRIP_WIDTH + width_step*i,
                             BORDER_STRIP_WIDTH + FIELD_LENGTH, BORDER_STRIP_WIDTH + width_step*i,
                             line_pen);
    }
}

void DataViewer::processBegin(){
    data_process_->begin();
}
