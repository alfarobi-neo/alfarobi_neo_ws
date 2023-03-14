#include "vision_monitor/frameviewer.h"

FrameViewer::FrameViewer(QObject *parent){
    frame_scene_ = new QGraphicsScene;

    frame_image_ = QPixmap(640,480).toImage();
    frame_image_.fill(QColor(0,0,0));

    frame_scene_->addPixmap(std::move(QPixmap::fromImage(frame_image_)));
    this->setScene(frame_scene_);

    color_class_dialog = new QDialog;
    color_class_layout = new QHBoxLayout;
    color_class_cb = new QComboBox;
    color_class_pb = new QPushButton;

    color_class_cb->addItem(tr("None"));
    color_class_cb->addItem(tr("Field"));
    color_class_cb->addItem(tr("Ball"));
    color_class_cb->addItem(tr("Line"));
    color_class_cb->addItem(tr("Ball Ref.")); 

    //yellow goalpost
    // color_class_cb->addItem(tr("Goalpost"));
    // color_class_cb->addItem(tr("Background"));
    // color_class_cb->addItem(tr("Goalpost Ref."));
    
    color_class_cb->setFixedSize(color_class_cb->minimumSizeHint());

    color_class_pb->setText(tr("Set"));
    color_class_pb->setFixedSize(color_class_pb->minimumSizeHint());

    color_class_layout->addWidget(color_class_cb);
    color_class_layout->addWidget(color_class_pb);
    color_class_layout->addStretch(1);

    color_class_dialog->setWindowTitle(tr("Color Class"));
    color_class_dialog->setLayout(color_class_layout);

    frame_process_ = new FrameProcess;
    connect(frame_process_,SIGNAL(capturedFrame(QImage)),this,SLOT(updateFrame(QImage)));
    connect(color_class_pb,SIGNAL(clicked(bool)),color_class_dialog,SLOT(accept()));
    connect(color_class_dialog,SIGNAL(accepted()),this,SLOT(colorClassAccept()));
    connect(color_class_dialog,SIGNAL(rejected()),this,SLOT(colorClassReject()));
}

FrameViewer::~FrameViewer(){

}

void FrameViewer::mouseMoveEvent(QMouseEvent *event){
    bool cond1 = ref_point.x() < event->pos().x();
    bool cond2 = ref_point.y() < event->pos().y();
    // frame_scene_->addPixmap(QPixmap::fromImage(frame_image_.rgbSwapped()));
    frame_scene_->addPixmap(std::move(QPixmap::fromImage(frame_image_)));
    frame_scene_->addRect(cond1?ref_point.x():event->pos().x(),cond2?ref_point.y():event->pos().y(),
                          abs(event->pos().x()-ref_point.x()),abs(event->pos().y()-ref_point.y()),QPen(QColor(0,255,50)));
    this->scene()->update();
    frame_process_->stop();
}

void FrameViewer::mousePressEvent(QMouseEvent *event){
    ref_point = event->pos();
}

void FrameViewer::mouseReleaseEvent(QMouseEvent *event){
    bool cond1 = ref_point.x() < event->pos().x();
    bool cond2 = ref_point.y() < event->pos().y();

    // frame_scene_->addPixmap(QPixmap::fromImage(frame_image_.rgbSwapped()));
    frame_scene_->addPixmap(std::move(QPixmap::fromImage(frame_image_)));
    QRect roi(cond1?ref_point.x():event->pos().x(), cond2?ref_point.y():event->pos().y(),
              abs(event->pos().x()-ref_point.x()), abs(event->pos().y()-ref_point.y()));
    frame_scene_->addRect(roi, QPen(QColor(255,0,255)));

    color_class_dialog->show();
    frame_roi = frame_image_.copy(roi);

    this->scene()->update();

}

void FrameViewer::updateFrame(QImage _img){
//    frame_image_ = _img.rgbSwapped().scaled(640,480);
    // frame_image_ = std::move(_img).rgbSwapped();
    frame_image_ = _img;
    frame_scene_->clear();
    // frame_scene_->addPixmap(QPixmap::fromImage(frame_image_.rgbSwapped()));
    frame_scene_->addPixmap(std::move(QPixmap::fromImage(frame_image_)));
    this->scene()->update();
}



void FrameViewer::colorClassAccept(){
    if(color_class_cb->currentIndex() == 4){
        emit ballReference(frame_roi);
    } 
    //yellow goalpost
    // else if(color_class_cb->currentIndex() == 7){
    //     emit goalpostReference(frame_roi);
    // }
    else{
        emit colorClass(color_class_cb->currentIndex());
        emit getROI(frame_roi);
    }
    colorClassReject();
}

void FrameViewer::colorClassReject(){
    color_class_dialog->hide();
    frame_process_->play();
}

void FrameViewer::startStream(){
    frame_process_->play();
}

void FrameViewer::stopStream(){
    frame_process_->stop();
}

