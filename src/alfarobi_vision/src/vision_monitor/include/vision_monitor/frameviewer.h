#ifndef FRAMEVIEWER_H
#define FRAMEVIEWER_H

#include <QtCore>
#include <QtGui>
#include <QtWidgets>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <utility>

#include "vision_monitor/frameprocess.h"

class FrameViewer:public QGraphicsView{
    Q_OBJECT
private:
    QGraphicsScene *frame_scene_;
    QImage frame_image_;
    QImage frame_roi;

    FrameProcess *frame_process_;

    QDialog *color_class_dialog;
    QHBoxLayout *color_class_layout;
    QComboBox *color_class_cb;
    QPushButton *color_class_pb;

    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

    QPoint ref_point;
    QPoint target_point;

public:
    FrameViewer(QObject *parent =0);
    ~FrameViewer();

    void startStream();
    void stopStream();

private slots:
    void updateFrame(QImage _img);
    void colorClassAccept();
    void colorClassReject();
signals:
    void getROI(const QImage &_img);
    void colorClass(int idx);
    void ballReference(const QImage &_img);
    //yellow goalpost
    // void goalpostReference(const QImage &_img);
};

#endif // FRAMEVIEWER_H
