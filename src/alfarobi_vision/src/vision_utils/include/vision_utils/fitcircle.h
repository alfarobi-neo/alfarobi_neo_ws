#pragma once

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#include "vision_utils/vision_common.h"

class FitCircle{
private:
    static FitCircle *instance;
    cv::Vec2d prattCharEq(float _eta, float _c2,float _c1,float _c0);
    float kasaCostFunc(const Points &_data, cv::Vec3f _circle);
public:
    FitCircle();
    ~FitCircle();
    cv::Vec4f newtonPrattMethod(const Points &_data, int _max_steps, double _epsilon);
    cv::Vec4f kasaMethod(const Points &_data);
    void sampleMean(const Points &_data, float  &_x_bar, float &_xx_bar,
                    float &_y_bar, float &_yy_bar,float &_xy_bar,
                    float &_xz_bar, float &_yz_bar, float &_z_bar, float &_zz_bar,
                    float &_x_avg,float &_y_avg, cv::Mat &_X);
    static FitCircle *getInstance(){
        if(!instance){
            instance = new FitCircle;
        }
        return instance;
    }
};
