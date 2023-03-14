#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "vision_utils/FieldBoundary.h"
#include "vision_utils/vision_common.h"

class LocalizationUtils{
public:
    LocalizationUtils();
    ~LocalizationUtils();

    static LocalizationUtils* getInstance(){
        if(!instance)
            instance = new LocalizationUtils;
        return instance;
    }

    void scanLinePoints(cv::Mat _invert_green, cv::Mat _segmented_white , const vision_utils::FieldBoundary &_field_boundary,
              Points &_target_points, int _orientation);
    //overload scanLinePoints function
    void scanLinePoints(const cv::Mat &_invert_green, const cv::Mat &_segmented_white , const vision_utils::FieldBoundary &_field_boundary,
              Points &_target_points);
    void adaptiveScanLinePoints(cv::Mat _invert_green, cv::Mat _segmented_white,
                                float _tilt, float _pan,
                                const vision_utils::FieldBoundary &_field_boundary,
                                Points &_target_points);
    void RANSAC(Points &_data_points, geometry_msgs::Vector3 &_line_model,int _n, int _k, float _t, int _d, Points &_inliers);

//    void RANSCCircle(Points &_data_points, geometry_msgs::Vector3 &_line_model,int _n, int _k, float _t, int _d,Points &_inliers);
private:
    static LocalizationUtils *instance;

    inline int stepFunc(float _idx){
        return (int)(1.226446e-7f*_idx*_idx + .0249607537f*_idx + 4.0f);
    }

    inline float followLineByX(float a, float b, float x){
        return a*x + b;
    }
    inline int followLineByY(int a, int b, int y){
        return (int)((float)(y - b))/((float)a + 1e-6f);
    }

};
