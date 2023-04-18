#include "vision_utils/localization_utils.h"

LocalizationUtils *LocalizationUtils::instance = 0;

LocalizationUtils::LocalizationUtils(){

}

LocalizationUtils::~LocalizationUtils(){

}

// Unfortunately it must pass by value
void LocalizationUtils::scanLinePoints(cv::Mat _invert_green, cv::Mat _segmented_white, const alfarobi_msgs_srvs_actions::FieldBoundary &_field_boundary,
                             Points &_target_points, int _orientation){

    if(_orientation){
        transpose(_invert_green, _invert_green);
        transpose(_segmented_white, _segmented_white);
    }
    
    Vectors3 boundary = (_orientation==0) ? _field_boundary.bound1 : _field_boundary.bound2;
    for(size_t i = 0; i < boundary.size();
        i += (_orientation==1) ? stepFunc(i) : 10){
        cv::Point target(boundary[i].x,-1);
        int start = boundary[i].y;
        while(_invert_green.at<uchar>(start, target.x) > 0 && start < boundary[i].z)
            start++;                       

        for(int j=start;j<boundary[i].z;j++){         
            if(_invert_green.at<uchar>(j, target.x) > 0 && target.y == -1){
                target.y = j;
            }else if(_invert_green.at<uchar>(j, target.x) == 0 && target.y != -1){
                int diff = j-target.y;
                if(diff < 50){
                    target.y = (target.y + (j-1))/2;
                    if(_segmented_white.at<uchar>(target.y, target.x))
                        _target_points.emplace_back(_orientation==1 ? cv::Point(target.y, target.x) : target);
                }
                target.y = -1;
            }
        }
    }
//    cv::imshow("DEBUG",debug);
//    cv::waitKey(0);
}

void LocalizationUtils::scanLinePoints(const cv::Mat &_invert_green, const cv::Mat &_segmented_white, const alfarobi_msgs_srvs_actions::FieldBoundary &_field_boundary,
                             Points &_target_points){

    scanLinePoints(_invert_green, _segmented_white, _field_boundary, _target_points, 0);
    scanLinePoints(_invert_green, _segmented_white, _field_boundary, _target_points, 1);

}

void LocalizationUtils::adaptiveScanLinePoints(cv::Mat _invert_green, cv::Mat _segmented_white,
                                               float _tilt, float _pan,
                                               const alfarobi_msgs_srvs_actions::FieldBoundary &_field_boundary,
                                               Points &_target_points){
    int height = _invert_green.rows;
    int width = _invert_green.cols;

    float default_step = 5.0f;
    float tilt_const = 10.0f;
    float pan_const = 4.0f;

    float l_num_step = default_step*(exp(-tilt_const*_tilt) + exp(-pan_const*_pan));
    float r_num_step = default_step*(exp(-tilt_const*_tilt) + exp(pan_const*_pan));

    float l_angle_step = (Math::PI_TWO)/l_num_step;
    float r_angle_step = (Math::PI_TWO)/r_num_step;

    cv::Mat cek = cv::Mat::zeros(_invert_green.size(), CV_8UC1);
    for(int i=0;i<l_num_step;i++){
        float search_angle = l_angle_step * i;
        float search_grad = tan(search_angle);
        float search_bias = 0;
        int last_y = 0;
        Points search_pattern;
        for(int x = 0;x < width && last_y < height; x++){
            int y = followLineByX(search_grad,search_bias,x);
            for(int j=last_y;j <= y && j < height && y >= 0;j++){
                cek.at<uchar>(j,x) = 255;
                search_pattern.push_back(cv::Point(x,j));
            }
            last_y = y;
        }
        cv::Point target(-1,-1);
        for(size_t idx=0;idx<search_pattern.size();idx++){
            int x = search_pattern[idx].x;
            int y = search_pattern[idx].y;
            if(_invert_green.at<uchar>(y, x) > 0 && target.y == -1){
                target.x = x;
                target.y = y;
            }else if(_invert_green.at<uchar>(y, x) == 0 && target.y != -1){
                int diff_y = y - target.y;
                int diff_x = x - target.x;
                if(diff_x < 100 && diff_y < 100){
                    target.x = (target.x + (x-1))/2;
                    target.y = (target.y + (y-1))/2;
                    if(_segmented_white.at<uchar>(target.y,target.x))
                        _target_points.push_back(target);
                }
                target.y = -1;
            }
        }
    }

    for(int i=0;i<r_num_step;i++){
        float search_angle = r_angle_step * i;
        float search_grad = tan(search_angle);
        float search_bias = 0;
        int last_y=0;
        Points search_pattern;
        for(int x = (width-1);x >= 0 && last_y < height; x--){
            int y = followLineByX(search_grad,search_bias,(width-x));
            for(int j=last_y;j <= y && j < height && y >= 0;j++){
                cek.at<uchar>(j,x) = 255;
                search_pattern.push_back(cv::Point(x,j));
            }
            last_y = y;
        }
        cv::Point target(-1,-1);
        for(size_t idx=0;idx<search_pattern.size();idx++){
            int x = search_pattern[idx].x;
            int y = search_pattern[idx].y;
            if(_invert_green.at<uchar>(y, x) > 0 && target.y == -1){
                target.x = x;
                target.y = y;
            }else if(_invert_green.at<uchar>(y, x) == 0 && target.y != -1){
                int diff_y = y - target.y;
                int diff_x = x - target.x;
                if(diff_x < 100 && diff_y < 100){
                    target.x = (target.x + (x-1))/2;
                    target.y = (target.y + (y-1))/2;
                    if(_segmented_white.at<uchar>(target.y,target.x))
                        _target_points.push_back(target);
                }
                target.y = -1;
            }
        }
    }
//    cv::imshow("CEKAJ",cek);
}

/*
 n - number of samples
 k - max. iteration
 t - error threshold as inliers
 d - num. inliers requirement as best model
 */

void LocalizationUtils::RANSAC(Points &_data_points, geometry_msgs::Vector3 &_line_model,
                               int _n, int _k, float _t, int _d,
                               Points &_inliers){

    cv::Mat A,b,x,r;
    cv::Mat temp1,temp2,temp3,temp4,temp5,temp6;
    Points best_inliers;
    std::vector<int > best_inliers_idx;
    cv::Mat ones_N2 = cv::Mat::ones(_n, 2, CV_32FC1);
    cv::Mat zeros_N1 = cv::Mat::zeros(_n, 1, CV_32FC1);
    cv::Vec2f temp_model;
    cv::Vec2f best_model;

    int data_sz = _data_points.size();
    float opt_error = std::numeric_limits<float>::max();
    int opt_inliers = 0;

    for(int i = 0; i < _k; i++){
        A = ones_N2.clone();
        b = zeros_N1.clone();
        for(int j = 0; j < _n; j++){
            int data_idx = rand()%data_sz;
            A.at<float > (2*j+1) = _data_points[data_idx].x;
            b.at<float > (j) = _data_points[data_idx].y;
        }

        temp1 = A.t();
        temp2 = temp1*A;
        temp3 = temp1*b;
        temp4 = temp2.inv();

        x = temp4*temp3;
        temp_model[0] = x.at<float>(0);
        temp_model[1] = x.at<float>(1);

        Points inliers;
        std::vector<int > inliers_idx;

        for(int j = 0; j < data_sz; j++){
            //y = ax + b -> ax - y + b = 0
            //fabs(_data_points[j].y - (temp_model[0] + temp_model[1]*_data_points[j].x));
            float err = fabs(temp_model[1]*_data_points[j].x - _data_points[j].y + temp_model[0])/sqrt(temp_model[1]*temp_model[1] + 1);

            if(err < _t){
                inliers.push_back(_data_points[j]);
                inliers_idx.push_back(j);
            }
        }

        int inliers_sz = inliers.size();

        if(inliers_sz > _d && inliers_sz > opt_inliers){

            A = cv::Mat::ones(inliers_sz,2,CV_32FC1);
            b = cv::Mat::zeros(inliers_sz,1,CV_32FC1);

            for(int j=0;j<inliers_sz;j++){
                A.at<float > (2*j+1) = inliers[j].x;
                b.at<float > (j) = inliers[j].y;
            }

            temp1 = A.t();
            temp2 = temp1*A;
            temp3 = temp1*b;
            temp4 = temp2.inv();

            x = temp4*temp3;
            r = (A*x);
            r = b - r;
            temp5 = r.t();
            temp6 = temp5*r;
            float err = temp6.at<float > (0);
            if(err < opt_error){
                cv::Vec2f temp7;
                temp7[0] = x.at<float > (0);
                temp7[1] = x.at<float > (1);
                best_model=temp7;
                opt_error=err;
                opt_inliers=inliers_sz;
                best_inliers = inliers;
                best_inliers_idx = inliers_idx;
            }
        }
    }

    Points selected_inliers;
    std::vector<int > selected_inliers_idx;
    Points temp_inliers;
    std::vector<int > temp_inliers_idx;
    int best_inliers_sz = (int)best_inliers.size() - 1;
    if((best_inliers_sz+1)){
        temp_inliers.push_back(best_inliers.front());
        temp_inliers_idx.push_back(best_inliers_idx.front());
    }

    for(int i = 0; i < best_inliers_sz; i++){
        if(abs(best_inliers[i].x - best_inliers[i+1].x) < 25 &&
           abs(best_inliers[i].y - best_inliers[i+1].y) < 25){
            temp_inliers.push_back(best_inliers[i+1]);
            temp_inliers_idx.push_back(best_inliers_idx[i+1]);
        }else{
            if(temp_inliers.size() > selected_inliers.size()){
                selected_inliers = temp_inliers;
                selected_inliers_idx = temp_inliers_idx;
            }
            temp_inliers.resize(0);
            temp_inliers_idx.resize(0);
        }
    }

    if(temp_inliers.size() > selected_inliers.size()){
        selected_inliers = temp_inliers;
        selected_inliers_idx = temp_inliers_idx;
    }

    for(size_t i = 0; i < selected_inliers_idx.size(); i++){
        _data_points.erase(_data_points.begin() + selected_inliers_idx[i] - i);
    }

//    for(size_t i=0;i<best_inliers_idx.size();i++){
//        _data_points.erase(_data_points.begin() + best_inliers_idx[i] - i);
//    }

    _inliers = selected_inliers;
    _line_model.x = best_model[0];
    _line_model.y = best_model[1];

}
