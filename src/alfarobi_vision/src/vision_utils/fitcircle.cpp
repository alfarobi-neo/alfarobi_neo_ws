#include "vision_utils/fitcircle.h"

FitCircle *FitCircle::instance = 0;

FitCircle::FitCircle(){

}

FitCircle::~FitCircle(){

}

void FitCircle::sampleMean(const Points &_data, float  &_x_bar, float &_xx_bar,
                float &_y_bar, float &_yy_bar,float &_xy_bar,
                float &_xz_bar, float &_yz_bar, float &_z_bar, float &_zz_bar,
                float &_x_avg,float &_y_avg, cv::Mat &_X){
    int data_size = (int)_data.size();

    _X = cv::Mat(data_size, 4, CV_32FC1);
    _x_avg=.0f;
    _y_avg=.0f;
    for(int i=0;i<data_size;i++){
        _x_avg += _data[i].x;
        _y_avg += _data[i].y;
    }

    _x_avg /= data_size;
    _y_avg /= data_size;

    for(int i=0;i<data_size;i++){
        cv::Point temp(_data[i].x - _x_avg, _data[i].y - _y_avg);
        float xx = temp.x*temp.x;
        float yy = temp.y*temp.y;
        _x_bar += temp.x;
        _xx_bar += xx;
        _y_bar += temp.y;
        _yy_bar += temp.y*temp.y;
        _xy_bar += temp.x*temp.y;
        float z = xx + yy;
        _z_bar += z;
        _zz_bar += z*z;
        _xz_bar += z*temp.x;
        _yz_bar += z*temp.y;

        float *X_ptr = _X.ptr<float>(i);
        X_ptr[0] = z;
        X_ptr[1] = temp.x;
        X_ptr[2] = temp.y;
        X_ptr[3] = 1.0f;
    }

    _x_bar /= data_size;
    _xx_bar /= data_size;
    _y_bar /= data_size;
    _yy_bar /= data_size;
    _xy_bar /= data_size;
    _z_bar /= data_size;
    _zz_bar /= data_size;
    _xz_bar /= data_size;
    _yz_bar /= data_size;
}

float FitCircle::kasaCostFunc(const Points &_data,cv::Vec3f _circle){
    float cost=.0f;
    for(size_t i=0;i<_data.size();i++){
        float term1 = _data[i].x - _circle[0];
        float term2 = _data[i].y - _circle[1];
        cost += fabs(term1*term1 + term2*term2 - _circle[2]*_circle[2]);
    }
    return cost*100.0f/((float)_data.size()*_circle[2]*_circle[2]);
}

cv::Vec4f FitCircle::kasaMethod(const Points &_data){
    //sample mean notation
    float x_bar=.0f;
    float xx_bar=.0f;
    float y_bar=.0f;
    float yy_bar=.0f;
    float xy_bar=.0f;
    float xz_bar=.0f;
    float yz_bar=.0f;
    float z_bar=.0f;
    float zz_bar=.0f;
    cv::Mat X;
    float x_avg,y_avg;
    sampleMean(_data,x_bar,xx_bar,
               y_bar,yy_bar,xy_bar,
               xz_bar,yz_bar,z_bar,zz_bar,
               x_avg,y_avg,X);

    cv::Mat A = (cv::Mat_<float>(3,3) << xx_bar, xy_bar, x_bar,
                                   xy_bar,yy_bar,y_bar,
                                   x_bar,y_bar,1.0f);

    cv::Mat b = (cv::Mat_<float>(3,1) << -1.0f*xz_bar, -1.0f*yz_bar, -1.0f*z_bar);

    cv::Mat BCD = A.inv()*b;
    cv::Vec4f circle_param;
    float B = BCD.at<float>(0);
    float C = BCD.at<float>(1);
    float D = BCD.at<float>(2);
    circle_param[0] = (-1.0f*B)/2.0f + x_avg;
    circle_param[1] = (-1.0f*C)/2.0f + y_avg;
    circle_param[2] = sqrt(B*B + C*C - 4.0f*D)/2.0f;
    circle_param[3] = kasaCostFunc(_data, std::move(cv::Vec3f(circle_param[0],circle_param[1],circle_param[2])));

    return circle_param;
}

cv::Vec2d FitCircle::prattCharEq(float _eta, float _c2,float _c1,float _c0){
    cv::Vec2d result;
    float eta = _eta;
    float eta2 = eta*eta;
    float eta3 = eta*eta2;
    float eta4 = eta*eta3;
    result[0] = 4.0f*eta4 + _c2*eta2 + _c1*eta + _c0;
    result[1] = 16.0f*eta3 + 2.0f*_c2*eta + _c1;
    return result;
}

//include cost function
cv::Vec4f FitCircle::newtonPrattMethod(const Points &_data, int _max_steps, double _epsilon){
    float x_bar=.0f;
    float xx_bar=.0f;
    float y_bar=.0f;
    float yy_bar=.0f;
    float xy_bar=.0f;
    float xz_bar=.0f;
    float yz_bar=.0f;
    float z_bar=.0f;
    float zz_bar=.0f;
    cv::Mat X;

    cv::Vec4f not_circle;
    not_circle[0] = -1.0f;
    not_circle[1] = -1.0f;
    not_circle[2] = -1.0f;
    not_circle[3] = -1.0f;

    float x_avg,y_avg;

    sampleMean(_data,x_bar,xx_bar,
               y_bar,yy_bar,xy_bar,
               xz_bar,yz_bar,z_bar,zz_bar,
               x_avg,y_avg,X);

    float c2 = -1.0f*zz_bar - 3.0f*xx_bar*xx_bar - 3.0f*yy_bar*yy_bar -4.0f*xy_bar*xy_bar - 2.0f*xx_bar*yy_bar;
    float c1 = z_bar*(zz_bar - z_bar*z_bar) + 4.0f*z_bar*(xx_bar*yy_bar - xy_bar*xy_bar) - xz_bar*xz_bar - yz_bar*yz_bar;
    float c0 = xz_bar*xz_bar*yy_bar + yz_bar*yz_bar*xx_bar - 2.0f*xz_bar*yz_bar*xy_bar - (xx_bar*yy_bar - xy_bar*xy_bar)*(zz_bar - z_bar*z_bar);

    float curr_root= .0f;
    float next_root= .0f;

//    std::cout << c2 << " , " << c1 << " , " << c0 << std::endl;

    for(int i=0;i<_max_steps;i++){
        cv::Vec2d result = prattCharEq(curr_root,c2,c1,c0);
        if(result[1] <= _epsilon)break;
        next_root = curr_root - result[0]/result[1];

        if(fabs((next_root-curr_root)/next_root) < _epsilon)break;

        curr_root=next_root;
    }

    cv::Mat matB = cv::Mat::zeros(4,4,CV_32FC1);
    matB.at<float>(3) = -2.0f * next_root;
    matB.at<float>(5) = 1.0f * next_root;
    matB.at<float>(10) = 1.0f * next_root;
    matB.at<float>(12) = -2.0f * next_root;

//    std::cout << X << std::endl;
    if(X.empty())
        return not_circle;

    cv::Mat XtX = X.t()*X;
    cv::Mat temp_mat = XtX - matB;

//    std::cout << temp_mat << std::endl;

    cv::Mat u,w,vt;
    cv::SVD::compute(temp_mat,w,u,vt);

    cv::normalize(w,w,0,1,cv::NORM_MINMAX,CV_32FC1);

    cv::Mat non_zero_sv = w > _epsilon;
    int rank_temp_mat = cv::countNonZero(non_zero_sv);

//    std::cout << w << std::endl;
//    std::cout << u << std::endl;
//    std::cout << vt << std::endl;
//    std::cout << rank_temp_mat << std::endl;

    if(rank_temp_mat == vt.cols)
        return not_circle;

    cv::Mat ABCD = vt.row(rank_temp_mat).clone();
//    cv::Mat tes = ABCD * matB;
//    std::cout << "CHECK : " << tes*ABCD.t() << std::endl;
//    std::cout << "RANK : " << rank_temp_mat << std::endl;

    float A,B,C,D;
    A = ABCD.at<float>(0);
    B = ABCD.at<float>(1);
    C = ABCD.at<float>(2);
    D = ABCD.at<float>(3);

//    Mat ABCD(4,1,CV_32FC1);
//    Mat blank = Mat::zeros(4,1,CV_32FC1);

//    solve(temp_mat,blank,ABCD,DECOMP_SVD);

//    float A=1,B,C,D,next_A,next_B,next_C,next_D;
//    B=0.5;
//    C=D=next_A=next_B=next_C=next_D=0;
//    A = ABCD.at<float>(0);
//    C = ABCD.at<float>(1);
//    B = ABCD.at<float>(2);
//    D = ABCD.at<float>(3);

//    for(int i=0;i<100;i++){
//        next_A = -1*(temp_mat.at<float>(1)*B + temp_mat.at<float>(2)*C + temp_mat.at<float>(3)*D)/temp_mat.at<float>(0);
//        next_B = -1*(temp_mat.at<float>(4)*next_A + temp_mat.at<float>(6)*C + temp_mat.at<float>(7)*D)/temp_mat.at<float>(5);
//        next_C = -1*(temp_mat.at<float>(8)*next_A + temp_mat.at<float>(9)*next_B + temp_mat.at<float>(11)*D)/temp_mat.at<float>(10);
//        next_D = -1*(temp_mat.at<float>(12)*next_A + temp_mat.at<float>(13)*next_B + temp_mat.at<float>(14)*next_C)/temp_mat.at<float>(15);

//        B = next_B;
//        C = next_C;
//        D = next_D;
//    }

    cv::Vec4f circle_param;

//    cout << A << " , " << B << " , " << C << " , " << D << endl;

    circle_param[0] = (-1.0f*B)/(2.0f*A) + x_avg;
    circle_param[1] = (-1.0f*C)/(2.0f*A) + y_avg;
    circle_param[2] = sqrt((B*B + C*C - 4.0f*A*D)/(4.0f*A*A));

    float total_cost = .0f;

//    for(size_t i=0;i<_data.size();i++){
//        float *X_ptr = X.ptr<float>(i);
//        float cost = A*X_ptr[0] + B*X_ptr[1] + C*X_ptr[2] + D;
//        total_cost += cost*cost;
//    }

//    cv::Vec3f cp;
//    cp[0] = circle_param[0];
//    cp[1] = circle_param[1];
//    cp[2] = circle_param[2];
    total_cost = kasaCostFunc(_data, std::move(cv::Vec3f(
                                                   circle_param[0],
                                                   circle_param[1],
                                                   circle_param[2])));

    circle_param[3] = total_cost;

    return circle_param;

}
