#pragma once

#include <opencv2/core/core.hpp>
#include <geometry_msgs/Vector3.h>

#define ALFA
// #define ROBI
// #define ABI
// #define FARO

#define NASIONAL

typedef std::vector<cv::Point > Points;
typedef std::vector<cv::Vec3f > Vecs3;
typedef std::vector<cv::Vec4f > Vecs4;
typedef std::vector<geometry_msgs::Vector3 > Vectors3;

namespace Alfarobi{

enum ImageEncode{
    GRAY8Bit = 0,
    BGR8Bit = 1
};

#ifdef ABI
    static constexpr float NECK_X = .015f;
    static constexpr float NECK_Y = .0f;
    static constexpr float NECK_Z = .035f;
    static constexpr float NECK2HEAD_X = .03f;
    static constexpr float NECK2HEAD_Y = .0f;
    static constexpr float NECK2HEAD_Z = .06f;
#endif

#if defined ALFA || defined ROBI
    // static constexpr float NECK_X = .0225f;
    // static constexpr float NECK_Y = .0f;
    // static constexpr float NECK_Z = .0415f;
    // static constexpr float NECK2HEAD_X = .035f;
    // static constexpr float NECK2HEAD_Y = .0f;
    // static constexpr float NECK2HEAD_Z = .06f;
    static constexpr float NECK_X = .02f;
    static constexpr float NECK_Y = .0f;
    static constexpr float NECK_Z = .041f;
    static constexpr float NECK2HEAD_X = .036644f;
    static constexpr float NECK2HEAD_Y = .0f;
    static constexpr float NECK2HEAD_Z = .05455f;
#endif

    static constexpr int FRAME_WIDTH = 640;
    static constexpr int FRAME_HEIGHT = 480;

    static constexpr int POINTS_MAP_H = 1200;
    static constexpr int POINTS_MAP_W = 1800;

    static constexpr int FIT_CIRCLE_MAX_STEPS = 20;
    static constexpr double FIT_CIRCLE_EPS = 1e-12;

    static constexpr int RANSAC_NUM_SAMPLES = 3;
    static constexpr int RANSAC_MAX_ITER = 40;

}

namespace Math{
    static constexpr float PI = 3.14159265359f;
    static constexpr float TWO_PI = 2.0f * PI;
    static constexpr float PI_TWO = PI / 2.0f;
    static constexpr float THREE_PI_TWO = 3.0f * PI_TWO;
    static constexpr float DEG2RAD = PI/180.0f;
    static constexpr float RAD2DEG = 1.0f/DEG2RAD;
}

namespace Color{
    static const std::string RESET = "\033[0m";
    static const std::string RED = "\033[31m";
    static const std::string GREEN = "\033[32m";
}

namespace Field{

#ifdef NASIONAL
    static constexpr int FIELD_LENGTH = 900;
    static constexpr int FIELD_WIDTH = 600;
    static constexpr int GOAL_DEPTH = 60;
    static constexpr int GOAL_WIDTH = 260;
    static constexpr int GOAL_AREA_LENGTH = 100;
    static constexpr int GOAL_AREA_WIDTH = 500;
    static constexpr int PENALTY_MARK_DISTANCE = 210;
    static constexpr int CENTER_CIRCLE_DIAMETER = 150;
    static constexpr int BORDER_STRIP_WIDTH = 70;
#else // REGIONAL
    static constexpr int FIELD_LENGTH = 600;
    static constexpr int FIELD_WIDTH = 400;
    static constexpr int GOAL_DEPTH = 60;
    static constexpr int GOAL_WIDTH = 260;
    static constexpr int GOAL_AREA_LENGTH = 100;
    static constexpr int GOAL_AREA_WIDTH = 300;
    static constexpr int PENALTY_MARK_DISTANCE = 150;
    static constexpr int CENTER_CIRCLE_DIAMETER = 150;
    static constexpr int BORDER_STRIP_WIDTH = 70;
#endif
}
