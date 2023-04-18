// --Comment other nodes main.cpp files if this node is going be used-- //
// #include <img_source/img_source.h>
// #include <v9_ball_detector/v9_ball_detector.h>
// #include <v10_goalpost_detector/v10_goalpost_detector.h>
// #include <v9_localization/v9_localization.h>

// int main(int argc,char **argv){
    
//     ros::init(argc,argv,"alfarobi_vision_node");

//     ImageSource image_source;
//     BallDetector ball_detector;
//     GoalpostDetector goalpost_detector;
//     Localization localization;

//     ros::Rate loop_rate(30);

//     while(ros::ok()){

//         image_source.process();
//         ball_detector.process();
//         goalpost_detector.process();
//         localization.process();

//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }