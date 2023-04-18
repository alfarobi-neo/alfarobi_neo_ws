#include "img_source/img_source.h"

int main(int argc,char **argv){
    
    ros::init(argc,argv,"img_source_node");

    ImageSource image_source;

    ros::Rate loop_rate(30);

    while(ros::ok()){

        image_source.process();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
