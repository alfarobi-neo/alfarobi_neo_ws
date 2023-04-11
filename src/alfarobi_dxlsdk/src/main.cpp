// #include "../src/alfarobi_dxlsdk/servo_controller.cpp"
#include "alfarobi_dxlsdk/servo_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    alfarobi::ServoController *robot = new alfarobi::ServoController();
    

    float goal_pos1;
    float goal_pos2;
    float goal_time1;
    float goal_time2;
    char choose;
    ros::Rate rate(2);

    // ActionModule *test = new ActionModule();

    // test->queue();

    // robot->writeMovingThreeshold(1);
    // robot->writeMovingThreeshold(2);
    robot->initialize();
    robot->torqueEnable();
    while(ros::ok())
    {
        //input goal position
        std::cout<<"goal_pos1\t: ";
        std::cin>>goal_pos1;
        // std::cout<<"goal_pos2\t: ";
        // std::cin>>goal_pos2;

        //input goal time
        std::cout<<"goal_time1\t: ";
        std::cin>>goal_time1;
        // std::cout<<"goal_time2\t: ";
        // std::cin>>goal_time2;

        // When the Drive Mode(10) is Velocity-based Profile, Profile Velocity(112) sets the maximum velocity of the Profile.
        // When the Drive Mode(10) is Time-based Profile, Profile Velocity(112) sets the time span to reach the velocity (the total time) of the Profile.
        robot->write(20, robot->deg2Bit(goal_pos1), goal_time1); //max 132 if using velocity-based, in milliseconds if using time-based
        // robot->write(2, robot->deg2Bit(goal_pos2), goal_time2);

        std::cout<<"read?\t: ";
        std::cin>>choose;
        if(choose =='y')
        {
            robot->read(20);
            // robot->read(11);
        }
        std::cout<<"Continue?(y/n)\t: ";
        std::cin>>choose;
        if (choose == 'n' || choose == 'N')
            break;
    }
    robot->torqueDisable();
    robot->dispose();

    // delete robot;

    return 0;
}
