#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "joint_value.h"                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_TORQUE_ENABLE          64                       // Control table address is different in Dynamixel model
#define ADDR_GOAL_POSITION          116 
#define ADDR_PRESENT_POSITION       132 
#define ADDR_GOAL_VELOCITY          104
#define ADDR_PROFILE_VELOCITY       112
#define ADDR_PROFILE_ACCELERATION   108
#define ADDR_PRESENT_VELOCITY       128
#define ADDR_MOVING                 122

// Data Byte Length
#define LEN_GOAL_POSITION           4
#define LEN_PRESENT_POSITION        4
#define LEN_GOAL_VELOCITY           4
#define LEN_MOVING                  1

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
// #define DXL1_ID                         14                   // Dynamixel#1 ID: 1
// #define DXL2_ID                         11                   // Dynamixel#2 ID: 2
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0                   // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

namespace alfarobi{

class ServoController
{
private:
    uint8_t dxl_id[20];
    joint_value servo;

    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // // Initialize GroupSyncWrite instance
    // dynamixel::GroupSyncWrite groupSyncWrite(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t addr_goal_pos, uint8_t len_goal_pos);

    // // Initialize Groupsyncread instance for Present Position
    // dynamixel::GroupSyncRead groupSyncRead(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t addr_present_pos, uint8_t len_present_pos);

    int dxl_comm_result;               // Communication result
    bool dxl_addparam_result;          // addParam result
    bool dxl_getdata_result;           // GetParam result
    int dxl_goal_position[20];         // Goal position, ganti disini kalau mau gerakin sesuai yg kita mau

    uint8_t dxl_error;                 // Dynamixel error
    uint8_t param_goal_position[4];
    uint8_t param_goal_velocity[4];
    int32_t dxl_present_position[20];  // Present position, tambah/kurang jika nambah/ngurang servo
    int32_t dxl_present_velocity[20];
    float dxl_pres_pos;
    bool dxl_is_moving;
    int32_t dxl_pres_vel;

public:
    ServoController();
    ~ServoController();

    void torqueEnable();
    void read(uint8_t dxl_id);
    void readVel(uint8_t dxl_id);
    bool isMoving(uint8_t dxl_id);
    void write(uint8_t dxl_id, double goal_pos, double goal_vel);
    void writeVel(uint8_t dxl_id, int goal_vel);
    int deg2Bit(float goal_pos_degree);
    int vel2Bit(float goal_vel);
};
}
#endif