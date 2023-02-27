#include "alfarobi_dxlsdk/servo_controller.h"
// #include "../src/dynamixel_sdk/packet_handler.cpp"
// #include "../src/dynamixel_sdk/port_handler.cpp"
// #include "../src/dynamixel_sdk/packet_handler.cpp"
// #include "../src/dynamixel_sdk/port_handler.cpp"
// #include "../src/dynamixel_sdk/port_handler_linux.cpp"
// #include "../src/dynamixel_sdk/group_sync_read.cpp"
// #include "../src/dynamixel_sdk/group_sync_write.cpp"
// #include "../src/dynamixel_sdk/group_bulk_read.cpp"
// #include "../src/dynamixel_sdk/group_bulk_write.cpp"
// #include "../src/dynamixel_sdk/protocol1_packet_handler.cpp"
// #include "../src/dynamixel_sdk/protocol2_packet_handler.cpp"
//

alfarobi::ServoController::ServoController()
{
    dxl_id[0] = 1;
    dxl_id[1] = 2;
    dxl_id[2] = 3;
    dxl_id[3] = 4;
    dxl_id[4] = 5;
    dxl_id[5] = 6;
    dxl_id[6] = 7;
    dxl_id[7] = 8;
    dxl_id[8] = 9;
    dxl_id[9] = 10;
    dxl_id[10] = 11;
    dxl_id[11] = 12;
    dxl_id[12] = 13;
    dxl_id[13] = 14;
    dxl_id[14] = 15;
    dxl_id[15] = 16;
    dxl_id[16] = 17;
    dxl_id[17] = 18;
    dxl_id[18] = 19;
    dxl_id[19] = 20;

    dxl_comm_result = COMM_TX_FAIL;
    dxl_addparam_result = false;
    dxl_getdata_result = false;

    dxl_error = 0;

    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port! [%s]\n", DEVICENAME);
        return;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        return;
    }
}

alfarobi::ServoController::~ServoController()
{
    // Disable Dynamixels Torque
    for(int i=0;i<20;i++)
    {
        if((servo.getIdByName(R_ELB) == i-1) || (servo.getIdByName(L_ELB) == i-1))
        {
            printf("Servo doesn't exist!\n");
            continue;
        }
        // if((i == 4) || (i == 5)){
        //     printf("Servo doesn't exist!\n");
        //     continue;
        // }

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel#%d has been successfully disconnected \n", dxl_id[i] + 1);
        }
    }
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[13], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    // if (dxl_comm_result != COMM_SUCCESS)
    // {
    //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    //     std::cout<<"konzz\n";
    // }
    // else if (dxl_error != 0)
    // {
    //     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    // }
    // else
    // {
    //     printf("Dynamixel#%d has been successfully disconnected \n", dxl_id[13]);
    // }
    
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[10], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    // if (dxl_comm_result != COMM_SUCCESS)
    // {
    //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    //     std::cout<<"konzz\n";
    // }
    // else if (dxl_error != 0)
    // {
    //     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    // }
    // else
    // {
    //     printf("Dynamixel#%d has been successfully disconnected \n", dxl_id[10]);
    // }
    portHandler->closePort();
}

void alfarobi::ServoController::torqueEnable()
{
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    for(int i=0; i<19; i++)
    {
        if((servo.getIdByName(R_ELB) == i-1) || (servo.getIdByName(L_ELB) == i-1))
        {
            printf("Servo doesn't exist!\n");
            continue;
        }
        // if((i == 4) || (i == 5) )
        // {
        //     printf("Servo doesn't exist!\n");
        //     continue;
        // }

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel#%d has been successfully connected \n", dxl_id[i]);
        }
        // Add parameter storage for Dynamixel#1 present position value
        dxl_addparam_result = groupSyncRead.addParam(dxl_id[i]);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed\n", dxl_id[i]);
            // return;
        }
    }
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[13], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // if (dxl_comm_result != COMM_SUCCESS)
    // {
    //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    // }
    // else if (dxl_error != 0)
    // {
    //     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    // }
    // else
    // {
    //     printf("Dynamixel#%d has been successfully connected \n", dxl_id[13]);
    // }
    // // Add parameter storage for Dynamixel# present position value
    // dxl_addparam_result = groupSyncRead.addParam(dxl_id[13]);
    // if (dxl_addparam_result != true)
    // {
    //     fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", dxl_id[13]);
    //     return;
    // }

    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[10], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // if (dxl_comm_result != COMM_SUCCESS)
    // {
    //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    // }
    // else if (dxl_error != 0)
    // {
    //     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    // }
    // else
    // {
    //     printf("Dynamixel#%d has been successfully connected \n", dxl_id[10]);
    // }
    // // Add parameter storage for Dynamixel# present position value
    // dxl_addparam_result = groupSyncRead.addParam(dxl_id[10]);
    // if (dxl_addparam_result != true)
    // {
    //     fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", dxl_id[10]);
    //     return;
    // }
}

void alfarobi::ServoController::write(uint8_t dxl_id, double goal_pos, double goal_vel)
{
    dynamixel::GroupSyncWrite groupSyncWritePos(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    dynamixel::GroupSyncWrite groupSyncWriteVel(portHandler, packetHandler, ADDR_PROFILE_VELOCITY, LEN_GOAL_VELOCITY);

    // at 12V the velocity is 30 rev/min. As I understand that I should set the value of velocity profile to 132 (30 divided to 0.229 rev/min = 132)
    // goal velocity must not be 0
    // Allocate goal position value into byte array
    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(goal_vel));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(goal_vel));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(goal_vel));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(goal_vel));

    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_pos));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_pos));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_pos));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_pos));

    // Add Dynamixel# goal velocity value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteVel.addParam(dxl_id, param_goal_velocity);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", dxl_id);
      return;
    }
    
    // Add Dynamixel# goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWritePos.addParam(dxl_id, param_goal_position);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", dxl_id);
      return;
    }

    // Syncwrite goal velocity
    dxl_comm_result = groupSyncWriteVel.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWritePos.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWritePos.clearParam();
    // // Clear syncwrite parameter storage
    groupSyncWriteVel.clearParam();

}

// void ServoController::writeVel(uint8_t dxl_id, int goal_vel)
// {
//     dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);
//     // Allocate goal position value into byte array
//     param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(goal_vel));
//     param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(goal_vel));
//     param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(goal_vel));
//     param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(goal_vel));

//     // Add Dynamixel# goal position value to the Syncwrite storage
//     dxl_addparam_result = groupSyncWrite.addParam(dxl_id, param_goal_position);
//     if (dxl_addparam_result != true)
//     {
//       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", dxl_id);
//       return;
//     }

//     // Syncwrite goal position
//     dxl_comm_result = groupSyncWrite.txPacket();
//     if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

//     // Clear syncwrite parameter storage
//     groupSyncWrite.clearParam();
// }

void alfarobi::ServoController::read(uint8_t dxl_id)
{
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    dxl_addparam_result = groupSyncRead.addParam(dxl_id);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", dxl_id);
        return;
    }
    // Syncread present position
    dxl_comm_result = groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (groupSyncRead.getError(dxl_id, &dxl_error))
    {
    printf("[ID:%03d] %s\n", dxl_id, packetHandler->getRxPacketError(dxl_error));
    }

    // Check if groupsyncread data of Dynamixel# is available
    dxl_getdata_result = groupSyncRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    if (dxl_getdata_result != true)
    {
    fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id);
    return;
    }

    // Get Dynamixel# present position value
    dxl_pres_pos = groupSyncRead.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

    dxl_pres_pos = (dxl_pres_pos * 360)/4095;

    printf("[ID:%03d] PresPos:%03f\n", dxl_id, dxl_pres_pos);
}

void alfarobi::ServoController::readVel(uint8_t dxl_id)
{
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_GOAL_POSITION);
    dxl_addparam_result = groupSyncRead.addParam(dxl_id);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", dxl_id);
        return;
    }
    // Syncread present position
    dxl_comm_result = groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (groupSyncRead.getError(dxl_id, &dxl_error))
    {
    printf("[ID:%03d] %s\n", dxl_id, packetHandler->getRxPacketError(dxl_error));
    }

    // Check if groupsyncread data of Dynamixel# is available
    dxl_getdata_result = groupSyncRead.isAvailable(dxl_id, ADDR_PRESENT_VELOCITY, LEN_GOAL_POSITION);
    if (dxl_getdata_result != true)
    {
    fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id);
    return;
    }

    // Get Dynamixel# present position value
    dxl_pres_vel = groupSyncRead.getData(dxl_id, ADDR_PRESENT_VELOCITY, LEN_GOAL_POSITION);

    printf("[ID:%03d] PresentVel:%03d\n", dxl_id, dxl_pres_vel);
}

int alfarobi::ServoController::deg2Bit(float goal_pos_degree)
{
    return (goal_pos_degree/360) * 4095;
}

int alfarobi::ServoController::vel2Bit(float goal_vel)
{
    return (goal_vel/360) * 4095;
}