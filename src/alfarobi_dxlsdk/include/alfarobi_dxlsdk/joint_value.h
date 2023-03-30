#ifndef JOINT_VALUE_H
#define JOINT_VALUE_H

#include <unordered_map>
#include <iostream>
#include <bits/stdc++.h>
#include <string>

 
#define R_SHO_P    1
#define L_SHO_P    2
#define R_SHO_R    3
#define L_SHO_R    4
#define R_ELB      5
#define L_ELB      6
#define R_HIP_Y    7
#define L_HIP_Y    8
#define R_HIP_R    9
#define L_HIP_R    10
#define R_HIP_P    11
#define L_HIP_p    12
#define R_KNEE     13
#define L_KNEE     14
#define R_ANK_P    15
#define L_ANK_P    16
#define R_ANK_R    17
#define L_ANK_R    18
#define HEAD_PAN   19
#define HEAD_TILT  20
    

namespace alfarobi {

// struct joint{

// } joints;

struct joint_value {    
    /* members */
    double  val[20],
            // pos[20],
            goal[20],
            target_time[20], 
            pause_time[20];
    bool    torque_enabled[20], 
            write[20],
            read[20];
    std::string name[20] = {
        "r_sho_p",
        "l_sho_p",
        "r_sho_r",
        "l_sho_r",
        "r_el",
        "l_el",
        "r_hip_y",
        "l_hip_y",
        "r_hip_r",
        "l_hip_r",
        "r_hip_p",
        "l_hip_p",
        "r_knee",
        "l_knee",
        "r_ank_p",
        "l_ank_p",
        "r_ank_r",
        "l_ank_r",
        "head_pan",
        "head_tilt",
    };

    /* constructor */
    joint_value() {
        for (uint8_t i = 0; i < 20; ++i) {
            val[i]  = 0.0;
            torque_enabled[i] = false;
            write[i] = false;
            read[i] = false;
            target_time[i] = 0.0;
            pause_time[i] = 0.0;
        }
        
    }

    /* methods */
    uint8_t getIdByName(uint16_t joint_name) {
        return joint_name;
    }

    uint8_t getIdByString(std::string joint_name) {
        uint8_t i = 0;
        
        while(i < 20){
            if(name[i] == joint_name){
                break;
            }
            i++;
        }
        
        // if(i >= 20) {
        //     std::cout<<"WRONG JOINT NAME !!!\n";
        // }
        return i;
    }

    double getVal(int index) {
        return val[index];
    }

    void setVal(int index, double nval) {
        val[index] = nval;
    }
};


}
#endif