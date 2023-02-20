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
#define R_HIP_P    9
#define L_HIP_P    10
#define R_HIP_R    11
#define L_HIP_R    12
#define R_KNEE     13
#define L_KNEE     14
#define R_ANK_R    15
#define L_ANK_R    16
#define R_ANK_P    17
#define L_ANK_P    18
#define HEAD_PAN   19
#define HEAD_TILT  20
    

namespace alfarobi {

// struct joint{

// } joints;

struct joint_value {    
    /* members */
    double  val[20],
            target_time;
    bool    torque_enabled[20];

    /* constructor */
    joint_value() {
        for (uint8_t i = 0; i < 20; ++i) {
            val[i]  = 0.0;
            torque_enabled[i] = false;
        }
        target_time = 0.0;
    }

    /* methods */
    uint8_t getIdByName(uint16_t joint_name) {
        return joint_name;
    }

    double getVal(int index) {
        return val[index];
    }

    void setVal(int index, double nval) {
        val[index] = nval;
    }
};


}