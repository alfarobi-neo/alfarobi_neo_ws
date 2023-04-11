#ifndef GLOBALS_H
#define GLOBALS_H

struct WalkParameter
{
  double Tstride;
  double alpha;
  double Zc;
};

//enum LegSupport
//{
//  LEG_RIGHT = 1,
//  lEG_LEFT = 0,
//  COM_SSP = 1,
//  COM_DSP = 0
//};

//enum WalkStatus
//{
//  WALK_IDLE,
//  WALK_DSP_INITIAL,
//  WALK_INITIAL_ACC,
//  WALK_SSP,
//  WALK_DSP,
//  WALK_FINISH,
//  LEFT_LEG,
//  RIGHT_LEG
//};

static const /*long*/ double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164;
static const /*long*/ double e = 2.71828182845904523536028747135266249775724709369995;
static const double degree_to_rad = pi / 180;
static const double rad_to_degree = 180 / pi;

static const double e9 = 1e+9;
static const double e6 = 1e+6;


#endif // GLOBALS_H
