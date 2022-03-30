#ifndef __NMPCSETTINGS_H__
#define __NMPCSETTINGS_H__

//Model
#define VMAX 0.4
#define d_Rob 0.2236 //0.354??

//Controller
#define NU1 1
#define NU2 10
#define NU3 2

//Controller Weights
#define Le1 800
#define Le2 500
#define Le3 0.05

//RPROP
#define JSTOP 0.05
#define IMAX 15
#define DELTA 0.1
#define ETA_M 0.5 
#define ETA_P 1.2
#define STEP_MAX 0.1
#define STEP_MIN 0.000001
#define CURR_STEP 0.1

//Others
#define SIM_TIME_STEP 0.04
#define BFC 1
#define DVAL 0
#define PI 3.14159265

#endif 