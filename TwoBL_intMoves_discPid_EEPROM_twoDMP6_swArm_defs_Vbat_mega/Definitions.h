#define SERIAL_ENABLED
#ifdef SERIAL_ENABLED
#define DATAOUT
#define VBATSERIAL
//#define JSON
#endif
//#define USE_OLD_DMPGETYAWPITCHROLL

#define NO_CALIBRATION

#define FAST_PWM
//#define PHASE_PWM

#define CAM_MPU
//#define FR_MPU

//pin names
#define ROLLOUT1 5
#define ROLLOUT2 3
#define ROLLOUT3 2
#define PITCHOUT1 6
#define PITCHOUT2 7
#define PITCHOUT3 8
#define ENROLL 4
#define ENPITCH 9
#define ARMSW 18
#define LED 13

//BL_vars
//#define STEPS 255
#define STEPS 360
#define STARTINGOFFSET 100

#define VBATCHECKTIME 3000  ///30sec, based on 100Hz interrupt
