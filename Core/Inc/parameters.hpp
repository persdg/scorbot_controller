#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#define ENC_PER_ROUND 988

#define ENC_1_DIV   1.0  // Motor 1 divider for encoder error in PID mode
#define ENC_2_DIV   1.0  // Motor 2 divider for encoder error in PID mode
#define ENC_3_DIV   1.0  // Motor 3 divider for encoder error in PID mode
#define ENC_4_DIV   1.0  // Motor 4 divider for encoder error in PID mode
#define ENC_5_DIV   1.0  // Motor 5 divider for encoder error in PID mode
#define ENC_6_DIV   1.0  // Motor 6 divider for encoder error in PID mode

#define KU			630.0
#define TU			0.16666

//#define PID_1_KP    KU*0.6 	  // Motor 1 PID proportional coefficient
//#define PID_1_KI    TU*0.5    // Motor 1 PID integral coefficient
//#define PID_1_KD    TU*0.125  // Motor 1 PID derivative coefficient
#define PID_1_KP	250.0
#define PID_1_KI	0.0
#define PID_1_KD	15.0
#define PID_1_POLE  0.25      // Motor 1 PID dirty derivative pole
//#define PID_1_POLE  0.10
#define PID_1_SAT   20000.0   // Motor 1 PID integral saturation

#define PID_2_KP    300.0     // Motor 1 PID proportional coefficient
#define PID_2_KI    100       // Motor 1 PID integral coefficient
#define PID_2_KD    200       // Motor 1 PID derivative coefficient
#define PID_2_POLE  0.1       // Motor 1 PID dirty derivative pole
#define PID_2_SAT   10000.0   // Motor 1 PID integral saturation

#define PID_3_KP    300.0     // Motor 1 PID proportional coefficient
#define PID_3_KI    100       // Motor 1 PID integral coefficient
#define PID_3_KD    200       // Motor 1 PID derivative coefficient
#define PID_3_POLE  0.1       // Motor 1 PID dirty derivative pole
#define PID_3_SAT   10000.0   // Motor 1 PID integral saturation

#define PID_4_KP    300.0     // Motor 1 PID proportional coefficient
#define PID_4_KI    100       // Motor 1 PID integral coefficient
#define PID_4_KD    200       // Motor 1 PID derivative coefficient
#define PID_4_POLE  0.1       // Motor 1 PID dirty derivative pole
#define PID_4_SAT   10000.0   // Motor 1 PID integral saturation

#define PID_5_KP    300.0     // Motor 1 PID proportional coefficient
#define PID_5_KI    100       // Motor 1 PID integral coefficient
#define PID_5_KD    200       // Motor 1 PID derivative coefficient
#define PID_5_POLE  0.1       // Motor 1 PID dirty derivative pole
#define PID_5_SAT   10000.0   // Motor 1 PID integral saturation

#define PID_6_KP    300.0     // Motor 1 PID proportional coefficient
#define PID_6_KI    100       // Motor 1 PID integral coefficient
#define PID_6_KD    200       // Motor 1 PID derivative coefficient
#define PID_6_POLE  0.1       // Motor 1 PID dirty derivative pole
#define PID_6_SAT   10000.0   // Motor 1 PID integral saturation

#endif /* PARAMETERS_HPP_ */
