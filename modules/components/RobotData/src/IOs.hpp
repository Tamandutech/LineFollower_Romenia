#ifndef IOS_HPP
#define IOS_HPP

//Sensores frontais

const int sPins[4] = {18, 19, 22, 23}; // array dos pinos digitais ligados ao MUX
#define s0Input 35 // analog pin ligado ao MUX 1
#define s1Input 33 // analog pin ligado ao MUX 2
#define sQuant 16 // quantidade de sensores
#define sQuantReading 4 // QUantidade de ultimas leituras que ficam salvas no robo

//sensores laterais

#define s_lat_esq 0
#define s_lat_dir 0

//sensores do corpo

#define s_c_esq 0
#define s_c_dir 0

//driver_motor

#define pwmA 13
#define pwmB 14

#define in_dir1	17
#define in_dir2 25

#define in_esq1 26
#define in_esq2 27

#define stby 0

// Brushless ~~ verificar a direção

#define brushless_dir 4
#define brushless_esq 16

//encoder

#define enc_eq_A 36
#define enc_eq_B 39
#define enc_dir_A 32
#define enc_dir_B 34

//LED

#define led_pin 2

// Buzzer

#define buzzer_pin 12

// IMU

#define IMU_SDA 21 // configurar depois
#define IMU_SCL 22 // configurar depois

#endif