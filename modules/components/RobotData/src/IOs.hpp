#ifndef IOS_HPP
#define IOS_HPP

//Sensores frontais

// MUX 1 e 2
const int sPins[4] = {18, 19, 22, 23}; // array dos pinos digitais ligados ao MUX
#define s0Input 35 // analog pin ligado ao MUX 1, sensores frontais
#define s1Input 33 // analog pin ligado ao MUX 2, sensores do corpo

// Quantidade ligadas aos sensores frontais
#define sQuant 16 // quantidade de sensores
#define sQuantReading 4 // QUantidade de ultimas leituras que ficam salvas no robo

//driver_motor

#define pwmA 13
#define pwmB 14

#define in_dir1	17
#define in_dir2 25

#define in_esq1 27
#define in_esq2 26

#define stby 999

// Brushless ~~ verificar a direção

#define brushless_dir 4 //4 ou 12
#define brushless_esq 16

//encoder

#define enc_eq_A 39
#define enc_eq_B 36
#define enc_dir_A 32
#define enc_dir_B 34

//LED

#define led_pin 2

// Buzzer

#define buzzer_pin 4 //12

// IMU

#define IMU_SDA 21 // configurar depois
#define IMU_SCL 22 // configurar depois

#endif