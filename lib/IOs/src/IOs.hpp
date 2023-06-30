#ifndef IOS_HPP
#define IOS_HPP

//Sensores frontais

const int sPins[4] = {0, 0, 0, 0}; // array dos pinos digitais ligados ao MUX
#define sInput 0 // analog pin ligado ao MUX
#define sQuant 16


//driver_motor

#define pwmA 0
#define pwmB 0

#define in_dir1	0
#define in_dir2 0

#define in_esq1 0
#define in_esq2 0

#define stby 0
 
//sensores_laterais

#define s_lat_esq 0
#define s_lat_dir 0


//encoder

#define enc_eq_A 0
#define enc_eq_B 0
#define enc_dir_A 0
#define enc_dir_B 0

#define led 0

#endif