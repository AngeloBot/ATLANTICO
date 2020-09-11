#ifndef DEF_SYSTEM_H_INCLUDED
#define DEF_SYSTEM_H_INCLUDED

//Servo
#define pos_zero 90 //define posição zero
#define servoPin 18 //define pin do servo
#define leme_min -30 //max de -30 graus
#define leme_max 30 //max de 30 graus
#define servo_min 30
#define servo_max 150
#define delta_servo 45 //variação máxima de 45 graus entre cada movimento de servo

//PID
#define Kp 2.0
#define Ki 0.05
#define Kd 0.5

//margens de trabalho
#define lambda_rumo 2 //poço de erro em relação ao rumo ideal
#define lambda_jibe 10 //poço de erro de rumo em relação ao rumo ideal ao dar jibe
#define ang_jibe 110 //diferença entre rumo real e ideal para realizar o jibe
#define constante 10

//buss
#define BUSS_X_OFFSET 200
#define BUSS_Y_OFFSET -230

//GPS
#define waypoint_radius 3 //em metros
#define waypoint_num 2 //numero de waypoints na lista

//hall
//pin para input proveniente sensores
#define pinLEFT 34
#define pinRIGHT 35 
#define pinFLEFT 32
#define pinFRIGHT 33
#define maxHallSignal 30 //variável a ser calibrada para valor máximo q atinge a leitura analógica do sensor hall quando ele detecta campo magnético

//mostradores
#define LED_Hall 2 //builtin led
#define LED_GPS 4
#define LED_Buss 19

#endif //DEF_SYSTEM_H_INCLUDED
