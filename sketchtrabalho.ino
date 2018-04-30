#include <PWMServo.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Adafruit_HMC5883_U.h>
#include <Wire.h> 

// servo - marrom gnd, vermelho vcc, laranja 9
// gps - rx na 7, tx na 6, desconectar antes de compilar
// bussola - scl na A5, sda na A4
// quando carregar o sketch desconectar pins seriais

//portas dos sensores
// 1: suply 5V; 2: ground; 3:output   frente escrita, esquerda pra dreita
int FR = 2;
int R = 4;
int L = 8;
int FL = 12;
 
 
PWMServo servo; // Variável Servo
 
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);   // SCL na A5, SDA na A4

float lat2_lon2_desvio[] = {-23.578426, -46.744687, -21.32, -23580636, -46.743040, -21.32};   //lista de latitude, longitude e desvio magnético dos waypoints

   float right1 = 0; 
   float far_right1 = 0; 
   float left1 = 0; 
   float far_left1 = 0;
   float pos = 90;
   float rumo_ideal;
   float rumo_real;
   float rumo_real1;
   float delta_rumo;
   float delta_rumo2;
   float erro_rumo;
   float erro_rumo2;
   bool em_bordo = false;
   bool estava_contra = false;
   float far_left = 0;
   float left = 0;
   float right = 0;
   float far_right = 0;
   float contador_pane_bordo = 0;
   float contador_pane_contra = 0;
   float distancia_waypoint = 20;   //em metros
   float distancia;
   float latitude, longitude; //As variaveis podem ser float, para não precisar fazer nenhum cálculo
   int i; //Variável para contagem
   float precisao; //Variável para melhorar a precisao do valor aferido
   int contador_lista = 0;
   float marcador_hall_bussola = 0;
   
SoftwareSerial serial1(6, 7); // RX, TX
TinyGPS gps1;



void setup() {

   serial1.begin(9600);
   Serial.begin(9600);
   
   pinMode(FR,INPUT);
   digitalWrite(FR, HIGH);
   pinMode(R,INPUT);
   digitalWrite(R, HIGH);
   pinMode(L,INPUT);
   digitalWrite(L, HIGH);
   pinMode(FL,INPUT);
   digitalWrite(FL, HIGH);

   Serial.println("O GPS está aguardando pelo sinal dos satelites...");
   
   servo.attach(SERVO_PIN_A);  //Define a porta a ser ligada ao servo, a 9
   servo.write(90); // Inicia motor posição zero
   Serial.println("servo iniciado em 90");
 
 
 // inicializa a bússola e aguarda pelo sinal
  if(!mag.begin()){

    while(1);
  }

}



void loop() {
  
  //GPS//-------------------------------------------------------------------------------------------------------------------------------
   
  
  bool recebido = false;
  static unsigned long delayPrint;
 
  if (!em_bordo) {
  while (!recebido){
  while (serial1.available()) {
     char cIn = serial1.read();
     recebido = (gps1.encode(cIn) || recebido);  //Verifica até receber o primeiro sinal dos satelites
  }

 if ( (recebido) && ((millis() - delayPrint) > 10) ) {  //Mostra apenas após receber o primeiro sinal. Após o primeiro sinal, mostra a cada segundo.
     delayPrint = millis();
  
  //Latitude e Longitude----------------------------------------------------------------------------------------------------------------
     Serial.println("----------------------------------------");
     unsigned long idadeInfo;
     gps1.f_get_position(&latitude, &longitude, &idadeInfo);   //O método f_get_position é mais indicado para retornar as coordenadas em variáveis float, para não precisar fazer nenhum cálculo 
  
     if (latitude != TinyGPS::GPS_INVALID_F_ANGLE) {
        Serial.print("Latitude: ");
        Serial.println(latitude, 6);  //Mostra a latitude com a precisão de 6 dígitos decimais
     }

     if (longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
        Serial.print("Longitude: ");
        Serial.println(longitude, 6);  //Mostra a longitude com a precisão de 6 dígitos decimais
     }
     }
  }
 //Rumo Ideal---------------------------------------------------------------------------------------------------------------------------

     //rumo_ideal (calculado a partir da posiçao atual e do proximo waypoint) 
    (rumo_ideal) = gps1.course_to(latitude, longitude, lat2_lon2_desvio [contador_lista], lat2_lon2_desvio [1+contador_lista]);        //lat2 e lon2 numa lista de waypoints//
     Serial.print("Rumo Ideal (grau): ");
     Serial.println(rumo_ideal);    
  }
   
   //BUSSOLA//--------------------------------------------------------------------------------------------------------------------------
   
    precisao = 0; //Zera a variável para uma nova leitura

    for(i=0;i<20;i++) { //Faz a leitura 100 e armazenar a somatória

     sensors_event_t event; 
     mag.getEvent(&event);
     float heading = atan2(event.magnetic.y, event.magnetic.x);  //rumo indicado no eixo x da bússola
   
  
    //Converte o valor aferido para angulo
    if(heading < 0) {
      heading += 2*PI;
    } 

   if(heading > 2*PI) {
      heading -= 2*PI;
    }  

    rumo_real1 = heading * 180/M_PI;    //converte pra graus

    precisao += rumo_real1;    //vai somando na precisão pra depois dividir por 100
    delay(1);
  }
   
   rumo_real1 = precisao / 50; //Pega a somatória e tira a média dos valores aferidos
   rumo_real1 += lat2_lon2_desvio[2 + contador_lista];
   Serial.print("Rumo Real (grau): ");
   Serial.println(rumo_real1);


   rumo_real += rumo_real1;




  // HALL //----------------------------------------------------------------------------------------------------------------------------
  
   right1 += digitalRead(R);
   far_right1 += digitalRead(FR);
   left1 += digitalRead(L);
   far_left1 += digitalRead(FL);

     Serial.println(far_right1);
     Serial.println(right1);
     Serial.println(left1);
     Serial.println(far_left1);
   
   marcador_hall_bussola += 1;
   
   
     
     
     //PILOTO AUTOMÁTICO//--------------------------------------------------------------------------------------------------------------
     
    
    if (marcador_hall_bussola >= 2 && !em_bordo){     //precisa mudar o marcador lá embaixo
     Serial.println("iniciado piloto automatico"); 

     
     rumo_real = (rumo_real / marcador_hall_bussola);
     delta_rumo = rumo_real - rumo_ideal;
     
     Serial.print("delta_rumo = ");
     Serial.println(delta_rumo);

     
     if ((right1 / marcador_hall_bussola) <= 0.5) {
       right = 1;
       
     }
     else {
       right = 0;
       
     }
     if ((far_right1 / marcador_hall_bussola) <= 0.5) {
       far_right = 1;

     }
     else{
       far_right = 0;
       
     } 
     if ((left1  / marcador_hall_bussola) <= 0.5) {
       left = 1;
       
     }
     else {
       left = 0;
       
     } 
     if ((far_left1 / marcador_hall_bussola)  <= 0.5) {
       far_left = 1;
       
     }
     else{
       far_left = 0;
       
     }
     

     
     erro_rumo = 0;
     
       if (((delta_rumo) < 0 && (delta_rumo) >= -180) || ((delta_rumo) > 0 && (delta_rumo) <= 180)) {
        erro_rumo = abs(delta_rumo);
       }
     
       if (((delta_rumo) < 360  && (delta_rumo) > 180) || ((delta_rumo) > -360 && (delta_rumo) < -180)) {
        erro_rumo = abs(abs(delta_rumo) - 360);
       }

       if ((delta_rumo <= 10 && delta_rumo >= -10) || (delta_rumo <= -350) || (delta_rumo >= 350)){
       erro_rumo = 0;
      }
       
       Serial.print("erro_rumo = ");
       Serial.println(erro_rumo);
     
       //VENTO FAVORAVEL//--------------------------------------------------------------------------------------------------------------
       if ((far_left != 1 && left != 1) && (right != 1 && far_right != 1)) {
        
          contador_pane_contra = 0;
     
          if (((delta_rumo) < 0 && (delta_rumo) > -180) || ((delta_rumo) < 360  && (delta_rumo) > 180) ){    //virar pra boreste
            if (!estava_contra) {
              pos = (90 + erro_rumo);            // talvez tenha que mudar o multiplicador
              Serial.println("virou a boreste no piloto automatico favoravel");
            }
            else {     //orça um pouco, pq antes estava no contravento e o erro_rumo é grande
              pos = 100;
              Serial.println("orçou para boreste no piloto automatico favoravel");
            }
          }
          if (((delta_rumo) > 0 && (delta_rumo) < 180) || ((delta_rumo) > -360 && (delta_rumo) < -180) ){      //virar pra bombordo
            if (!estava_contra) {
              pos = (90 - erro_rumo);
              Serial.println("virou a bombordo no piloto automatico favoravel");
            }
            else {
               pos = 80;      //orça um pouco, pq antes estava no contravento e o erro_rumo é grande
               Serial.println("orçou a bombordo no piloto automatico favoravel");
            }   
         }
          estava_contra = false;
       }
     
      //VENTO CONTRA POR BORESTE//------------------------------------------------------------------------------------------------------
      
      if (far_right == 1 || right == 1) {
        
         if (((delta_rumo) > 0 && (delta_rumo) < 180) || ((delta_rumo) > -360 && (delta_rumo) < -180)) {      //virar pra bombordo
         pos = 80;
         Serial.println("arribou pq o rumo ideal está mais aberto");
         contador_pane_contra = 0;
         }
         
         if (far_right == 1 && right == 1) {
         pos = 80;
         Serial.println("arribou a bombordo");
         contador_pane_contra = 0;
         }
         
         if (far_right == 0 && right == 1) {
         pos = 55;
         Serial.println("arribou incisivamente a bombordo");
         contador_pane_contra += 1;
         Serial.print("contador pane contra = ");
         Serial.println(contador_pane_contra);
         }

         if (far_right == 1 && right == 0){
         Serial.println("orçando com vento por boreste");
         contador_pane_contra = 0;
         }
         
    
      estava_contra = true;
      }
      
      //VENTO CONTRA POR BOMBORDO//-----------------------------------------------------------------------------------------------------
     
      if (far_left == 1 || left == 1) {

         if (((delta_rumo) < 0 && (delta_rumo) > -180) || ((delta_rumo) < 360  && (delta_rumo) > 180)) {    //virar pra boreste
         pos = 100;
         Serial.println("arribou pq o rumo ideal está mais aberto");
         contador_pane_contra = 0;
         }
      
         if (far_left == 1 && left == 1) {
         pos = 100;
         Serial.println("arribou a boreste");
         contador_pane_contra = 0;
         }
         
         if (far_left == 0 && left == 1) {
         pos = 115;
         Serial.println("arribou incisivamente a boreste");
         contador_pane_contra += 1;         
         Serial.print("contador pane contra = ");
         Serial.println(contador_pane_contra);
         }

         if (far_left == 1 && left == 0) {
         Serial.println("orçando com vento por bombordo");
         contador_pane_contra = 0;
         }
     
      estava_contra = true;
     }
    
   
   }
     
     

     
     //BORDO//--------------------------------------------------------------------------------------------------------------------------
     

     Serial.println(far_right);
     Serial.println(right);
     Serial.println(left);
     Serial.println(far_left);

    delta_rumo2 = rumo_real1 - rumo_ideal;   //não passa pela média
     
    if (((delta_rumo2) < 0 && (delta_rumo2) >= -180) || ((delta_rumo2) > 0 && (delta_rumo2) <= 180)) {
        erro_rumo2 = abs(delta_rumo2);
       }
     
    if (((delta_rumo2) < 360  && (delta_rumo2) > 180) || ((delta_rumo2) > -360 && (delta_rumo2) < -180)) {
        erro_rumo2 = abs(abs(delta_rumo2) - 360);
       }
       
    Serial.print("erro_rumo2 = ");
    Serial.println(erro_rumo2);
     
     //BORDO A BORESTE//----------------------------------------------------------------------------------------------------------------
     if (!em_bordo && far_right == 1 && right == 0 && (erro_rumo2) > 90 && (((delta_rumo) < -10 && (delta_rumo) > -180) || ((delta_rumo) < 350  && (delta_rumo) > 180) ) ){
           pos = 179;
           servo.write(pos);
           Serial.println("comando de bordo para boreste");
           delay(1000);
           em_bordo = true;
         }
    
     //BORDO A BOMBORDO//---------------------------------------------------------------------------------------------------------------
     
     if (!em_bordo && far_left == 1 && left == 0 && erro_rumo2 > 90 && (((delta_rumo) > 10 && (delta_rumo) < 180) || ((delta_rumo) > -350 && (delta_rumo) < -180))){
            pos = 1;
            servo.write(pos);
            Serial.println("comando de bordo para bombordo");
            delay(1000);
            em_bordo = true;
     }
     
     //FIM DO BORDO//-------------------------------------------------------------------------------------------------------------------
     
     if (em_bordo) {
        contador_pane_bordo += 1;
        delay(250);
        Serial.print("contador pane bordo = ");
        Serial.println(contador_pane_bordo);
        if (erro_rumo2 < 23) {     //talvez seja melhor usar aqui que o hall oposto ligue
           pos = 90;
           servo.write(pos);
           Serial.println("término de bordo");
           delay(300);
           em_bordo = false;
           contador_pane_bordo = 0;
           }
     }
     
     //MODO PANE//----------------------------------------------------------------------------------------------------------------------
     
     if ((contador_pane_bordo > 20) || (contador_pane_contra > 20)) {
         pos = 90;
         em_bordo = false;
         estava_contra = false;
         servo.write(pos);
         Serial.println("modo pane acionado");
         delay(10000);
         contador_pane_bordo = 0;
         contador_pane_contra = 0;
         marcador_hall_bussola = 0;right = 0;
         far_right = 0;
         left = 0;
         far_left = 0;
         right1 = 0;
         far_right1 = 0;
         left1 = 0;
         far_left1 = 0;
         
     } 
     
     //MOVE SERVO//---------------------------------------------------------------------------------------------------------------------
     
     if (pos > 180) {     //impede angulos impossiveis pro servo
         pos = 180;
         Serial.println("leme corrigido para 180");
         }
     if (pos < 0) {
         pos = 0;
         Serial.println("leme corrigido para 0");
         }
         
     if (!em_bordo && pos != 90) {
         servo.write(pos);
         Serial.print("pos = ");
         Serial.println(pos);
         Serial.println("leme acionado em piloto automatico");
         delay(1500);       //talvez tenha que aumentar esse delay pra dar tempo do barco responder
         pos = 90;
         servo.write(pos);
         delay(10);
         
         }              //talvez tenha que voltar um pouco o leme depois que mover
     
     if (marcador_hall_bussola >= 2 && !em_bordo){
     right = 0;
     far_right = 0;
     left = 0;
     far_left = 0;
     right1 = 0;
     far_right1 = 0;
     left1 = 0;
     far_left1 = 0;
     rumo_real = 0;
     marcador_hall_bussola = 0;
      }
     
     //PROXIMO WAYPOINT-----------------------------------------------------------------------------------------------------------------
     
     distancia = gps1.distance_between(latitude, longitude, lat2_lon2_desvio [contador_lista], lat2_lon2_desvio [1+contador_lista]);
     if (distancia < distancia_waypoint) {
     contador_lista += 3;
     Serial.println("NOVO WAYPOINT");
     }
    

}
