#include <PWMServo.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Adafruit_HMC5883_U.h>
#include <Wire.h> 

// servo - marrom gnd, vermelho vcc, laranja 9
// gps - rx na 6, tx na 7, desconectar antes de compilar
// bussola - scl na A5, sda na A4
// quando carregar o sketch desconectar pins seriais

//portas dos sensores
int FR = 0;
int R = 1;
int L = 2;
int FL = 3;
 
 
PWMServo servo; // Vari�vel Servo
 
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);   // SCL na A5, SDA na A4

float lat2_lon2_desvio[] = {-23.578426, -46.744687, -0.3726278, 0, 0, 0};   //lista de latitude, longitude e desvio magn�tico dos waypoints


   float pos = 90;
   //float soma_rumos = 0;
   //float marcador_rumos = 0;
   float rumo_ideal;
   float rumo_real;
   float rumo_medio;
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
   float distancia_waypoint = 20;
   float distancia;
   float latitude, longitude; //As variaveis podem ser float, para n�o precisar fazer nenhum c�lculo
   int i; //Vari�vel para contagem
   float precisao; //Vari�vel para melhorar a precisao do valor aferido
   int contador_lista = 0;
   float marcador_hall = 0;
   
SoftwareSerial serial1(6, 7); // RX, TX
TinyGPS gps1;



void setup() {

   serial1.begin(9600);
   Serial.begin(9600);
   
   pinMode(FR,INPUT);
   pinMode(R,INPUT);
   pinMode(L,INPUT);
   pinMode(FL,INPUT);
 

   Serial.println("O GPS est� aguardando pelo sinal dos satelites...");
   
   servo.attach(SERVO_PIN_A);  //Define a porta a ser ligada ao servo, a 9
   servo.write(90); // Inicia motor posi��o zero
   Serial.println("servo iniciado em 90");
 
 
 // inicializa a b�ssola e aguarda pelo sinal
  if(!mag.begin()){

    while(1);
  }

}



void loop() {
  
  //GPS//-------------------------------------------------------------------------------------------------------------------------------
   
  
  bool recebido = false;
  static unsigned long delayPrint;
  while (!recebido){
  while (serial1.available()) {
     char cIn = serial1.read();
     recebido = (gps1.encode(cIn) || recebido);  //Verifica at� receber o primeiro sinal dos satelites
  }

 if ( (recebido) && ((millis() - delayPrint) > 1000) ) {  //Mostra apenas ap�s receber o primeiro sinal. Ap�s o primeiro sinal, mostra a cada segundo.
     delayPrint = millis();
  
  //Latitude e Longitude----------------------------------------------------------------------------------------------------------------
     Serial.println("----------------------------------------");
     unsigned long idadeInfo;
     gps1.f_get_position(&latitude, &longitude, &idadeInfo);   //O m�todo f_get_position � mais indicado para retornar as coordenadas em vari�veis float, para n�o precisar fazer nenhum c�lculo 
  
     if (latitude != TinyGPS::GPS_INVALID_F_ANGLE) {
        Serial.print("Latitude: ");
        Serial.println(latitude, 6);  //Mostra a latitude com a precis�o de 6 d�gitos decimais
     }

     if (longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
        Serial.print("Longitude: ");
        Serial.println(longitude, 6);  //Mostra a longitude com a precis�o de 6 d�gitos decimais
     }
     }
  }
 //Rumo Ideal---------------------------------------------------------------------------------------------------------------------------

     //rumo_ideal (calculado a partir da posi�ao atual e do proximo waypoint) 
     rumo_ideal = gps1.course_to(latitude, longitude, lat2_lon2_desvio [contador_lista], (lat2_lon2_desvio [1+contador_lista]));        //lat2 e lon2 numa lista de waypoints//
     Serial.print("Rumo Ideal (grau): ");
     Serial.println(float(rumo_ideal), 2);    
   
   //BUSSOLA//--------------------------------------------------------------------------------------------------------------------------
   
    precisao = 0; //Zera a vari�vel para uma nova leitura

    for(i=0;i<500;i++) { //Faz a leitura 100 e armazenar a somat�ria

     sensors_event_t event; 
     mag.getEvent(&event);
     float heading = atan2(event.magnetic.y, event.magnetic.x);
   
     heading += lat2_lon2_desvio[2 + contador_lista];
  
    //Converte o valor aferido para angulo
    if(heading < 0) {
      heading += 2*PI;
    } 

   if(heading > 2*PI) {
      heading -= 2*PI;
    }  

    rumo_real = heading * 180/M_PI;    //converte pra graus

    precisao = precisao + rumo_real;    //vai somando na precis�o pra depois dividir por 100
    delay(1);
  }

   rumo_real = precisao / 100; //Pega a somat�ria e tira a m�dia dos valores aferidos
   Serial.print("Rumo Real (grau): ");
   Serial.println(rumo_real);
   
     //soma_rumos += rumo_real;
     //marcador_rumos ++;

  // HALL //----------------------------------------------------------------------------------------------------------------------------
  
   right += digitalRead(R);
   far_right += digitalRead(FR);
   left += digitalRead(L);
   far_left += digitalRead(FL);
   
   marcador_hall += 1;
  
     
     
     //PILOTO AUTOM�TICO//--------------------------------------------------------------------------------------------------------------
     
     //if (marcador_rumos >=2 && !em_bordo && marcador_hall >= 2){      //ver a velocidade dos dados de hall e rumo 
    
    if (marcador_hall >=2 && !em_bordo){
     Serial.println("iniciado piloto automatico"); 
     
     //rumo_medio = soma_rumos / marcador_rumos;
     rumo_medio = rumo_real;
     
     delta_rumo = rumo_medio - rumo_ideal;
     
     
     
       if ((right / marcador_hall) >= 0.5) {
       right = 1;
       Serial.println("right acionado");
       }
       if ((right / marcador_hall) < 0.5) {
       right = 0;
       }
        if ((far_right / marcador_hall) >= 0.5) {
       far_right = 1;
       Serial.println("far_right acionado");
       }
       if ((far_right / marcador_hall) < 0.5){
       far_right = 0;
       } 
       if ((left  / marcador_hall) >= 0.5) {
       left = 1;
       Serial.println("left acionado");
       }
       if ((left / marcador_hall)  < 0.5) {
       left = 0;
       } 
       if ((far_left / marcador_hall)  >= 0.5) {
       far_left = 1;
       Serial.println("far_left acionado");
       }
       if(( far_left / marcador_hall)  < 0.5){
       far_left = 0;
       }
     
     erro_rumo = 0;
     
       if (((delta_rumo) < -10 && (delta_rumo) > -180) || ((delta_rumo) > 10 && (delta_rumo) < 180)) {
        erro_rumo = abs(delta_rumo);
       }
     
       if (((delta_rumo) < 350  && (delta_rumo) > 180) || ((delta_rumo) > -350 && (delta_rumo) < -180)) {
        erro_rumo = abs(delta_rumo - 360);
       }
       
       Serial.print("erro_rumo = ");
       Serial.println(erro_rumo);
     
       //VENTO FAVORAVEL//--------------------------------------------------------------------------------------------------------------
       if (far_left == 0 && left == 0 && right == 0 && far_right == 0) {
     
          if (((delta_rumo) < -10 && (delta_rumo) > -180) || ((delta_rumo) < 350  && (delta_rumo) > 180) ){    //virar pra boreste
            if (!estava_contra) {
              pos = (90 + erro_rumo);            // talvez tenha que mudar o multiplicador
              Serial.println("virou a boreste no piloto automatico favoravel");
            }
            else {     //or�a um pouco, pq antes estava no contravento e o erro_rumo � grande
              pos = 95;
              Serial.println("or�ou para boreste no piloto automatico favoravel");
            }
          }
          if (((delta_rumo) > 10 && (delta_rumo) < 180) || ((delta_rumo) > -350 && (delta_rumo) < -180) ){      //virar pra bombordo
            if (!estava_contra) {
              pos = (90 - erro_rumo);
              Serial.println("virou a bombordo no piloto automatico favoravel");
            }
            else {
               pos = 85;      //or�a um pouco, pq antes estava no contravento e o erro_rumo � grande
               Serial.println("or�ou a bombordo no piloto automatico favoravel");
            }   
         }
          estava_contra = false;
       }
     
      //VENTO CONTRA POR BORESTE//------------------------------------------------------------------------------------------------------
      
      if (far_right == 1 || right == 1) {
        
         if (((delta_rumo) > 5 && (delta_rumo) < 180) || ((delta_rumo) > -355 && (delta_rumo) < -180)) {      //virar pra bombordo
         pos = 85;
         Serial.println("arribou pq o rumo ideal est� mais aberto");
         }
         
         if (far_right == 1 && right == 1) {
         pos = 85;
         Serial.println("arribou a bombordo");
         }
         
         if (far_right == 0 && right == 1) {
         pos = 70;
         Serial.println("arribou incisivamente a bombordo");
         contador_pane_contra += 1;
         Serial.print("contador pane contra = ");
         Serial.println(contador_pane_contra);
         }

         if (far_right == 1 && right == 0){
         Serial.println("or�ando com vento por boreste");
         }
         
    
      estava_contra = true;
      }
      
      //VENTO CONTRA POR BOMBORDO//-----------------------------------------------------------------------------------------------------
     
      if (far_left == 1 || left == 1) {

         if (((delta_rumo) < -5 && (delta_rumo) > -180) || ((delta_rumo) < 355  && (delta_rumo) > 180)) {    //virar pra boreste
         pos = 95;
         Serial.println("arribou pq o rumo ideal est� mais aberto");
         }
      
         if (far_left == 1 && left == 1) {
         pos = 95;
         Serial.println("arribou a boreste");
         }
         
         if (far_left == 0 && left == 1) {
         pos = 110;
         Serial.println("arribou incisivamente a boreste");
         contador_pane_contra += 1;         
         Serial.print("contador pane contra = ");
         Serial.println(contador_pane_contra);
         }

         if (far_left == 1 && left == 0) {
         Serial.print("or�ando com vento por bombordo");
         }
     
      estava_contra = true;
     }
    
         // soma_rumos = 0;
         // marcador_rumos = 0;
              
    right = 0;
    far_right = 0;
    left = 0;
    far_left = 0;
    marcador_hall = 0;
   }
     
     
     if (left == 0 && right == 0) {       //zera os marcadores de pane se o vento n�o est� mais contra
         contador_pane_contra = 0;
         }
         
     
     //BORDO//--------------------------------------------------------------------------------------------------------------------------
     
    delta_rumo2 = rumo_real - rumo_ideal;        //aqui usa o rumo_real, pq o rumo muda r�pido durante o bordo
     
    if (((delta_rumo2) < -10 && (delta_rumo2) > -180) || ((delta_rumo2) > 10 && (delta_rumo2) < 180)) {
        erro_rumo2 = abs(delta_rumo);
       }
     
    if (((delta_rumo2) < 350  && (delta_rumo2) > 180) || ((delta_rumo2) > -350 && (delta_rumo2) < -180)) {
        erro_rumo2 = abs(delta_rumo - 360);
       }
     
     //BORDO A BORESTE//----------------------------------------------------------------------------------------------------------------
     if (!em_bordo && far_right == 1 && right == 0 && (erro_rumo) > 90 && (((delta_rumo) < -10 && (delta_rumo) > -180) || ((delta_rumo) < 350  && (delta_rumo) > 180) ) ){
           pos = 180;
           servo.write(pos);
           Serial.println("comando de bordo para boreste");
           delay(3000);
           em_bordo = true;
         }
    
     //BORDO A BOMBORDO//---------------------------------------------------------------------------------------------------------------
     
     if (!em_bordo && far_left == 1 && left == 0 && erro_rumo > 90 && (((delta_rumo) > 10 && (delta_rumo) < 180) || ((delta_rumo) > -350 && (delta_rumo) < -180))){
            pos = 0;
            servo.write(pos);
            Serial.println("comando de bordo para bombordo");
            delay(3000);
            em_bordo = true;
     }
     
     //FIM DO BORDO//-------------------------------------------------------------------------------------------------------------------
     
     if (em_bordo) {
        contador_pane_bordo += 1;
        delay(250);
        Serial.print("contador pane bordo = ");
        Serial.println(contador_pane_bordo);
        if (erro_rumo2 < 10) {     //talvez seja melhor usar aqui que o hall oposto ligue
           pos = 90;
           servo.write(pos);
           Serial.println("t�rmino de bordo");
           delay(3000);
           em_bordo = false;
           contador_pane_bordo = 0;
           }
     }
     
     //MODO PANE//----------------------------------------------------------------------------------------------------------------------
     
     if ((contador_pane_bordo > 30) || (contador_pane_contra > 20)) {
         pos = 90;
         em_bordo = false;
         estava_contra = false;
         servo.write(pos);
         Serial.println("modo pane acionado");
         delay(10000);
         contador_pane_bordo = 0;
         contador_pane_contra = 0;
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
         
     if (!em_bordo) {
         servo.write(pos);
         //Serial.print("pos = ");
         //Serial.println(pos);
         // Serial.println("leme acionado em piloto automatico");
         delay(1500);       //talvez tenha que aumentar esse delay pra dar tempo do barco responder
         pos = 90;
         servo.write(pos);
         delay(1500);
         
         }              //talvez tenha que voltar um pouco o leme depois que mover
     
     
     
     //PROXIMO WAYPOINT-----------------------------------------------------------------------------------------------------------------
     
     distancia = gps1.distance_between(latitude, longitude, lat2_lon2_desvio [3+contador_lista], lat2_lon2_desvio [4+contador_lista]);
     if (distancia < distancia_waypoint) {
     contador_lista += 3;
     
     }
    

}